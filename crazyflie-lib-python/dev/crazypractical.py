# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
"""
import logging
import time
from threading import Timer
import datetime as dt

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

#  import PositionHlCommander (ADDED)
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.positioning.motion_commander import MotionCommander

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

import numpy as np
import os


class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self.count = 0
        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # paramter initialization
        self.x = 0
        self.y = 0
        self.z = 0
        self.position = np.zeros(3)
        self.stored_position = []

        self.default_height = 0.5 # default relative height during flight


        self.directions = [] # list storing the flight directions of WP navigation
        self.direction = [1] # current flight direction (1 at the beginning

        self.waypoints = np.zeros(1)

        self.delta = 0.25 # step size from the edge to the center of the box

        self.T = 175 # size of the convolution kernel (period of the sinus in 10ms)
        self.kernel = np.sin(np.arange(self.T-1)/self.T*2*np.pi) # sinusoidal kernel
        self.conv_thresh = -2.5 # threshold on edge detection

        self.ready = False # ready for flight after default height is reached
        #self.wp_reached = False

        self.edges = [] #  list of (x,y) components of the detected edges

        self.landing_site = np.array([0.4,0,self.default_height]) # coordinates of the landing site


        self.logs = np.zeros([10000,6]) # Initialize log variable

        self.is_connected = True # Variable used to keep main loop occupied until disconnect


    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        # the logging variable cannot log more than 6 variables together
        #self._lg_stab.add_variable('stabilizer.roll', 'float')
        #self._lg_stab.add_variable('stabilizer.pitch', 'float')
        #self._lg_stab.add_variable('stabilizer.yaw', 'float')


        # Add position values
        self._lg_stab.add_variable('acc.z','float')
        self._lg_stab.add_variable('stateEstimate.x','float')
        self._lg_stab.add_variable('stateEstimate.y','float')
        self._lg_stab.add_variable('stateEstimate.z','float')
        self._lg_stab.add_variable('range.zrange','float')
        self._lg_stab.add_variable('baro.pressure')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')


    def find_box(self, id):
        """ Algorithm to land the drone on the box found during flight"""
        # definitions of the coordinates of the WP
        x_min = 0.8
        y_center = 0
        x_max = 1.1
        y_min = -0.55
        y_max = 0.55
        vel = 0.15 # velocity of the drone

        # definition of waypoints: do one rectangular trajectory in CCW direction (in 2nd half of the map)
        self.waypoints = np.array([
                                [x_min, y_center, self.default_height],
                                [x_min,y_min,self.default_height],
                                [x_max,y_min,self.default_height],
                                [x_max,y_center,self.default_height],
                                [x_max,y_max,self.default_height],
                                [x_min,y_max,self.default_height],
                                [x_min,y_center,self.default_height]])

        # initialize directions list depending on defined WP
        self.directions = self.get_waypoint_directions()
        print('calculated directions', self.directions)

        # Sync with drone
        with SyncCrazyflie(id, cf=self._cf) as scf:
        # Send position commands
            with PositionHlCommander(scf,default_height=0.5) as pc:
                time.sleep(2)
                self.stored_position = [] # reset all stored position
                self.ready = True # ready for waypoint navigation
                for i,wp in enumerate(self.waypoints):
                    self.direction = self.directions[i]
                    targ_x = wp[0]
                    targ_y = wp[1]
                    targ_z = wp[2]
                    print('waypoint is:', wp)
                    pc.go_to(targ_x,targ_y,targ_z, vel)
                    time.sleep(1)

                    if (self.edge_detection()):
                        self.stored_position = [] # reset if edge detected
                        break

                    self.stored_position = [] # reset if no edge detected

                print('End of Waypoint navigation')
                print('Number of edges found: ', len(self.edges))

                edge1_x = self.edges[0][0]
                edge1_y = self.edges[0][1]
                print("Edge position:", self.edges[0])

                # move a distance delta further the position of the edge (usually this is the center of the box)
                center_x, center_y= self.get_to_center(self.edges[0])
                time.sleep(1)
                pc.go_to(center_x, center_y, self.default_height, vel)
                time.sleep(1)

                orthog_x, orthog_y = self.go_orthogonal(self.edges[0])
                pc.go_to(orthog_x, orthog_y, self.default_height, vel)
                time.sleep(2)
                self.stored_position =[]
                time.sleep(2)
                print('Position outside: ', self.position)
                search_x,search_y = self.search_for_second_edge(self.edges[0])
                pc.go_to(search_x, search_y, self.default_height,vel*2)
                time.sleep(2)

                # define the landing position
                land_x, land_y = self.find_landing_site(self.edge_detection())
                self.landing_site[0] = land_x
                self.landing_site[1] = land_y

                self.stored_position = []

                print('LANDING at position :', self.landing_site)
                # move to landing site
                time.sleep(2)
                pc.go_to(self.landing_site[0], self.landing_site[1], self.landing_site[2],vel)
                time.sleep(1)
                # decrease height before landing
                pc.go_to(self.landing_site[0], self.landing_site[1], 0.3,vel)
                time.sleep(1)
                pc.go_to(self.landing_site[0], self.landing_site[1], 0.2,vel)
                time.sleep(1)

        self._disconnected

    def get_waypoint_directions(self):
        """ returns a list of directions depending on self.waypoints.
                 1: x_pos
                 2: x_neg
                 3: y_pos
                 4: y_neg
        """
        directions = [1] # initial direction is 1
        for i in range(len(self.waypoints)-1):
            if (self.waypoints[i][0]!=self.waypoints[i+1][0]):
                if(self.waypoints[i][0]<self.waypoints[i+1][0]):
                    directions.append(1) # positive x direction
                else:
                    directions.append(3) # negative x direction
            else:
                if(self.waypoints[i][1]<self.waypoints[i+1][1]):
                    directions.append(2) # positive y direction
                else:
                    directions.append(4) # negative y direction
        return directions

    def get_to_center(self,edge):
        """ Navigates the drone to the center of the box when it arrives at the first edge"""
        if (self.direction%2==1): # moving in x direction
            if (self.direction==1): # moving in positive x direction
                x = edge[0] + self.delta
                y = edge[1]
            else:
                x = edge[0] - self.delta
                y = edge[1]
        else:
            if(self.direction==2): # positive y direction
                x = edge[0]
                y = edge[1]+self.delta
            else:
                x = edge[0]
                y = edge[1]-self.delta
        return x,y

    def go_orthogonal(self, edge):
        if (self.direction%2==1): # moving in x direction
            if (self.direction==1): # moving in positive x direction
                x = edge[0] + self.delta
                y = edge[1] + self.delta
            else:
                x = edge[0] - self.delta
                y = edge[1] - self.delta
        else:
            if(self.direction==2): # positive y direction
                x = edge[0] - self.delta
                y = edge[1] + self.delta
            else:
                x = edge[0] + self.delta
                y = edge[1] - self.delta
        self.direction+= 1
        if (self.direction ==5):
            self.direction = 1
        return x,y

    def search_for_second_edge(self, edge):
        if (self.direction%2==1):
            if (self.direction==1):
                x = edge[0]
                y = edge[1] - self.delta
            else:
                x = edge[0]
                y = edge[1] + self.delta
        else:
            if (self.direction==2):
                x = edge[0]+self.delta
                y = edge[1]
            else:
                x = edge[0] - self.delta
                y = edge[1]

        self.direction+=2
        if (self.direction==5):
            self.direction = 1
        if (self.direction == 6):
            self.direction = 2
        return x,y

    def find_landing_site(self,second_edge_found):
        first_edge = self.edges[0]
        if (len(self.edges)==2):
            second_edge = self.edges[1]

        if (second_edge_found):
            print('Second edge detected at:', self.edges[1])
            if (self.direction%2==1):
                if (self.direction==1):
                    x = second_edge[0]  + self.delta
                    y = first_edge[1]   + self.delta
                else:
                    x = second_edge[0] - self.delta
                    y = first_edge[1]  - self.delta
            else:
                if(self.direction==2):
                    x = first_edge[0]   - self.delta
                    y = second_edge[1]  + self.delta
                else:
                    x = first_edge[0]   +  self.delta
                    y = second_edge[1]  -  self.delta
        else: # no second edges
            if (self.direction%2==1):
                if (self.direction==1):
                    x = first_edge[0]  - self.delta/2
                    y = first_edge[1]   + self.delta
                else:
                    x = first_edge[0] + self.delta/2
                    y = first_edge[1]  - self.delta
            else:
                if(self.direction==2):
                    x = first_edge[0]   - self.delta
                    y = first_edge[1]   - self.delta/2
                else:
                    x = first_edge[0]   +  self.delta
                    y = first_edge[1]   +  self.delta/2
        return x,y

    def edge_detection(self):
        edge_detected = False
        if (self.ready):
            z = []
            for i in range(len(self.stored_position)):
                z.append(self.stored_position[i][2])
            z = np.asarray(z)
            conv_prod = [np.sum((z[i:i+self.T-1]-self.default_height)*self.kernel) for i in range(z.shape[0]-self.T)]

            is_edge = any(np.array(conv_prod)<self.conv_thresh)
            print('Edge detected: ', is_edge)
            print('minimum of correlation:', min(conv_prod))
            if (is_edge):
                idx = np.argmin(conv_prod)
                self.edges.append(self.stored_position[idx])
                edge_detected = True
        return edge_detected



    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        self.x = data['stateEstimate.x']
        self.y = data['stateEstimate.y']
        self.z = data['stateEstimate.z']
        self.position = np.array([self.x, self.y, self.z])
        self.stored_position.append(self.position)

        # Save info into log variable
        for idx,i in enumerate(list(data)):
            self.logs[self.count][idx] = data[i]
        self.count +=1

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

        # Get timestamp
        dtime = dt.datetime.now().strftime("%Y-%m-%d_%H_%M_%S")
        # Save log to file
        if not os.path.exists('logs'):
            os.makedirs('logs')
        np.savetxt('logs\\'+dtime+'.csv', self.logs, delimiter=',')


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    start = time.time()
    # enter crazyflie ID here
    id = 'radio://0/80/2M/E7E7E7E7E7'

    le = LoggingExample('radio://0/80/2M/E7E7E7E7E7')
    le.find_box(id)

    end = time.time()
    print('Total time required',end - start)
    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)
