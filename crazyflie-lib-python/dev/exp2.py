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

        self.x = 0
        self.y = 0
        self.z = 0
        self.default_height = 0.5

        self.wp_reached = False
        self.position = np.zeros(3)
        self.stored_position = []

        self.T = 175
        self.kernel = np.sin(np.arange(self.T-1)/self.T*2*np.pi)
        self.conv_thresh = 1.5

        self.ready = False
        self.edges = [] #  list of (x,y) components

        # Initialize log variable
        self.logs = np.zeros([10000,6])


        #self.fun.add_callback(self.somefun)

        #print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        #self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True
        #self.fly_square(link_uri)


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

        # Start a timer to disconnect in 5s
        # ADD TIMER HERE
        #t = Timer(5, self._cf.close_link)
        #t.start()

    # OWN IMPLEMENTS FOR WAYPOINT NAVIGATION

    def get_distance_to_waypoint(self, waypoint):
        distance = np.linalg.norm(waypoint-self.position)
        return distance

    def get_direction_to_waypoint(self, waypoint):
        direction = waypoint - self.position
        if (np.linalg.norm(direction)>0.3):
            direction = 0.3*direction/np.linalg.norm(direction)
        return direction[0], direction[1], direction[2]


    def fly_square(self, id):

        #def print_location(t):



        """ Example of simple logico to make the drone fly in a square
        trajectory at fixed speed"""

        """ mode defines what the drone does:
        1: use the position commmander
        2: use the motion commmander
        """
        mode = 1


        k_p = 1
        # set a list of waypoints

        x_min = 0.6
        y_center = 0
        x_max = 0.9
        y_min = -0.4
        y_max = 0.4
        vel = 0.1
        waypoints = np.array([#[0,0,self.default_height],
                            [x_min,y_center,self.default_height],
                            [x_min,y_max,self.default_height]])
        # 1: x_pos
        # 2: x_neg
        # 3: y_pos
        # 4: y_neg
        directions = [1,3]
        """[x_min,y_max,self.default_height],
                            [x_max,y_max,self.default_height],
                            [x_max,y_min,self.default_height],
                            [x_min,y_min,self.default_height],
                            [x_min,y_center,self.default_height]])"""

        # set relative goal position, add the initial position (not reset to 0)

        """goal_x = 0 + self.x
        goal_y = 0 + self.y
        goal_z = 0.5 + self.z
        """
        #t = Timer(0.1, self.func)
        #self._cf.close_link)
        #t.start()

        # Sync with drone
        with SyncCrazyflie(id, cf=self._cf) as scf:
            if (mode==1):
            # Send position commands
                with PositionHlCommander(scf,default_height=0.5) as pc: #default_height=0.5
                    # do something
                    time.sleep(2)

                    self.stored_position = []
                    self.ready = True

                    for i,wp in enumerate(waypoints):
                        targ_x = wp[0]
                        targ_y = wp[1]
                        targ_z = wp[2]
                        distance = self.get_distance_to_waypoint(wp)
                        dir = directions[i]
                        print('waypoint is:', wp)
                        pc.go_to(targ_x,targ_y,targ_z, vel)
                        distance = self.get_distance_to_waypoint(wp)

                        if (self.edge_detection()):
                            print('EDGE DETECTED')
                            print(self.edges)
                            if (len(self.edges)==1):
                                edge_x = self.edges[0][0]
                                edge_y = self.edges[0][1]
                                print("Edge position:")
                                print(edge_x, edge_y)
                                print("present position:")
                                print(self.position)

                                self.stored_position = []
                                if (dir==1): # move along x_pos
                                    pc.go_to(edge_x + 0.1, edge_y) # go to y_center
                                    time.sleep(2)
                                    pc.go_to(edge_x+0.1, edge_y + 0.2) # move orthogonally
                                    self.edge_detection()
                                    self.stored_position = []
                                    time.sleep(2)


                                    if (len(self.edges)==2):
                                        edge2_x = self.edges[1][0]
                                        edge2_y = self.edges[1][1]
                                        pc.go_to(edge_x+0.1,edge2_y-0.1)

                                    else:
                                        pc.go_to(edge_x+0.1,edge_y+0.1)
                            break
                        self.stored_position = []

                    print('position:', self.position)
                    time.sleep(1)
                    pc.go_to(self.x, self.y, 0.3)
                    time.sleep(1)

            elif(mode==2):
                print("MOTION COMMANDER")
                #rel_waypoint = np.array([waypoint + self.position for waypoint in waypoints])
                # here position is at [0,0,0]

                with MotionCommander(scf, default_height = 0.4) as mc:
                    # here position is at some weird offset
                    # however the drone is already flying and z is bigger than 0!!
                    #time.sleep(2)
                    rel_waypoint = np.array([waypoint + self.position for waypoint in waypoints])
                    print('Crazyflie initial location:')
                    print(self.position)
                    #print('First waypoint')
                    #print(rel_waypoint)
                    for wp in rel_waypoint:
                        distance = self.get_distance_to_waypoint(wp)
                        print('Distance to waypoint:')
                        print(distance)

                        while (distance>distance_threshold):
                            distance = self.get_distance_to_waypoint(wp)
                            dx,dy,dz = self.get_direction_to_waypoint(wp)
                            #print('Distance to waypoint:')
                            #print(distance)
                            #print('Crazyflie position')
                            #print(self.position)
                            #print('dx: '+str(dx)+'--dy: '+str(dy)+'--dz:'+str(dz))

                            mc.start_linear_motion(k_p*dx,k_p*dy,k_p*dz)
                            time.sleep(0.1)
                        print('WAYPOINT REACHED')
                    mc.stop()
        self._disconnected


    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        #print('[%d][%s]: %s' % (timestamp, logconf.name, data))

        self.x = data['stateEstimate.x']
        self.y = data['stateEstimate.y']
        self.z = data['stateEstimate.z']
        self.position = np.array([self.x, self.y, self.z])
        self.stored_position.append(self.position)

        # Save info into log variable
        for idx,i in enumerate(list(data)):

            self.logs[self.count][idx] = data[i]

        self.count +=1

    def edge_detection(self):
        edge_detected = False
        if (self.ready):
            z = []
            for i in range(len(self.stored_position)):
                z.append(self.stored_position[i][2])
            z = np.asarray(z)
            conv_prod = [np.sum(abs(z[i:i+self.T-1]-self.default_height)*self.kernel) for i in range(z.shape[0]-self.T)]
            #print(conv_prod)
            threshold = np.hstack([np.asarray([(self.T-2)*[False]]).flatten(),np.asarray(conv_prod)<-self.conv_thresh])
            print(min(conv_prod))
            for idx in range(len(threshold)-1):
                if (not threshold[idx] and threshold[idx+1]):
                    self.edges.append(self.stored_position[idx])
                    edge_detected = True
        return edge_detected


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

    # enter crazyflie ID here
    id = 'radio://0/80/2M/E7E7E7E7E7'

    le = LoggingExample('radio://0/80/2M/E7E7E7E7E7')
    le.fly_square(id)



    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)
