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

        # Initialize log variable
        self.logs = np.zeros([10000,6])

        #print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        #self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True
        self.fly_square(link_uri)

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
        self._lg_stab.add_variable('stabilizer.thrust','float')
        self._lg_stab.add_variable('stateEstimate.x','float')
        self._lg_stab.add_variable('stateEstimate.y','float')
        self._lg_stab.add_variable('stateEstimate.z','float')
        self._lg_stab.add_variable('range.zrange','float')

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

    def fly_square(self, id):
        """ Example of simple logico to make the drone fly in a square
        trajectory at fixed speed"""

        # Sync with drone
        with SyncCrazyflie(id, cf=self._cf) as scf:
            # Send position commands
            with PositionHlCommander(scf) as pc:
                pc.up(0.3)

                time.sleep(2)
                # print('Moving up')
                #pc.forward(0.5)
                # print('Moving forward')
                #pc.left(0.5)
                # print('Moving left')
                #pc.back(0.5)
                # print('Moving back')
                #pc.right(0.5)
                # print('Moving right')

        self._disconnected

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        print('[%d][%s]: %s' % (timestamp, logconf.name, data))

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

    # enter crazyflie ID here
    id = 'radio://0/80/2M/E7E7E7E7E7'

    le = LoggingExample('radio://0/80/2M/E7E7E7E7E7')



    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)
