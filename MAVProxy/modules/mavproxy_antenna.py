#!/usr/bin/env python
'''
antenna pointing module
Andrew Tridgell
June 2012
'''

import sys, os, time, math
from cuav.lib import cuav_util
from MAVProxy.modules.lib import mp_module

class AntennaModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(AntennaModule, self).__init__(mpstate, "antenna", "antenna pointing module")
        self.gcs_location = None
        self.last_bearing = 0
        self.last_announce = 0
        self.add_command('antenna', self.cmd_antenna, "antenna link control")
        self.vehicle_position = None
        self.vehicle_alt = None
        self.home_alt = None

    def cmd_antenna(self, args):
        '''set gcs location'''
        if len(args) != 2:
            if self.gcs_location is None:
                print("GCS location not set yet")
            else:
                print("GCS location %s" % str(self.gcs_location))
            return
        self.gcs_location = (float(args[0]), float(args[1]))

    def try_set_home(self):
        if self.gcs_location is not None:
            return

        if self.vehicle_position is not None:
            self.gcs_location = self.vehicle_position
            print('GCS location auto-set to vehicle position')
        elif self.module('wp').wploader.count() > 0:
            home = self.module('wp').get_home()
            self.gcs_location = (home.x, home.y)
            print('GCS location auto-set to waypoint home')

        print(f'GCS location auto-set to {self.gcs_location}')

    def process_vehicle_pos(self, lat, lon):
        self.vehicle_position = (lat, lon)
        self.try_set_home()
        self.point_antenna()

    def calc_elevation_degrees(self, distance):
        if self.home_alt is None or self.vehicle_alt is None or distance <= 0:
            return 0

        self.vehicle_distance = distance
            
        c2 = math.sqrt(distance ** 2 + self.vehicle_alt ** 2)
        tmp = self.vehicle_alt / c2
        return math.degrees(math.asin(tmp))

    def point_antenna(self):
        if self.vehicle_position is None or self.gcs_location is None:
            return

        (lat, lon) = self.vehicle_position
        (gcs_lat, gcs_lon) = self.gcs_location
        bearing = cuav_util.gps_bearing(gcs_lat, gcs_lon, lat, lon)
        elevation = self.calc_elevation_degrees(cuav_util.gps_distance(gcs_lat, gcs_lon, lat, lon))
                
        self.console.set_status('Antenna', 'Antenna %.0f' % bearing, row=0)
        if (time.time() - self.last_announce) > 5:
            self.last_bearing = bearing
            self.last_announce = time.time()
            self.say(f'Antenna bearing {int(bearing + 0.5)} elevation {int(elevation + 0.5)}')
            self.say(f'Vehicle distance {self.vehicle_distance} altitude {self.vehicle_alt}')

    def try_set_home_alt(self, alt, relative_alt):
        if self.home_alt is not None:
            return
        if alt == relative_alt:
            print('GCS auto-set home altitude to {alt}')
            self.home_alt = alt
        else:
            print('GCS auto-set home altitude to zero, because relative altitude is provided by vehicle')
            self.home_alt = 0

    def process_vehicle_alt(self, alt, relative_alt):
        self.try_set_home_alt(alt, relative_alt)

        self.vehicle_alt = relative_alt - self.home_alt
        self.point_antenna()

    def mavlink_packet(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'GPS_RAW':
            self.process_vehicle_pos(m.lat, m.lon)
        elif m.get_type() == 'GPS_RAW_INT':
            self.process_vehicle_pos(m.lat / 1.0e7, m.lon / 1.0e7)
        elif m.get_type() == 'GLOBAL_POSITION_INT':
            self.process_vehicle_alt(float(m.alt) / 1000.0, float(m.relative_alt) / 1000.0)


def init(mpstate):
    '''initialise module'''
    return AntennaModule(mpstate)
