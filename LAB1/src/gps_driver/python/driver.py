#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Char
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from std_msgs.msg import Header
from gps_driver.msg import gps_msg
import serial
import math as mathlib
from optparse import OptionParser
import sys

DEFAULT_PORT = '/dev/pts/1'

class gps_parser():
   
    K0 = 0.9996

    E = 0.00669438
    E2 = E * E
    E3 = E2 * E
    E_P2 = E / (1 - E)

    SQRT_E = mathlib.sqrt(1 - E)
    _E = (1 - SQRT_E) / (1 + SQRT_E)
    _E2 = _E * _E
    _E3 = _E2 * _E
    _E4 = _E3 * _E
    _E5 = _E4 * _E

    M1 = (1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256)
    M2 = (3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024)
    M3 = (15 * E2 / 256 + 45 * E3 / 1024)
    M4 = (35 * E3 / 3072)

    P2 = (3 / 2 * _E - 27 / 32 * _E3 + 269 / 512 * _E5)
    P3 = (21 / 16 * _E2 - 55 / 32 * _E4)
    P4 = (151 / 96 * _E3 - 417 / 128 * _E5)
    P5 = (1097 / 512 * _E4)

    R = 6378137

    ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX"
    GPGGA_DATA_POSITIONS = [1,2,3,4,5,7,9]
    
    
    def __init__(self,
                 serial_instance,
                 publisher_instance):
        self.serial_instance = serial_instance
        self.publisher_instance = publisher_instance
        
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.UTMEasting = 0
        self.UTMNorthing = 0
        self.number_of_satellites = 0
        self.gps_timestamp_secs = 0
        self.gps_timestamp_nsecs = 0
        self.zone = None
        self.grid = None
        self.central_longitude = None
        self.new_data = False
        self.HDOP = None
        
    def parse_new_gps_data(self):
        
        try:
            self.new_gps_data = (self.serial_instance.readline()).decode('utf-8')
            self.new_gps_data = self.new_gps_data.strip('\n')
            self.new_gps_data = self.new_gps_data.strip('\r')        
            self.new_gps_data = self.new_gps_data.split(',')
        except:
            self.new_gps_data = None
        if(self.new_gps_data is not None):           #Check for GPGGA Data
            if(self.new_gps_data[0]=='$GPGGA'):
                for i in range (0,len(gps_parser.GPGGA_DATA_POSITIONS)):
                    if(self.new_gps_data[gps_parser.GPGGA_DATA_POSITIONS[i]])=='':
                        self.new_data=False
                        print('Data Frame with Null values... dropping frame')
                        print(self.new_gps_data)
                        return
                self.gps_timestamp_secs = int(self.new_gps_data[1][:2])*3600 + int(self.new_gps_data[1][2:4])*60 + int(self.new_gps_data[1][4:6])
                self.gps_timestamp_nsecs = float(self.new_gps_data[1][4:])%1
                self.gps_timestamp_nsecs = int(self.gps_timestamp_nsecs*10**9)
                self.latitude = float(self.new_gps_data[2])//100 + (float(self.new_gps_data[2])%100)/60  #deg + min->deg
                if(self.new_gps_data[3]=='S'):
                    self.latitude = self.latitude*(-1)
                self.longitude = float(self.new_gps_data[4])//100 + (float(self.new_gps_data[4])%100)/60  #deg + min->deg
                if(self.new_gps_data[5]=='W'):
                    self.longitude = self.longitude*(-1)

                self.altitude = float(self.new_gps_data[9])
                self.number_of_satellites = int(self.new_gps_data[7])
                self.HDOP = float(self.new_gps_data[8])
                
                self.__latlongToUTM()
                
            
            
    
    def pubish_gps_data(self):
        
        if (self.new_data):
            gps_data = gps_msg()
            
            gps_data.Header.frame_id = 'GPS1_Frame'
            gps_data.Header.stamp.secs = self.gps_timestamp_secs
            gps_data.Header.stamp.nsecs = self.gps_timestamp_nsecs
            
            gps_data.Latitude = self.latitude
            gps_data.Longitude = self.longitude
            gps_data.Altitude = self.altitude

            gps_data.UTM_easting = self.UTMEasting
            gps_data.UTM_northing = self.UTMNorthing

            gps_data.Letter = self.grid
            gps_data.Zone = self.zone
            
            print('No. Of Satellites : {}   HDOP: {}'.format(self.number_of_satellites,self.HDOP))
            self.publisher_instance.publish(gps_data)
            self.__clear_gps_data()
        
    def __latlongToUTM(self):
        """This function converts Latitude and Longitude to UTM coordinate

        Parameters
        ----------
        latitude: float 
        Latitude between 80 deg S and 84 deg N, e.g. (-80.0 to 84.0)

        longitude: float 
        Longitude between 180 deg W and 180 deg E, e.g. (-180.0 to 180.0).

        Zone number is represented by global map numbers of an UTM zone
        numbers map. You may force conversion to be included within one
        UTM zone number.  For more information see utmzones [1]_

        You may force conversion to be included within one UTM zone
        letter.  For more information see utmzones [1]_

        Returns
        -------
        easting: float 
        Easting value of UTM coordinates

        northing: float 
        Northing value of UTM coordinates

        zone_number: int
        Zone number is represented by global map numbers of a UTM zone
        numbers map. More information see utmzones [1]_

        zone_letter: char
        Zone letter is represented by a string value. UTM zone designators
        can be accessed in [1]_


        .. _[1]: http://www.jaworski.ca/utmzones.htm
        """
        if not self.__in_bounds(self.latitude, -80.0, 84.0):
            print('latitude out of range (must be between 80 deg S and 84 deg N)')
            self.new_data=False
            return
        
        if not self.__in_bounds(self.longitude, -180.0, 180.0):
            print('longitude out of range (must be between 180 deg W and 180 deg E)')
            self.new_data=False
            return 

        self.__latitude_to_zone_letter()
        self.__latlon_to_zone_number()
            
        lat_rad = mathlib.radians(self.latitude)
        lat_sin = mathlib.sin(lat_rad)
        lat_cos = mathlib.cos(lat_rad)

        lat_tan = lat_sin / lat_cos
        lat_tan2 = lat_tan * lat_tan
        lat_tan4 = lat_tan2 * lat_tan2

        
        lon_rad = mathlib.radians(self.longitude)
        central_lon = self.__zone_number_to_central_longitude()
        central_lon_rad = mathlib.radians(central_lon)

        n = gps_parser.R / mathlib.sqrt(1 - gps_parser.E * lat_sin**2)
        c = gps_parser.E_P2 * lat_cos**2

        a = lat_cos * self.__mod_angle(lon_rad - central_lon_rad)
        a2 = a * a
        a3 = a2 * a
        a4 = a3 * a
        a5 = a4 * a
        a6 = a5 * a

        m = gps_parser.R * (gps_parser.M1 * lat_rad -
                gps_parser.M2 * mathlib.sin(2 * lat_rad) +
                gps_parser.M3 * mathlib.sin(4 * lat_rad) -
                gps_parser.M4 * mathlib.sin(6 * lat_rad))

        self.UTMEasting = gps_parser.K0 * n * (a +
                            a3 / 6 * (1 - lat_tan2 + c) +
                            a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * gps_parser.E_P2)) + 500000

        self.UTMNorthing = gps_parser.K0 * (m + n * lat_tan * (a2 / 2 +
                                            a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c**2) +
                                            a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * gps_parser.E_P2)))
        
        if self.__negative(self.latitude):
            self.UTMNorthing += 10000000

        self.new_data = True

        
    def __latitude_to_zone_letter(self):
        if -80 <= self.latitude <= 84:
            self.grid = gps_parser.ZONE_LETTERS[int(self.latitude + 80) >> 3]
        else:
            self.grid = None


    def __latlon_to_zone_number(self):

        if 56 <= self.latitude < 64 and 3 <= self.longitude < 12:
            self.zone = 32

        elif 72 <= self.latitude <= 84 and self.longitude >= 0:
            if self.longitude < 9:
                self.zone = 31
            elif self.longitude < 21:
                self.zone = 33
            elif self.longitude < 33:
                self.zone = 35
            elif self.longitude < 42:
                self.zone = 37
        else:
            self.zone = int((self.longitude + 180) / 6) + 1


    def __zone_number_to_central_longitude(self):
        if self.zone is not None:
            return((self.zone - 1) * 6 - 180 + 3)
        else:
            return None
    
    def __mod_angle(self,value):
        """Returns angle in radians to be between -pi and pi"""
        return (value + mathlib.pi) % (2 * mathlib.pi) - mathlib.pi
    
    def __in_bounds(self,x, lower, upper, upper_strict=False):      
        if upper_strict:
            return lower <= x < upper
        else:
            return lower <= x <= upper
        
    def __negative(self,x):
        return x<0
    
    def __clear_gps_data(self):
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0
        self.UTMEasting = 0
        self.UTMNorthing = 0
        self.number_of_satellites = 0
        self.gps_timestamp_secs = 0
        self.gps_timestamp_nsecs = 0
        self.zone = None
        self.grid = None
        self.central_longitude = None
        self.new_data = False
        
    def __del__(self): 
        print("Destructor called, Object deleted.") 
    

def main():
    rospy.init_node('gps_data_parser_node',anonymous=True)
    
    serial_port = None
    serial_baud = None
    
    # parser = OptionParser()
    # parser.add_option("-p","--port",
    #     help="Pass USB Port of Operation",
    #     dest="port")
    
    # parser.add_option("-b","--baud",
    #     help="Pass baud rate of Operation",
    #     dest="baud")
    
    # (opts, args_) = parser.parse_args()
    # serial_port = opts.port
    # serial_baud = opts.baud
    
    if len(sys.argv)<2:
        print('No usb port has been passed... using default port %s',DEFAULT_PORT)
    
    serial_port = rospy.get_param('~port')
    serial_baud = int(rospy.get_param('~baud'))
    print(sys.argv)
    print(serial_port)
    print(serial_baud)
    
        
    serial_data_stream = None   
    try:
        serial_data_stream = serial.Serial(serial_port, serial_baud, timeout=3)
    except serial.SerialException as e:
        raise e
    
    gps_data_publisher = rospy.Publisher('/gps', gps_msg, queue_size=10)
    rate = rospy.Rate(20) # Rate in Hz i.e loop per second
    
    myParser = gps_parser(serial_data_stream,gps_data_publisher)
    
    while not rospy.is_shutdown():
        
        myParser.parse_new_gps_data()
        myParser.pubish_gps_data()
        rate.sleep()
    
     
if __name__ == "__main__":
    
    try:
        main()
    except rospy.ROSInterruptException:
        print("Something wrong!!")
        


