#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String
from imu_driver.msg import Vectornav
from imu_driver.srv import convert_to_quaternion
import numpy as np

########################################################
## IMU publisher ##
    
def set_config(data):
    binary="$VNWRG,75,2,20,15,0009,000C,0014*XX\r\n"
    data.write(binary.encode())
    print(data.readline().decode())

def handle_convert_quat(req):
    # time.sleep(5)
    yaw=np.deg2rad(req.yaw)
    pitch=np.deg2rad(req.pitch)
    roll=np.deg2rad(req.roll)
    
    
    # print("values handled")
    x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return x,y,z,w

def to_quat_client(yaw,pitch,roll):
    rospy.wait_for_service('convert_to_quaternion')
    quated=rospy.ServiceProxy('convert_to_quaternion',convert_to_quaternion)
    resp1=quated(yaw,pitch,roll)
    return (resp1.x,resp1.y,resp1.z,resp1.w)


def imu_driver(serial_data):
    publish=rospy.Publisher('imu',Vectornav,queue_size=10)
    rospy.init_node('imu_driver',anonymous=True)
    service=rospy.Service('convert_to_quaternion',convert_to_quaternion,handle_convert_quat)
    while not rospy.is_shutdown():
        splitted=serial_data.readline().decode().replace("$","").replace("\r\n","").replace("\x00","").split('*')
        splitted=splitted[0].split(',')
        # decoded=line
        # splitted=decoded.split(',')
        # print(splitted)
        if len(splitted)>=13 and "VNYMR" not in splitted[1:]:
            
            imu_message=Vectornav()
            now=rospy.get_rostime()

            imu_message.Header.frame_id="imu1_frame"
            imu_message.Header.stamp.secs=now.secs
            imu_message.Header.stamp.nsecs=now.nsecs
            
            yaw=float(splitted[1])
            pitch=float(splitted[2])
            roll=float(splitted[3])
            x,y,z,w=to_quat_client(yaw,pitch,roll)
            imu_message.imu.x=x
            imu_message.imu.y=y
            imu_message.imu.z=z
            imu_message.imu.w=w

            magx=float(splitted[4])
            magy=float(splitted[5])
            magz=float(splitted[6])

            imu_message.mag_field.x=magx
            imu_message.mag_field.y=magy
            imu_message.mag_field.z=magz

            accelx=float(splitted[7])
            accely=float(splitted[8])
            accelz=float(splitted[9])

            imu_message.linear_acceleration.x=accelx
            imu_message.linear_acceleration.y=accely
            imu_message.linear_acceleration.z=accelz

            gyrox=float(splitted[10])
            gyroy=float(splitted[11])
            gyroz_unfilter=splitted[12]
            # gyroz_unfilter.replace("\r\n","")
            gyroz=float(gyroz_unfilter)

            imu_message.angular_velocity.x=gyrox
            imu_message.angular_velocity.y=gyroy
            imu_message.angular_velocity.z=gyroz

            publish.publish(imu_message)
            serial_data.flush()
      
        
if __name__== '__main__':
    port=rospy.get_param('/imu/port')
    serial_data=serial.Serial(port,115200)
    set_config(serial_data)
    try:
        imu_driver(serial_data)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass