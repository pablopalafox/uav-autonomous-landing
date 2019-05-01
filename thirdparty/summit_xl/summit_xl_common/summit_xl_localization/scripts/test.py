#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import MagneticField
from math import atan2, sin, cos, sqrt

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "mx: %s", data.magnetic_field.x)
    xmag = data.magnetic_field.x
    ymag = data.magnetic_field.y
    zmag = data.magnetic_field.z
    mag_norm=sqrt((xmag*xmag)+(ymag*ymag)+(zmag*zmag))
    # for normalization
    magx=xmag/mag_norm
    magy=ymag/mag_norm
    magz=zmag/mag_norm
        
    Roll = 0;
    Pitch = 0;
    #yaw =atan2( (-ymag*cos(Roll) + zmag*sin(Roll) ) , (xmag*cos(Pitch) + ymag*sin(Pitch)*sin(Roll)+ zmag*sin(Pitch)*cos(Roll)) ) 
    #yaw =atan2( (-magy*cos(Roll) + magz*sin(Roll) ) , (magx*cos(Pitch) + magy*sin(Pitch)*sin(Roll)+ magz*sin(Pitch)*cos(Roll)) ) 
    
    cos_pitch = cos(Pitch)
    sin_pitch = sin(Pitch)
    cos_roll = cos(Roll)
    sin_roll = sin(Roll)
    t_mag_x = magx * cos_pitch + magz * sin_pitch
    t_mag_y = magx * sin_roll * sin_pitch + magy * cos_roll - magz * sin_roll * cos_pitch
    head_x = t_mag_x
    head_y = t_mag_y  
    yaw = atan2(head_x, head_y)

    
    rospy.loginfo(rospy.get_caller_id() + "yaw: %s", yaw)
        
def listener():

    rospy.init_node('test', anonymous=True)

    rospy.Subscriber("/mavros/imu/mag", MagneticField, callback)
                      
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
