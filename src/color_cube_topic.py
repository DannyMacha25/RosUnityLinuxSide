#!/usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA


def main():
    rospy.init_node("color_cube",anonymous=True)
    
    pub = rospy.Publisher("/ColorCube",ColorRGBA,queue_size=10)
    
    r = rospy.get_param("r",1)
    g = rospy.get_param("g",0)
    b = rospy.get_param("b",0)

    col_msg = ColorRGBA()
    col_msg.r = r
    col_msg.b = b
    col_msg.g = g
    col_msg.a = 1.0

    rate = rospy.Rate(5)
    pub.publish(col_msg)
    while not rospy.is_shutdown():
        r = rospy.get_param("r",1)
        g = rospy.get_param("g",0)
        b = rospy.get_param("b",0)
        col_msg.r = r
        col_msg.b = b
        col_msg.g = g
        pub.publish(col_msg)
        print("color published!")
        rate.sleep()
if __name__ == '__main__':
    main()
