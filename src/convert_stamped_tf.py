#!/usr/bin/env python2.7

from tf.msg import tfMessage
import rospy
from geometry_msgs.msg import TransformStamped

topic = '/dumb'



def cb(msg):
    tfmsg = tfMessage()
    tfmsg.transforms.append(msg)
    pub = rospy.Publisher("/tf",tfMessage,queue_size=10)
    pub.publish(tfmsg)
    #print("published tf")


def main():
    rospy.init_node("converter")

    rospy.Subscriber(topic,TransformStamped,cb,queue_size=10)

    loop = rospy.Rate(10)

    while not rospy.is_shutdown():
        #loop.sleep()
        rospy.spin()

if __name__ == "__main__":
    main()
