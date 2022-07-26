#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

topic = "default_topic"

def cb(msg):
    print("Msg recieved!")
    #print("Position of x: {} y: {} z: {} recieved".format(msg.transform.translation.x))


def listener():
    topic = rospy.get_param("topic","cool_topic")
    print("Listening on {0} for stamped transforms".format(topic))
    rospy.Subscriber(topic,PoseStamped,cb)

def main():
    rospy.init_node("unity_transform_reciever")
    listener()
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        topic = rospy.get_param("topic","cool_topic")
        loop.sleep()
    

if __name__ == "__main__":
    main()

