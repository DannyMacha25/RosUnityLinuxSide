#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import TransformStamped 
from tf.msg import tfMessage
from tf import TransformBroadcaster
topic = "/tf"
topic_static = "/tf_static"


def main():
    rospy.init_node("unity_transform_reciever")

    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        #pubTransform('/map','/base_link',0,0,0,topic)
        pubTransform('/base_link','/camera',5,1,0,topic_static)
        pubTransform('/base_link','/hand',3,2,0,topic)
        pubTransform('/hand','/finger',1,1,2,topic)
        loop.sleep()
        #rospy.spin()
        #loop.sleep()
        
    

def pubTransform(frame_id,child_id,x,y,z,top):
    t = tfMessage()
    trans = TransformStamped()
    trans.child_frame_id = child_id
    trans.header.frame_id = frame_id

    trans.header.stamp = rospy.get_rostime()

    trans.transform.translation.x = x
    trans.transform.translation.y = y
    trans.transform.translation.z = z

    trans.transform.rotation.x = 0
    trans.transform.rotation.y = 0
    trans.transform.rotation.z = 0
    trans.transform.rotation.w = 1
    br = TransformBroadcaster()
    br.sendTransform((x,y,z),(0,0,0,1),rospy.Time(rospy.get_rostime().secs,rospy.get_rostime().nsecs),child_id,frame_id)

    t.transforms.append(trans)
    pub = rospy.Publisher("/tf",tfMessage,queue_size=10)
    #pub.publish(t)


if __name__ == "__main__":
    main()