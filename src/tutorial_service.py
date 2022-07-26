#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger,TriggerResponse

def service(req):
    print("service called :)")
    response = TriggerResponse() #I left off with adding a response to trigger
    response.message = ":3"
    response.success = True
    return response
    

def service_server():
    serv = rospy.Service('testservice',Trigger,service)
    print("Test Service started")

def main():
    rospy.init_node('test_service_server_node')
    service_server()
    rospy.spin()
    
if __name__ == "__main__":
    main()