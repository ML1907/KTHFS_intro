#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def publisher_node():
    rospy.init_node('Publisher')
    pub = rospy.Publisher('lodetti', Int32, queue_size=2)
    rate = rospy.Rate(20)  
    k=4

    while not rospy.is_shutdown():   # we don't use rospy.spin to set the sleep rate. For this reason we also need to introduce rospy.is_shutdown() to see when the node is stopped
        pub.publish(k)
        rate.sleep()
        k+=4 

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass 

