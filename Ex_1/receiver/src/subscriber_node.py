#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import matplotlib.pyplot as plt

data_list=[]
data2_list=[]
q=0.15

def callback(data):
    rospy.loginfo("Received message: %d",data.data)
    data_list.append(data.data)

    data2_list.append(data.data/q)
    pub.publish(data.data/q)
    plot_data()

def subscriber_node():
    rospy.init_node('Subscriber')
    #Subscriber
    rospy.Subscriber('lodetti', Int32, callback)
    #Publisher
    global pub
    pub = rospy.Publisher('/kthfs/result', Int32, queue_size=2)
    
    rospy.spin()

def plot_data():
    plt.clf()   # Clear the figure
    plt.plot(data_list,label='K')
    plt.plot(data2_list,label='K/q')
    plt.xlabel('iteration')
    plt.ylabel('value')
    plt.title('K and K/q plot')
    plt.legend() # display data labels
    plt.pause(0.1)

if __name__ == '__main__':
    try:
        plt.ion()  # interactive mode
        plt.show() # show the plot (initially empty)
        subscriber_node()
    except rospy.ROSInterruptException:
        pass 