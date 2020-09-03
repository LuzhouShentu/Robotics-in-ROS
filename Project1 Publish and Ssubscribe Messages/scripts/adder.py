#!/usr/bin/env python
import rospy
from assignment0.msg import TwoInt
from std_msgs.msg import Int16
    
def callback(data):   
    pub=rospy.Publisher('/sum', Int16, queue_size=10)
    pub.publish(data.num1+data.num2)
    
def adder():
    rospy.init_node('adder', anonymous=True)
    rospy.Subscriber("/numbers", TwoInt, callback)
    rospy.spin()

if __name__=='__main__':
        adder()
