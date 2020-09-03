#!/usr/bin/env python

import rospy
from assignment0.msg import TwoInt
import random

def generator():
    pub = rospy.Publisher('/numbers', TwoInt, queue_size=10)
    rospy.init_node('generator', anonymous=True)
    rate = rospy.Rate(10)
    msg = TwoInt()
    
 
    while not rospy.is_shutdown():
        msg.num1 = random.randint(1,100)
        msg.num2 = random.randint(1,100) 
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        generator()
    except rospy.ROSInterruptException:
        pass
