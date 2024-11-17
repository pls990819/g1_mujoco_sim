#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import signal
import sys

def signal_handler(sig, frame):
    rospy.loginfo("Shutting down keyboard_publisher node...")
    rospy.signal_shutdown("Keyboard Interrupt")

def keyboard_publisher():
    pub = rospy.Publisher('/ctrlMode', String, queue_size=10)
    rospy.init_node('keyboard_interface', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    signal.signal(signal.SIGINT, signal_handler)
    while not rospy.is_shutdown():
        # try:
        user_input = input("Enter a number (0, 1, 2, 3, 4): ")
        if user_input in ['0', '1', '2', '3', '4']:
            rospy.loginfo(user_input)
            pub.publish(user_input)
        else:
            rospy.logwarn("Invalid input! Please enter 0, 1, 2, 3, or 4.")
        # except EOFError:
        #     break
        rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_publisher()
    except rospy.ROSInterruptException:
        pass
