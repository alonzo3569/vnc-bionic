#!/usr/bin/env python
'''
Author: Logan Zhang
Date: 2021/03/19
Using logitech controller to control wamv motor
'''

import rospy
from sensor_msgs.msg import Joy
from motor.msg import Motor

class Joystick():

    def __init__(self):

        # Params
        self.forward_value = 0.5
        self.backward_value = 0.5
        self.motor_msg = Motor()

        # Publisher
        self.motor_pub = rospy.Publisher("motor", Motor, queue_size=10)

        # Subscriber
        self.joy_sub = rospy.Subscriber("joy", Joy, self.cb_joy)

    def cb_joy(self, msg):
        # axes   : type float, left/up : +1 ;  right/down : -1 
        # button : type int, press : 1, release : 0
        print msg.axes
        print msg.buttons
        [leftStickLR, lefttStickUpDown, rightStickLR, rightStickUpDown, crossLR, crossUpDown] = msg.axes
        button_state = msg.buttons
        [X, A, B, Y, LB, RB, LT, RT, back, start, button_stick_left, button_stick_right] = button_state
        #rospy.loginfo("X : [%i]" %(X))
        #rospy.loginfo("Type of X is [%s]" %(type(X)))
        #rospy.loginfo("Left stick left/Right : [%f]" %(leftStickLR))
        #rospy.loginfo("Type of axis is [%s]" %(type(leftStickLR)))

        # Button mode
        if button_state.count(1) > 1:
            rospy.loginfo("Please press one button at a time.")
        else:
            try:
                if button_state.index(1) == 0:
                    self.left()
                elif button_state.index(1) == 1:
                    self.backward()
                elif button_state.index(1) == 2:
                    self.right()
                elif button_state.index(1) == 3:
                    self.forward()
            except ValueError,err :
                rospy.loginfo("idle")
                self.stop()
                
    def forward(self):
        rospy.loginfo("Forward")
        self.motor_msg.left  = self.forward_value 
        self.motor_msg.right = self.forward_value
        self.motor_pub.publish(self.motor_msg)

    def backward(self):
        rospy.loginfo("Backward")
        self.motor_msg.left  = -self.backward_value
        self.motor_msg.right = -self.backward_value
        self.motor_pub.publish(self.motor_msg)


    def right(self):
        rospy.loginfo("Right")
        self.motor_msg.left  = self.forward_value
        self.motor_msg.right = -self.backward_value 
        self.motor_pub.publish(self.motor_msg)


    def left(self):
        rospy.loginfo("Left")
        self.motor_msg.left  = -self.backward_value 
        self.motor_msg.right = self.forward_value
        self.motor_pub.publish(self.motor_msg)

    def stop(self):
        rospy.loginfo("Stop")
        self.motor_msg.left  = 0.0
        self.motor_msg.right = 0.0
        self.motor_pub.publish(self.motor_msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." %(rospy.get_name()))




if __name__ == '__main__':
    rospy.init_node('joystick_control',anonymous=False)
    joystick = Joystick()
    rospy.on_shutdown(joystick.on_shutdown)
    rospy.spin()
