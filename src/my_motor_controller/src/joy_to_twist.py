#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO
import subprocess
import shlex

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(13,GPIO.OUT,initial=GPIO.HIGH)
GPIO.setup(33,GPIO.OUT)
shoot = GPIO.PWM(33,100)
shoot.start(0)

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def shooter(check):
    if(check):
        shoot.ChangeDutyCycle(99)
    else:
        shoot.ChangeDutyCycle(0)



class JoyClass:
    def __init__(self, scale=1.0, offset=0.0, deadband=0.1):
        rospy.init_node("joy5_node")
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.wheel1_pub = rospy.Publisher("/wheel1_vel", Float32, queue_size=10)
        self.wheel2_pub = rospy.Publisher("/wheel2_vel", Float32, queue_size=10)
        self.wheel3_pub = rospy.Publisher("/wheel3_vel", Float32, queue_size=10)
        self.wheel4_pub = rospy.Publisher("/wheel4_vel", Float32, queue_size=10)
        self.zaxis_pub = rospy.Publisher("/zaxis_vel", Float32, queue_size=10)
        #self.test = rospy.Publisher("/test", Float32, queue_size=10)
        self.rate = rospy.Rate(10)
        self.shutd = 0

    def joy_callback(self, msg):
        wheel1, wheel2, wheel3, wheel4, joyZ, angular = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        rightTrig = (abs(msg.axes[3]-1.0)/2) 
        leftTrig = (-abs(msg.axes[4]-1.0)/2)
        joyZ = round(msg.axes[4],1)
        if(rightTrig!=0 and leftTrig!=0):
            angular = 0.0
        elif(rightTrig>0.0 and leftTrig == 0.0):
            angular = rightTrig
        elif(leftTrig<0.0 and rightTrig == 0.0):
            angular = leftTrig
        #wheel1 = (msg.axes[0]+msg.axes[6]+angular)*1.5
        #wheel3 = (-msg.axes[0]-msg.axes[6]+angular)*1.5
        #wheel2 = (-msg.axes[1]-msg.axes[7]-angular)*1.5
        #wheel4 = (msg.axes[1]+msg.axes[7]-angular)*3.5      
        wheel1 = (msg.axes[0]*1.5) + (msg.axes[6]*1.5) + (angular * 0.6)
        wheel2 = (msg.axes[1]*1.5) + (msg.axes[7]*1.5) + (angular * 0.6)
        wheel3 = (-msg.axes[0]*1.5) + (-msg.axes[6]*1.5) + (angular * 0.6)
        wheel4 = (-msg.axes[1]*1.5) + (-msg.axes[7]*1.5) + (angular * 0.6)  #angular negative due to inward placement of wheels
        #wheel1 = (msg.axes[6])
        #wheel2 = (msg.axes[7])
        #wheel3 = (-msg.axes[6])
        #wheel4 = (-msg.axes[7])  
        self.wheel1_pub.publish(constrain(wheel1,-1.5,1.5))
        self.wheel2_pub.publish(constrain(wheel2,-1.5,1.5))
        self.wheel3_pub.publish(constrain(wheel3,-1.5,1.5))
        self.wheel4_pub.publish(constrain(wheel4,-1.5,1.5))
        #self.test.publish(angular)
        if(joyZ > 0.3 or joyZ < -0.3 or joyZ==0.0):
            self.zaxis_pub.publish(-(msg.axes[5]*1.7))
        if(msg.buttons[0]):
            shooter(1)
        if(msg.buttons[2]):
            shooter(0)
        if(msg.buttons[1]):
            self.shutd += 1
            if(self.shutd == 3):
                cmd = shlex.split("sudo shutdown -h now")
                subprocess.call(cmd)
if __name__=="__main__":
    try:
        JoyClass()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
