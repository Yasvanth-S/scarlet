#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO
import subprocess
import shlex
import time
from geometry_msgs.msg import Twist

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.OUT,initial=GPIO.HIGH)
GPIO.setup(13,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(29,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(33,GPIO.OUT)
shoot = GPIO.PWM(33,100)
shoot.start(0)

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def shooter(check):
    if(check == 50):
        shoot.ChangeDutyCycle(38)
    elif(check == 75):
        shoot.ChangeDutyCycle(40)
    elif(check == 99):
        shoot.ChangeDutyCycle(42)  
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
        self.velocity_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)
        self.x = 0
        self.y = 0
        self.z = 0
    
    def velocity_callback(self,msg):
        self.x = msg.linear.x
        self.y = msg.linear.y
        self.z = msg.angular.z
        wheel1 = (-self.x/0.05) + (self.z /0.05)
        wheel2 = (-self.y/0.05) + (self.z / 0.05)
        wheel3 = (self.x/0.05) + (self.z / 0.05) # 0.038 is wheel radius
        wheel4 = (self.y/0.05) + (self.z / 0.05)#angular negative due to inward placement of wheels
        self.wheel1_pub.publish(wheel1)
        self.wheel2_pub.publish(wheel2)
        self.wheel3_pub.publish(wheel3)
        self.wheel4_pub.publish(wheel4)

    def joy_callback(self, msg):
        wheel1, wheel2, wheel3, wheel4, joyZ, angular = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        rightTrig = (abs(msg.axes[3]-1.0)/2) 
        leftTrig = (msg.axes[4]-1.0)/2
        joyZ = round(msg.axes[4],1)
        if(rightTrig!=0 and leftTrig!=0):
            angular = 0.0
        elif(rightTrig>0):
            angular = rightTrig
        elif(leftTrig<0):
            angular = leftTrig
        #wheel1 = (msg.axes[0]+msg.axes[6]+angular)*1.5
        #wheel3 = (-msg.axes[0]-msg.axes[6]+angular)*1.5
        #wheel2 = (-msg.axes[1]-msg.axes[7]-angular)*1.5
        #wheel4 = (msg.axes[1]+msg.axes[7]-angular)*3.5      
        wheel1 = (-msg.axes[0]*0.8) + (-msg.axes[6]*0.8) + (angular * 0.3)
        wheel2 = (-msg.axes[1]*0.8) + (-msg.axes[7]*0.8) + (angular * 0.3)
        wheel3 = (msg.axes[0]*0.8) + (msg.axes[6]*0.8) + (angular * 0.3)
        wheel4 = (msg.axes[1]*0.8) + (msg.axes[7]*0.8) + (angular * 0.3)  #angular negative due to inward placement of wheels
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
            shooter(50)
        if(msg.buttons[4]):
            shooter(75)
        if(msg.buttons[5]):
            shooter(99)
        if(msg.buttons[2]):
            shooter(0)
        if(msg.buttons[1]):
            self.shutd += 1
            if(self.shutd == 3):
                cmd = shlex.split("sudo shutdown -h now")
                subprocess.call(cmd)
        if(msg.buttons[3]):
            GPIO.output(29,GPIO.HIGH)
            GPIO.output(29,GPIO.LOW)

if __name__=="__main__":
    try:
        JoyClass()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
