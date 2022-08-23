#!/usr/bin/python3

import rospy
import roslib
#roslib.load_manifest('differential_drive')
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import UInt32

#############################################################################
class OmniWheelTF:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("omniwheel_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('ticks_meter', 1163600))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.72)) # The wheel base width in meters
        
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        
        self.encoder_min = rospy.get_param('encoder_min', 0)
        self.encoder_max = rospy.get_param('encoder_max', 4294967295)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.enc_front = None
        self.enc_back = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.front = 0
        self.back = 0
        self.lmult = 0
        self.rmult = 0
        self.fmult = 0
        self.bmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.prev_fencoder = 0
        self.prev_bencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0
        self.dy = 0                 # speeds in y/rotation
        self.dr = 0
        self.then = rospy.Time.now()
        
        # subscriptions
        rospy.Subscriber("wheel4_enc_32bit", UInt32, self.rwheelCallback)
        rospy.Subscriber("wheel2_enc_32bit", UInt32, self.lwheelCallback)
        rospy.Subscriber("wheel1_enc_32bit", UInt32, self.fwheelCallback)
        rospy.Subscriber("wheel3_enc_32bit", UInt32, self.bwheelCallback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
     
    #############################################################################
    def update(self):
    #############################################################################
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            
            # calculate odometry
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.enc_right - self.right) / self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right

            if self.enc_front == None:
                d_front = 0
                d_back = 0
            else:
                d_front = (self.front - self.enc_front) / self.ticks_meter
                d_back = (self.enc_back - self.back) / self.ticks_meter
            self.enc_front = self.front
            self.enc_back = self.back

            # distance traveled is the average of the two wheels 
            dx = ( d_left + d_right ) / 2
            dy = ( d_front + d_back ) / 2 
            # this approximation works (in radians) for small angles
            th = ( (( d_right - d_left ) + ( d_back - d_front )) /2 ) / self.base_width
            # calculate velocities
            self.dx = dx / elapsed
            self.dy = dy / elapsed
            self.dr = th / elapsed
             
            if (dx != 0 or dy != 0 ):
                # calculate distance traveled in x and y
                x = cos( th ) * dy
                y = cos( th ) * dx
                # calculate the final position of the robot
                self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                self.y = self.y + ( cos( self.th ) * y - sin( self.th ) * x )
            if( th != 0):
                self.th = self.th + th
                
            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( self.th / 2 )
            quaternion.w = cos( self.th / 2 )
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = self.dy
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)
            
            


    #############################################################################
    def lwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
            
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lencoder = enc
        
    #############################################################################
    def rwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

    #############################################################################
    def fwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_fencoder > self.encoder_high_wrap):
            self.fmult = self.fmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_fencoder < self.encoder_low_wrap):
            self.fmult = self.fmult - 1
            
        self.front = 1.0 * (enc + self.fmult * (self.encoder_max - self.encoder_min))
        self.prev_fencoder = enc

    #############################################################################
    def bwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_bencoder > self.encoder_high_wrap):
            self.bmult = self.bmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_bencoder < self.encoder_low_wrap):
            self.bmult = self.bmult - 1
            
        self.back = 1.0 * (enc + self.bmult * (self.encoder_max - self.encoder_min))
        self.prev_bencoder = enc

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        omniTf = OmniWheelTF()
        omniTf.spin()
    except rospy.ROSInterruptException:
        pass
