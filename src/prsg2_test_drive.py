#!/usr/bin/env python
# prsg2_test_drive - a node for ROS to drive (Twist) the psrg2 robot and collect 
#              servo and sensor data.
#
# Usage: when a roscore (kinetic) is succesfully running:
# rosrun prsg2_test_drive prsg2_test_drive.py
#
# Jay Salmonson
# 11/15/2016 - quadrille
# 8/6/2017 - prsg2_test_drive
# 9/29/2017 - added ability to use in STDR simulator

import roslib; roslib.load_manifest("prsg2_test_drive")
import rospy

from geometry_msgs.msg import Twist

from timeit import default_timer as timer
from math import sin, cos, pi
import sys

sys.path.append("../ros_arduino_bridge")
from ros_arduino_msgs.srv import *

from servo_srv_sim import Servo_Srv_Sim

class test_drive(object):

    def __init__(self, bpm = 60, meter = 2):
        self.bpm   = bpm
        self.meter = meter  # numerator of key time signature. e.g. 3 in 3/4.
        self.t_start = -1   # time of start of entire dance
        self.t_spb   = 60./self.bpm  # seconds per beat
        self.led_dur = 0.1*0.25*self.t_spb  # time to keep LED on
        self.led1 = 103
        self.led2 = 104
        self.led1_on = False
        self.led2_on = False

        rospy.init_node('test_drive')

        #: For robot:
        self.servo_srv = rospy.ServiceProxy('/arduino/servo_write', ServoWrite)
        cmd_vel = 'cmd_vel'
        #: For simulation:
        #self.servo_srv = Servo_Srv_Sim()
        #cmd_vel = 'robot0/cmd_vel'

        self.pub = rospy.Publisher(cmd_vel, Twist, queue_size = 1)

        #speed = rospy.get_param("~speed", 0.5)
        #turn  = rospy.get_param("~turn",  1.0)

        #self.led_srv = rospy.ServiceProxy('/arduino/digital_write',DigitalWrite)
        
    def servo_time(self, time):
        t_song    = time - self.t_start
        n_beat    = int(t_song/self.t_spb) + 1 #(1st is 1)
        n_measure = int(n_beat/self.meter) + 1  # measure number (1st is 1)
        # count of current measure:
        n_count   = n_beat - self.meter * (n_measure - 1)

        srv_ang_min = 20 #10
        srv_ang_max = 160 #170
        # servo service invoked if time since last beat is < led duration
        # *** need to refine this: should service only be invoked once?
        if (t_song - (n_beat - 1)*self.t_spb) < self.led_dur:
            if n_count == 1:
                self.servo_srv(id = 0, value = srv_ang_min)
                self.servo_srv(id = 1, value = srv_ang_max)
                '''
                if self.led1_on is False:
                    self.led1_on = True
                    self.led_srv(pin=self.led1,value=1)
                if self.led2_on is True:
                    self.led2_on = False
                    self.led_srv(pin=self.led2,value=0)
                '''
            else:
                self.servo_srv(id = 0, value = srv_ang_max)
                self.servo_srv(id = 1, value = srv_ang_min)
                '''
                if self.led1_on is True:
                    self.led1_on = False
                    self.led_srv(pin=self.led1,value=0)
                if self.led2_on is False:
                    self.led2_on = True
                    self.led_srv(pin=self.led2,value=1)
                '''
        else: # no service request in between time:
            #self.servo_srv(id = 0, value = 90)
            #self.servo_srv(id = 1, value = 90)
            '''
            if self.led1_on is True:
                self.led1_on = False
                self.led_srv(pin=self.led1,value=0)
            if self.led2_on is True:
                self.led2_on = False
                self.led_srv(pin=self.led2,value=0)
            '''
    '''
    def led_off(self):
        self.led_srv(pin=self.led1,value=0)
        self.led_srv(pin=self.led2,value=0)
    '''

    def takeStep(self, dist, n_beats, direction, theta_max):
        t_dur      = n_beats*self.t_spb
        vel_ave    = dist/t_dur # m/s
        shimmy_max = theta_max * pi / t_dur # max angular twist speed
        rate = rospy.Rate(20)  # Hz
        t = t0 = timer()
        # start master timer:
        if self.t_start == -1: self.t_start = t
        # publish twist 0.0 here
        while (t < t0 + t_dur):
            rate.sleep()
            t = timer()
            self.servo_time(t) # blink meter LEDs
            vel = direction * vel_ave * (1. - cos(2.*pi*(t-t0)/t_dur))
            shimmy = shimmy_max * sin(pi*(t-t0)/t_dur)
            twist = Twist()
            twist.linear.x = vel
            twist.angular.z = shimmy
            self.pub.publish(twist)
            print (t-t0), vel, t_dur
        twist = Twist()
        twist.linear.x = 0.
        twist.angular.z = 0.
        self.pub.publish(twist)

    def stepForward(self, dist = 0.5, n_beats = 2):
        self.takeStep(dist, n_beats, 1, 0)
    def stepBackward(self, dist = 0.5, n_beats = 2):
        self.takeStep(dist, n_beats, -1, 0)

if __name__=="__main__":

    drive = test_drive(bpm = 25)
    try:
        #drive.takeStep(0.,2, 1, 0.)
       
        for i in range(2):
            print "i = ",i
            drive.takeStep(2.*0.2, 2, 1, 0.)
            drive.takeStep(2.*0.2, 2, -1, 0.)    # drive
            
            #drive.takeStep(0., 2, 1, 0.01*pi/2.) # turn
            
        
        for i in range(2):
            print "2. i = ",i
            drive.takeStep(0.2, 2,  1, 10.*0.01*pi/2.)
            drive.takeStep(0.2, 2, -1, 10.*0.01*pi/2.)
        
        #drive.led_off()
    except rospy.ROSInterruptException:
        print "Exception"
