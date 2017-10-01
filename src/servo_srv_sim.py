'''
servo_srv_sim.py - simulate the psrg2 servo service with the 
                   stdr_prsg2 fake robot.

Jay Salmonson
9/29/2017
'''
from __future__ import print_function
import sys
from math import pi


import rospy

from std_msgs.msg import Int16

#sys.path.append("../../stdr_simulator")
from stdr_msgs.srv import ReadSensorPose, WriteSensorPose

ms = 1.e-3 # [milliseconds/second]

class Servo_Srv_Sim(object):

    def __init__(self, dt = 10, rate = 0.1, N_servos = 2):
        self.dt = dt  #: [ms] time til next servo move
        self.rate = rate #: [degree/ms] sweep rate of servo
        self.delt = rate*dt #: [degrees/move]
        self.pubrate = 10 #: [Hz] rate of servo position publication
        
        self.N_servos = N_servos #: number of servos

        self.servos = []
        basenm = '/robot0/sonar_'
        for isrv in range(N_servos):
            servo = {}

            servo['index'] = isrv
            
            servo['read_srv'] = \
                rospy.ServiceProxy(basenm+str(isrv)+'/readpose',
                                   ReadSensorPose)
            servo['write_srv'] = \
                rospy.ServiceProxy(basenm+str(isrv)+'/writepose',
                                   WriteSensorPose)
            
            rd_srv = servo['read_srv']()
            servo['pose'] = rd_srv.pose
            servo['th_srv'] = rd_srv.pose.theta # servo->robot angle [rad]

            # The following are in the servo reference frame [deg]:
            servo['theta_ref'] = 90 # center, reference angle of servo
            servo['theta_req'] = servo['theta_ref'] # requested theta

            servo['theta'] = servo['theta_ref'] # actual servo theta

            servo['pub_theta'] = rospy.Publisher(basenm+str(isrv)+'/angle',
                                                 Int16, queue_size = 2)
            
            self.servos.append(servo)
            
        # Init timer for servo movement:
        self.timer = rospy.Timer(rospy.Duration(self.dt*ms), self.onTimer)
        # Init timer for publication of servo position:
        self.timerpub = rospy.Timer(rospy.Duration(1./self.pubrate), self.onTimerpub)
        
    def onTimer(self, event):
        '''
        # could use 
        cr = event.current_real.to_sec()
        try:
            lr = event.last_real.to_sec()
        except:
            lr = cr
        dt = lr - cr
        delt = dt
        '''
        #print("{} {} {} {}".format(event.current_real, event.last_real, event.last_duration, cr - lr))  #, event.current_real - event.last_real))

        # move servo thetas:
        myend = ''
        for srv in self.servos:
            if not srv['theta'] == srv['theta_req']:            
                if srv['theta'] - srv['theta_req'] > 0.9*self.delt:
                    srv['theta'] -= self.delt
                elif srv['theta'] - srv['theta_req'] < -0.9*self.delt:
                    srv['theta'] += self.delt
                else:
                    srv['theta'] = srv['theta_req']

                dth = (srv['theta'] - srv['theta_ref'])*pi/180.
                srv['pose'].theta = srv['th_srv'] + dth
                srv['write_srv'](srv['pose'])
                
            #print(" {} {}  ".format(srv['theta_req'], srv['theta']), end=myend)
            myend = "\n"
            
    def onTimerpub(self, event):
        for srv in self.servos:
            msg = Int16()
            msg.data = srv['theta']
            srv['pub_theta'].publish(msg)
        
    def __call__(self, id, value):
        self.servos[id]['theta_req'] = min(max(int(value), 0), 180)
        

if __name__ == "__main__":

    import time
    
    rospy.init_node("sss_node")
    sss = Servo_Srv_Sim()
    sss(0,2)
    sss(1,178)
    time.sleep(1.5)
    sss(0,43)
    sss(1,87)
    time.sleep(1)
