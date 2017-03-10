#!/usr/bin/env python
"""homekinesis with python, compare playfulmachines
$ python hk.py -h"""

# FIXME: put the learner / control structure into class to easily load
#        der/martius or reservoir model

import time, argparse, sys
import numpy as np
import scipy.sparse as spa
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32, Float32MultiArray, ColorRGBA
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion #, Point, Pose, TwistWithCovariance, Vector3
# from smp_msgs.msg import reservoir
import tf

# from smp_thread import smp_thread_ros

from reservoirs import Reservoir

# TLE = False
# TLE = True

def dtanh(x):
    return 1 - np.tanh(x)**2

def idtanh(x):
    return 1./dtanh(x) # hm?    
    
class LPZRos(object):
    modes = {"hs": 0, "hk": 1, "eh_pi_d": 2}
    def __init__(self, mode="hs"):
        self.name = "lpzros"
        self.mode = LPZRos.modes[mode]
        self.cnt = 0
        ############################################################
        # ros stuff
        rospy.init_node(self.name)
        # pub=rospy.Publisher("/motors", Float64MultiArray, queue_size=1)
        self.sub = {}
        self.pub = {}
        self.sub["imu"] = rospy.Subscriber("/imu",
                                        Imu, self.cb_imu)
        self.sub["odom"] = rospy.Subscriber("/odom",
                                        Odometry, self.cb_odom)
        self.pub["twist"] = rospy.Publisher("/cmd_vel", Twist)
        self.pub["color"] = rospy.Publisher("/set_color", ColorRGBA)

        self.cb_imu_cnt = 0
        self.cb_odom_cnt = 0
        # self.pub["target"] = rospy.Publisher("/learner/target", reservoir)
        # self.pub["learn_zn"] = rospy.Publisher("/learner/zn", reservoir)
        
        # self.pub_motors  = rospy.Publisher("/motors", Float64MultiArray, queue_size = 2)
        # self.sub_sensor = rospy.Subscriber("/sensors", Float64MultiArray, self.cb_sensors)
        # self.pub_sensor_exp = rospy.Publisher("/sensors_exp", Float64MultiArray, queue_size = 2)
        # pub=rospy.Publisher("/chatter", Float64MultiArray)

        # sphero color
        self.color = ColorRGBA()
        self.motors = Twist()
        
        self.msg_motors     = Float64MultiArray()
        self.msg_sensor_exp = Float64MultiArray()

        ############################################################
        # model + meta params
        self.numsen_raw = 8 # 5 # 2
        self.numsen = 8 # 5 # 2
        self.nummot = 2
        self.bufsize = 2
        self.creativity = 0.5
        # self.epsA = 0.1
        self.epsA = 0.02
        # self.epsA = 0.001
        # self.epsC = 0.001
        # self.epsC = 0.1
        self.epsC = 0.5

        ############################################################
        # forward model
        # self.A = np.eye(self.numsen) * 1.
        self.A  = np.zeros((self.numsen, self.nummot))
        self.A[range(self.nummot),range(self.nummot)] = 1.
        self.b = np.zeros((self.numsen,1))
        # controller
        # self.C  = np.eye(self.nummot) * 0.4
        self.C  = np.zeros((self.nummot, self.numsen))
        self.C[range(self.nummot),range(self.nummot)] = 1 * 0.4
        print "self.C", self.C
        self.h  = np.zeros((self.nummot,1))
        self.g  = np.tanh # sigmoidal activation function
        self.g_ = dtanh # derivative of sigmoidal activation function
        # state
        self.x = np.ones ((self.numsen, self.bufsize))
        self.y = np.zeros((self.nummot, self.bufsize))
        self.z = np.zeros((self.numsen, 1))
        # auxiliary variables
        self.L     = np.zeros((self.numsen, self.nummot))
        self.v_avg = np.zeros((self.numsen, 1)) 
        self.xsi   = np.zeros((self.numsen, 1))

        self.odom = Odometry()
        
        # expansion
        self.exp_size = self.numsen
        self.exp_hidden_size = 100
        self.res = Reservoir(N = self.exp_hidden_size, p = 0.1, g = 1.5, tau = 0.1, input_num = self.numsen_raw, input_scale = 5.0)
        self.res_wo_expand     = np.random.randint(0, self.exp_hidden_size, self.exp_size)
        self.res_wo_expand_amp = np.random.uniform(0, 1, (self.exp_size, 1)) * 0.8
        self.res_wi_expand_amp = np.random.uniform(0, 1, (self.exp_size, self.numsen_raw)) * 1.0
        
    def expansion_random_system(self, x, dim_target = 1):
        # dim_source = x.shape[0]
        print "x", x.shape
        self.res.execute(x)
        print "self.res.r", self.res.r.shape
        a = self.res.r[self.res_wo_expand]
        print "a.shape", a.shape
        b = a * self.res_wo_expand_amp
        print "b.shape", b.shape
        c = b + np.dot(self.res_wi_expand_amp, x)
        return c
        
    def cb_odom(self, msg):
        """ROS odometry callback, copy incoming data into local memory"""
        # print type(msg)
        self.odom = msg
        # self.iosm.x[0] = self.odom.pose.pose.position.x
        # self.iosm.x[1] = self.odom.pose.pose.position.y
        # self.iosm.x_raw[0] = self.odom.pose.pose.position.x
        # self.iosm.x_raw[1] = self.odom.pose.pose.position.y
        # self.iosm.x_raw[0] = self.odom.twist.twist.linear.x
        # self.iosm.x_raw[1] = self.odom.twist.twist.linear.x - self.cfg.target
        
        # self.iosm.x_raw[3] = self.odom.twist.twist.linear.y
        # self.cb_sensors((self.odom.twist.twist.linear.x,self.odom.twist.twist.linear.y))
        imu_lin_acc_gain = 0.01
        imu_orienta_gain = 0.1
        (r, p, y) = tf.transformations.euler_from_quaternion([self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w])
        self.cb_sensors((self.odom.twist.twist.linear.x,self.odom.twist.twist.linear.y,
                         self.imu.linear_acceleration.x * imu_lin_acc_gain, self.imu.linear_acceleration.y * imu_lin_acc_gain,
                         self.imu.linear_acceleration.z * imu_lin_acc_gain,
                         r * imu_orienta_gain, p * imu_orienta_gain, y * imu_orienta_gain))
        self.cb_odom_cnt += 1
        return
    
    def cb_imu(self, msg):
        """ROS IMU callback: use odometry and incoming imu data to trigger
        sensorimotor loop execution"""
        # print "imu", msg
        # return
        self.imu = msg

        # (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        # # print r, p, y
        # self.iosm.x_raw[4] = r
        # self.iosm.x_raw[5] = p
        # self.iosm.x_raw[6] = y
        # self.iosm.x_raw[7] = msg.angular_velocity.x
        # self.iosm.x_raw[8] = msg.angular_velocity.y
        # self.iosm.x_raw[9] = msg.angular_velocity.z
        # self.iosm.x_raw[10] = msg.linear_acceleration.x
        # self.iosm.x_raw[11] = msg.linear_acceleration.y
        # self.iosm.x_raw[12] = msg.linear_acceleration.z

        self.cb_imu_cnt += 1
        
    def cb_sensors(self, msg):
        """lpz sensors callback: receive sensor values, sos algorithm attached"""
        # self.msg_motors.data = []
        self.x = np.roll(self.x, 1, axis=1) # push back past
        self.y = np.roll(self.y, 1, axis=1) # push back past
        # update with new sensor data
        self.x[:,0] = np.array(msg)
        print "msg", msg
        
        # xa = np.array([msg.data]).T
        # self.x[:,[0]] = self.expansion_random_system(xa, dim_target = self.numsen)
        # self.msg_sensor_exp.data = self.x.flatten().tolist()
        # self.pub_sensor_exp.publish(self.msg_sensor_exp)
        
        # compute new motor values
        x_tmp = np.atleast_2d(self.x[:,0]).T + self.v_avg * self.creativity
        print "x_tmp.shape", x_tmp.shape
        # print self.g(np.dot(self.C, x_tmp) + self.h)
        m1 = np.dot(self.C, x_tmp)
        print "m1.shape", m1.shape
        t1 = self.g(m1 + self.h).reshape((self.nummot,))
        self.y[:,0] = t1

        self.cnt += 1
        if self.cnt <= 2: return

        # print "x", self.x
        # print "y", self.y
        
        # local variables
        x = np.atleast_2d(self.x[:,1]).T
        y = np.atleast_2d(self.y[:,1]).T
        x_fut = np.atleast_2d(self.x[:,0]).T

        # print "x", x.shape, x, x_fut.shape, x_fut
        z = np.dot(self.C, x + self.v_avg * self.creativity) + self.h
        # z = np.dot(self.C, x)
        # print z.shape, x.shape
        # print z - x

        g_prime = dtanh(z) # derivative of g
        g_prime_inv = idtanh(z) # inverse derivative of g

        print "g_prime", self.cnt, g_prime
        print "g_prime_inv", self.cnt, g_prime_inv
        
        xsi = x_fut - (np.dot(self.A, y) + self.b)
        print "xsi =", xsi
        
        # forward model learning
        self.A += self.epsA * np.dot(xsi, y.T) + (self.A * -0.003) * 0.1
        # self.A += self.epsA * np.dot(xsi, np.atleast_2d(self.y[:,0])) + (self.A * -0.003) * 0.1
        self.b += self.epsA * xsi              + (self.b * -0.001) * 0.1

        print "A", self.cnt, self.A
        # print "b", self.b

        if self.mode == 1: # TLE / homekinesis
            eta = np.dot(np.linalg.pinv(self.A), xsi)
            zeta = np.clip(eta * g_prime_inv, -1., 1.)
            print "eta", self.cnt, eta
            print "zeta", self.cnt, zeta
            # print "C C^T", np.dot(self.C, self.C.T)
            # mue = np.dot(np.linalg.pinv(np.dot(self.C, self.C.T)), zeta)
            lambda_ = np.eye(2) * np.random.uniform(-0.01, 0.01, 2)
            mue = np.dot(np.linalg.pinv(np.dot(self.C, self.C.T) + lambda_), zeta)
            v = np.clip(np.dot(self.C.T, mue), -1., 1.)
            self.v_avg += (v - self.v_avg) * 0.1
            print "v", self.cnt, v
            print "v_avg", self.cnt, self.v_avg
            EE = 1.0

            # print EE, v
            if True: # logarithmic error
                # EE = .1 / (np.sqrt(np.linalg.norm(v)) + 0.001)
                EE = .1 / (np.square(np.linalg.norm(v)) + 0.001)
            # print EE
            # print "eta", eta
            # print "zeta", zeta
            # print "mue", mue
            
            dC = (np.dot(mue, v.T) + (np.dot((mue * y * zeta), -2 * x.T))) * EE * self.epsC
            dh = mue * y * zeta * -2 * EE * self.epsC

            # pass
            # dC = np.zeros_like(self.C)
            # dh = np.zeros_like(self.h)
            
        elif self.mode == 0: # homestastic learning
            eta = np.dot(self.A.T, xsi)
            print "eta", self.cnt, eta.shape, eta
            dC = np.dot(eta * g_prime, x.T) * self.epsC
            dh = eta * g_prime * self.epsC
            # print dC, dh
            # self.C +=

        self.h += np.clip(dh, -.1, .1)
        self.C += np.clip(dC, -.1, .1)

        # print "C", self.C
        # print "h", self.h
        print "y", self.y
        
                
        self.motors.linear.x = self.y[0,0] * 100
        self.motors.linear.y = self.y[1,0] * 100
        print "self.motors", self.motors
        self.pub["twist"].publish(self.motors)
        # self.msg_motors.data.append(m[0])
        # self.msg_motors.data.append(m[1])
        # self.msg_motors.data = self.y[:,0].tolist()
        # print("sending msg", msg)
        # self.pub_motors.publish(self.msg_motors)
        # time.sleep(0.1)
        # if self.cnt > 20:
        #     rospy.signal_shutdown("stop")
        #     sys.exit(0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="lpzrobots ROS controller: test homeostatic/kinetic learning")
    parser.add_argument("-m", "--mode", type=str, help="select mode [hs] from " + str(LPZRos.modes), default = "hs")
    # parser.add_argument("-m", "--mode", type=int, help="select mode: ")
    args = parser.parse_args()

    # sanity check
    if not args.mode in LPZRos.modes:
        print "invalid mode string, use one of " + str(LPZRos.modes)
        sys.exit(0)
    
    lpzros = LPZRos(args.mode)
    rospy.spin()
    # while not rospy.shutdown():
