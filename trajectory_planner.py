import numpy as np 
import time
import math
from numpy.linalg import inv
from kinematics import *
"""
TODO: build a trajectory generator and waypoint planner 
        so it allows your state machine to iterate through
        the plan at the desired command update rate
"""

class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        #self.kinematics = kinematics
        self.num_joints = rexarm.num_joints
        self.initial_wp = [0.0]*self.num_joints
        self.final_wp = [0.0]*self.num_joints 
        self.dt = 0.05 # command rate
        self.t_f = .25
        self.bigAngle = 0
    
    def set_initial_wp(self):
        pass

    def set_final_wp(self, waypoint):
        pass

    def go(self, max_speed = 2.5):
        pass

    def stop(self):
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, speed):
        bigAngle = 0
        for i in range(self.rexarm.num_joints):
            deltaAngle = 180 - abs(abs(initial_wp[i] - final_wp[i]) - 180); 
            if deltaAngle > bigAngle:
                bigAngle = deltaAngle
            if speed == "fast":
                 t_f = 1.5 + bigAngle*.75
            elif speed == "slow":
                t_f = 2.5 + bigAngle*1.25
        


        return t_f

    def generate_cubic_spline(self, initial_wp, final_wp, t_f):
        t_o = 0
        constant_array = np.zeros((6,4))
        for i,joint in enumerate(self.rexarm.joints):
            M = np.array([[1, t_o, pow(t_o,2), pow(t_o,3)],
                        [0, 1, 2*t_o, 3*pow(t_o,2)],
                        [1, t_f, pow(t_f,2), pow(t_f,3)],
                        [0, 1, 2*t_f, 3*pow(t_f,2)]])
            a = np.array([[0],[0],[0],[0]])
            b = np.array([[initial_wp[i]],[0],[final_wp[i]],[0]])

            a = np.matmul(inv(M),b)
            #print "A matrix = " + str(a)
            constant_array[i][0] = a[0][0]
            constant_array[i][1] = a[1][0]
            constant_array[i][2] = a[2][0]
            constant_array[i][3] = a[3][0]
        return constant_array

    def calculate_velocity(self, constants, t_f, steps):
        velocity = np.zeros((steps,6))
        time = 0
        time_step = t_f/steps
        
        count = 0
        while count < steps:
            time = time + time_step
            i = 0
            for joint in enumerate(self.rexarm.joints):
                velocity[count][i] = constants[i][1] + 2*constants[i][2]*time + 3*constants[i][3]*pow(time,2)
                if velocity[count][i] < 0:
                    velocity[count][i] = abs(velocity[count][i])
                if velocity[count][i] >= 1:
                    velocity[count][i] = .99
                if velocity[count][i] < .05:
                    velocity[count][i] = 0
                
                i += 1
            velocity[count] = np.around(velocity[count],decimals = 3)
            count += 1

        return velocity

    def calculate_position(self, constants, t_f, steps):
        position = np.zeros((steps,6))
        time = 0
        time_step = t_f/steps
        count = 0
        while count < steps:
            time = time + time_step
            i = 0
            for i,joint in enumerate(self.rexarm.joints):
                position[count][i] = constants[i][0] + constants[i][1]*time + constants[i][2]*pow(time,2) + constants[i][3]*pow(time,3)
                i += 1
            position[count] = np.around(position[count],decimals = 3)
            count += 1

        return position

    def execute_plan(self, final_wp, speed):
        
        initial_wp = self.rexarm.get_positions()
        #print "Arm is currently at: " + str(initial_wp)
        #print "Arm is moving to: " + str(final_wp)
        t_f = self.calc_time_from_waypoints(initial_wp, final_wp, speed)
        print "This will take " + str(t_f) + " seconds"
        constants = self.generate_cubic_spline(initial_wp, final_wp, t_f)
        print constants
        T = time.time()
        start = time.time()
        #print "Current time is: " + str(T)
        steps = 125
        

        velocity = self.calculate_velocity(constants, t_f, steps)
        #print velocity
        position = self.calculate_position(constants, t_f, steps)
        #print "Final position: " + str(final_wp)
        #print "Calculated Final position: " + str(position[steps-1])
        
        self.rexarm.set_speeds_normalized([0.02,0.02,0.02,0.02,0.02,0.02], update_now = True)
        time.sleep(0.1)
        #self.rexarm.set_positions(position[steps-1], update_now = True)
        
        count = 1
        while count <= steps:
            next_move = start + count*t_f/steps
            if T > next_move:
                ellapsed = T-start
                #print "Getting velocity..."
                #velocity = self.calculate_velocity(constants, t_f, steps)
                #print "velocity is: " + str(velocity)
                #print "Getting position..."
                #position = self.calculate_position(constants, t_f, steps)
                #print "position is: " + str(position)
                #self.kinematics.FK_dh

               # print "position (FK) = " + str(self.rexarm.get_positions())
                joint_angles = self.rexarm.get_positions()
                T0 = FK_dh(joint_angles, 6)
                T2= get_pose_from_T(T0)
                with open('F_S_trajpath.csv','a') as f:
                    f.write(str(T2)+", "+str(ellapsed))
                    f.write("\n")
                print "position (FK) = " + str(T2)
                print "velocity = " + str(velocity[count - 1])
                print "position (rad) = " + str(position[count - 1])
                print "position (deg) = " + str(position[count - 1]*180.0/3.141592)
                self.rexarm.set_positions(position[count-1], update_now = True)
                self.rexarm.set_speeds_normalized(velocity[count-1], update_now = True)
               # self.rexarm.set_positions(position[count-1], update_now = True
                count += 1
            T = time.time()
        """
        count = 1
        self.rexarm.set_speeds_normalized(velocity[count-1], update_now = True)
        self.rexarm.set_positions(final_wp, update_now = True)
        count += 1
        while count <= steps:
            next_move = start + count*t_f/steps
            if T > next_move:
                ellapsed = start-T
                
                
                print "velocity = " + str(velocity[count - 1])
                print "position (rad) = " + str(position[count - 1])
                print "position (deg) = " + str(position[count - 1]*180.0/3.141592)
                self.rexarm.set_speeds_normalized(velocity[count-1], update_now = True)
                count += 1
            T = time.time()
     
        """






        pass