import time
import numpy as np
import csv
from numpy.linalg import inv
from kinematics import *

sequence = []

record_number = 0

"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""
class StateMachine():
    def __init__(self, rexarm, planner, kinect):
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"


    def set_next_state(self, state):
        self.next_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        if(self.current_state == "manual"):
            if (self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()                
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "idle"):
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()
            if(self.next_state == "waypoint"):
                self.waypoint()
            if(self.next_state == "teach"):
                self.teach()
            if(self.next_state == "repeat"):
                self.repeat()
            if(self.next_state == "record"):
                self.record()
            if(self.next_state == "IK"):
                self.IK()
            if(self.next_state == "clickgrab"):
                self.clickgrab()
            if(self.next_state == "open_gripper"):
                self.open_gripper()
            if(self.next_state == "close_gripper"):
                self.close_gripper()
            if(self.next_state == "color_sort"):
                self.color_sort()
            if(self.next_state == "color_stack"):
                self.color_stack()
            if(self.next_state == "square"):
                self.square()
            if(self.next_state == "incline"):
                self.incline()
                
        if(self.current_state == "estop"):
            self.next_state = "estop"
            self.estop()  

        if(self.current_state == "calibrate"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "waypoint"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "teach"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop() 
            if(self.next_state == "repeat"):
                self.repeat()           

        if(self.current_state == "repeat"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop() 
            if(self.next_state == "teach"):
                self.teach() 

        if(self.current_state == "record"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()  

        if(self.current_state == "square"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "IK"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "clickgrab"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "close_gripper"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "open_gripper"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "color_sort"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
                
        if(self.current_state == "color_stack"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            
        if(self.current_state == "incline"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()


    """Functions run for each state"""


    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"
        self.rexarm.send_commands()
        self.rexarm.get_feedback()

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"
        self.rexarm.get_feedback()

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()
        
    def calibrate(self):
        self.current_state = "calibrate"
        self.next_state = "idle"
        self.tp.go(max_speed=2.0)
        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board",
                            "center of shoulder motor"]
    
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False        
        
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False
   
        #print self.kinect.rgb_click_points
        #print self.kinect.depth_click_points


        self.kinect.getAffineTransform(self.kinect.rgb_click_points, self.kinect.depth_click_points)
        position = np.array([0,0,0,0,0,0])*3.141592/180.0
        self.rexarm.set_positions(position, update_now = True)
        
        """TODO Perform camera calibration here"""

        self.status_message = "Calibration - Completed Calibration"
        self.kinect.calibrated = True
        time.sleep(1)

    def waypoint(self):
        value = csv.reader(open('waypoints_value.CSV'))
        waypoint_value=[]
        for row in value:
            for i in range (6):
                waypoint_value.append(float(row[i]))
        waypoint_value=np.reshape(waypoint_value,(6,6))
        
        self.status_message = "State: waypoint"
        self.current_state = "waypoint"
        self.next_state = "idle"
        self.rexarm.send_commands()
        start = time.time()
        for i in range(6):
            #self.tp.execute_plan(waypoint_value[i,:], "fast")
            self.rexarm.set_positions(waypoint_value[i,:], update_now = True)
            Wcounter = 1
            
            #T3 = DeltaTime/Wcounter
            while Wcounter < 100:
                joint_angles = self.rexarm.get_positions()
                T0 = FK_dh(joint_angles, 6)
                T2= get_pose_from_T(T0)
                DeltaTime = time.time() - start
                with open('F_N_trajpath.csv','a') as f:
                    f.write(str(T2)+", "+str(DeltaTime))
                    f.write("\n")
                Wcounter += 1
                time.sleep(0.05)
                #T3 = DeltaTime/Wcounter

            
            self.rexarm.pause(4)
        self.rexarm.get_feedback()


    def teach(self):
        self.status_message = "State: teach"
        self.current_state = "teach"
        self.next_state = "idle"
        for joint in self.rexarm.joints:
            joint.set_torque_limit(0.0)
        global record_number
        record_number = 0
        del sequence[:]
        

    def record(self):
        self.status_message = "State: record"
        self.current_state = "record"
        self.next_state = "idle"
        position = []
        self.rexarm.get_positions()
        for i,joint in enumerate(self.rexarm.joints):
            position.append(self.rexarm.joint_angles_fb[i])
        
        sequence.append(position)

        global record_number 
        record_number = record_number + 1

    def repeat(self):
        self.status_message = "State: repeat"
        self.current_state = "repeat"
        self.next_state = "idle"
        self.rexarm.set_speeds_normalized([0.05, 0.05, 0.05, 0.05, 0.05, 0.05], update_now = True)
        for joint in self.rexarm.joints:
            joint.set_torque_limit(0.8)
        
        np.savetxt("Sequence.csv", sequence, delimiter=",")

        value = csv.reader(open('Sequence.csv'))
        Sequence_Value=[]
        for row in value:
            for i in row:
                Sequence_Value.append(float(i))
        Sequence_Value=np.reshape(Sequence_Value,(record_number,6))
        self.rexarm.send_commands()
        for i in range(record_number):
            self.rexarm.set_positions(Sequence_Value[i,:], update_now = True)
            time.sleep(0.2)
            
        self.rexarm.get_feedback()

    def open_gripper(self):
        self.status_message = "State: Open Gripper"
        self.current_state = "open_gripper"
        self.next_state = "idle"

        self.rexarm.open_gripper()

    def close_gripper(self):
        self.status_message = "State: Close Gripper"
        self.current_state = "close_gripper"
        self.next_state = "idle"

        self.rexarm.close_gripper()

    def IK(self):
        self.status_message = "State: Inverse Kinematics"
        self.current_state = "IK"
        self.next_state = "idle"
        Array = self.kinect.inverseK
        print Array
        j = 0
        final_position = [[-150,0,50,-np.pi/2],[-155,0,100,-np.pi/2],[-140,0,145,-np.pi/2]]
        self.rexarm.set_speeds_normalized([0.17, 0.17, 0.17, 0.17, 0.17, 0.17], update_now = True)
        for row in Array:
           
            
            
            angles = IK(row[0:4],row[4])
            row[2] = row[2] - 70
            block_initial = IK(row[0:4],row[4])
            row[2] = row[2] + 150
            intermediate = IK(row[0:4],row[4])
            block_final = IK(final_position[j][0:4],0)
            final_position[j][2] = final_position[j][2] + 150
            intermediate2 = IK(final_position[j][0:4],0)
           
            self.rexarm.set_positions(angles, update_now = True)
            self.rexarm.moving()
            self.rexarm.set_positions(block_initial, update_now = True)
            self.rexarm.moving()
            self.rexarm.close_gripper()
            time.sleep(0.5)
            self.rexarm.set_positions(intermediate, update_now = True)
            self.rexarm.moving()
            self.rexarm.set_positions(intermediate2, update_now = True)
            self.rexarm.moving()
            self.rexarm.set_positions(block_final, update_now = True)
            self.rexarm.moving()
            self.rexarm.open_gripper()
            time.sleep(0.75)
            self.rexarm.set_positions(intermediate2, update_now = True)
            self.rexarm.moving()
            
            j = j + 1

        intermediate2 = np.array([0,0,0,0,0,0])
        self.rexarm.set_positions(intermediate2, update_now = True)
        
    def clickgrab(self):
        self.current_state = "clickgrab"
        self.next_state = "idle"
        self.rexarm.set_speeds_normalized([0.1, 0.1, 0.1, 0.1, 0.1, 0.1], update_now = True)
        self.kinect.clickandplace = np.zeros((2,2),int)
        self.tp.go(max_speed=2.0)
        Array = self.kinect.inverseK
        print "\n\n" + str(Array) + "\n\n"
        i = 0

        while i < 1:
            self.status_message = "Select a block to grab." 
            self.rexarm.get_feedback()
            if(self.kinect.new_click == True):
                self.kinect.clickandplace[i] = self.kinect.last_click.copy()
                i = i + 1
                self.kinect.new_click = False        

        coord = np.array([[self.kinect.clickandplace[0][0]],[self.kinect.clickandplace[0][1]],[1]])
        z = self.kinect.currentDepthFrame[int(coord[1])][int(coord[0])]
        depth = 0.1236 * np.tan(z/2842.5 + 1.1863)
        camera = depth*np.matmul(inv(self.kinect.camera_matrix),coord)
        points = np.array([[camera[0]],[camera[1]],[1]])
        pos = np.matmul(self.kinect.affineB.astype(float),points.astype(float))
        board_depth = 0.1236 * np.tan(718/2842.5 + 1.1863)
                
        depth = (board_depth-depth)*1000


        j = 0
        block_check = False
        for block in Array:
            if abs(block[0] - pos[0]) < 10 and abs(block[1] - pos[1]) < 10:
                point = np.array(block)
                block_check = True
            j = j+1
        
        if block_check == False:
            point = np.array([pos[0], pos[1], 100, -np.pi/2,0])
            print "block check false"
        print point
        angles = IK(point[0:4],point[4])
        point[2] = point[2] - 60
        intermediate = IK(point[0:4],point[4]) 
        '''
        self.tp.execute_plan(angles, "slow")
        self.tp.execute_plan(intermediate, "slow")
        self.rexarm.close_gripper()
        time.sleep(0.5)
        self.tp.execute_plan([0,0,0,0,0,0], "slow")
        
        '''
        self.rexarm.set_positions(angles, update_now = True)
        self.rexarm.moving()
        #self.rexarm.set_positions(intermediate, update_now = True)
        #self.rexarm.moving()
        self.rexarm.close_gripper()
        time.sleep(0.5)
        self.rexarm.set_positions([0,0,0,0,0,0], update_now = True)
        self.rexarm.moving()
        
        while i < 2:
            self.status_message = "Select where to place it."
            self.rexarm.get_feedback()
            if(self.kinect.new_click == True):
                self.kinect.clickandplace[i] = self.kinect.last_click.copy()
                i = i + 1
                self.kinect.new_click = False
   
        coord = np.array([[self.kinect.clickandplace[1][0]],[self.kinect.clickandplace[1][1]],[1]])
        z = self.kinect.currentDepthFrame[int(coord[1])][int(coord[0])]
        depth = 0.1236 * np.tan(z/2842.5 + 1.1863)
        camera = depth*np.matmul(inv(self.kinect.camera_matrix),coord)
        points = np.array([[camera[0]],[camera[1]],[1]])
        pos = np.matmul(self.kinect.affineB.astype(float),points.astype(float))
        angles = IK([pos[0], pos[1], 60,-np.pi/2],0)
        intermediate = IK([pos[0], pos[1], 120,-np.pi/2],0)

        '''
        self.tp.execute_plan(angles, "slow")
        self.rexarm.open_gripper()
        time.sleep(0.5)
        self.tp.execute_plan(intermediate, "slow")
        self.tp.execute_plan([0,0,0,0,0,0], "slow")
        '''

        self.rexarm.set_positions(angles, update_now = True)
        self.rexarm.moving()
        self.rexarm.open_gripper()
        time.sleep(0.5)
        self.rexarm.set_positions(intermediate, update_now = True)
        self.rexarm.moving()
        self.rexarm.set_positions([0,0,0,0,0,0], update_now = True)
        self.rexarm.moving()
        

    def color_sort(self):
        self.status_message = "State: Color Sort"
        self.current_state = "color_sort"
        self.next_state = "idle"
        self.rexarm.set_speeds_normalized([0.05, 0.05, 0.05, 0.05, 0.05, 0.05], update_now = True)
        
        colors = ["BLACK","RED","ORANGE","YELLOW","GREEN","BLUE","PURPLE","PINK"]
        positions = [[110,165],[110,115],[110,65],[110,15],[110,-40],[110,-90],[110,-140],[110,-185]]
        #positions = [[-185,110],[-135,110],[-85,110],[-35,110],[15,110],[65,110],[115,110],[165,110]]
        i = 0

        for color in colors:
            Array = self.kinect.inverseK
            for block in Array:
                angles = IK(block[0:4],block[4])
                current = self.rexarm.get_positions()
                if block[2] > 170:
                    block[2] = block[2] + 40
                
                intermediate = np.array([current[0],0,0,0,0,0])
                intermediate2 = np.array([angles[0],0,0,0,0,0])
                if block[2] > 200:
                    block[2] = block[2] - 110
                else:
                    block[2] = block[2] - 70
                block_initial = IK(block[0:4],block[4])
                """
                self.tp.execute_plan(intermediate, "slow")
                self.tp.execute_plan(intermediate2, "slow")
                self.tp.execute_plan(angles, "slow")
                self.tp.execute_plan(block_initial,"slow")
                self.rexarm.close_gripper()
                time.sleep(0.5)

                self.tp.execute_plan(angles, "slow")
                self.tp.execute_plan(intermediate2, "slow")
                """
                self.rexarm.set_positions(intermediate, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(intermediate2, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(angles, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(block_initial, update_now = True)
                self.rexarm.moving()
                self.rexarm.close_gripper()
                time.sleep(0.5)                   

                self.rexarm.set_positions(angles, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(intermediate2, update_now = True)
                self.rexarm.moving()
                

                angles = IK([positions[i][0],positions[i][1],110,-np.pi/2],0)
                intermediate = np.array([angles[0],0,0,0,0,0])
                block_final = IK([positions[i][0],positions[i][1],50,-np.pi/2],0)
                
                """
                self.tp.execute_plan(intermediate, "slow")
                self.tp.execute_plan(angles, "slow")
                self.tp.execute_plan(block_final, "slow")
                self.rexarm.open_gripper()
                time.sleep(0.5)
                self.tp.execute_plan(angles, "slow")
                """

                self.rexarm.set_positions(intermediate, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(angles, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(block_final, update_now = True)
                self.rexarm.moving()
                self.rexarm.open_gripper()
                time.sleep(0.5)
                self.rexarm.set_positions(angles, update_now = True)
                self.rexarm.moving()
                


                i = i + 1
                break

    def color_stack(self):
        self.status_message = "State: Color Stack"
        self.current_state = "color_stack"
        self.next_state = "idle"
        self.rexarm.set_speeds_normalized([0.05, 0.05, 0.05, 0.05, 0.05, 0.05], update_now = True)

        colors = ["BLACK","RED","ORANGE","YELLOW","GREEN","BLUE","PURPLE","PINK"]
        positions = [[200,0,50],[200,0,100],[200,0,150],[200,0,200],[200,0,250],[200,0,300],[200,0,350],[200,0,400]]
        i = 0

        for color in colors:
            Array = self.kinect.inverseK
            for block in Array:
                angles = IK(block[0:4],block[4])
                current = self.rexarm.get_positions()
                if block[2] > 170:
                    block[2] = block[2] + 40
                
                intermediate = np.array([current[0],0,0,0,0,0])
                intermediate2 = np.array([angles[0],0,0,0,0,0])
                if block[2] > 200:
                    block[2] = block[2] - 110
                else:
                    block[2] = block[2] - 70
                block_initial = IK(block[0:4],block[4])

                """
                self.tp.execute_plan(intermediate, "slow")
                self.tp.execute_plan(intermediate2, "slow")
                self.tp.execute_plan(angles, "slow")
                self.tp.execute_plan(block_initial,"slow")
                self.rexarm.close_gripper()
                time.sleep(0.5)

                self.tp.execute_plan(angles, "slow")
                self.tp.execute_plan(intermediate2, "slow")
                """
                self.rexarm.set_positions(intermediate, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(intermediate2, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(angles, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(block_initial, update_now = True)
                self.rexarm.moving()
                self.rexarm.close_gripper()
                time.sleep(0.5)                   

                self.rexarm.set_positions(angles, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(intermediate2, update_now = True)
                self.rexarm.moving()


                angles = IK([positions[i][0],positions[i][1],positions[i][2]+70,-np.pi/2],0)
                intermediate = np.array([angles[0],0,0,0,0,0])
                block_final = IK([positions[i][0],positions[i][1],positions[i][2],-np.pi/2],0)
                
                """
                self.tp.execute_plan(intermediate, "slow")
                self.tp.execute_plan(angles, "slow")
                self.tp.execute_plan(block_final, "slow")
                self.rexarm.open_gripper()
                time.sleep(0.5)
                self.tp.execute_plan(angles, "slow")
                """

                self.rexarm.set_positions(intermediate, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(angles, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(block_final, update_now = True)
                self.rexarm.moving()
                self.rexarm.open_gripper()
                time.sleep(0.5)
                self.rexarm.set_positions(angles, update_now = True)
                self.rexarm.moving()


                i = i + 1
                break

    def square(self):
        self.status_message = "State: Square"
        self.current_state = "square"
        self.next_state = "idle"

        self.rexarm.set_speeds_normalized([0.15, 0.15, 0.15, 0.15, 0.15, 0.15], update_now = True)
        # Grab the block
        self.rexarm.open_gripper()
        value = csv.reader(open('grab.csv'))
        waypoint_value=[]
        for row in value:
            i = 0
            for number in row:
                waypoint_value.append(float(row[i]))
                i += 1
           
        waypoint_value=np.reshape(waypoint_value,(31,6))
        
        
        count = 0
        for row in waypoint_value:
            self.rexarm.set_positions(waypoint_value[count,:], update_now = True)
            time.sleep(0.13)
            count += 1
        self.rexarm.get_feedback()
        self.rexarm.close_gripper()
        time.sleep(0.5)

        #Drag it around
        self.rexarm.set_speeds_normalized([0.05, 0.05, 0.05, 0.05, 0.05, 0.05], update_now = True)
        start = time.time()
        value = csv.reader(open('square.csv'))
        waypoint_value1=[]
        for row in value:
            i = 0
            for number in row:
                waypoint_value1.append(float(row[i]))
                i += 1
           
        waypoint_value1=np.reshape(waypoint_value1,(81,6))
        
        
        count = 0
        for row in waypoint_value1:
            self.rexarm.set_positions(waypoint_value1[count,:], update_now = True)
            time.sleep(0.13)
            count += 1
            joint_angles = self.rexarm.get_positions()
            T0 = FK_dh(joint_angles, 6)
            T2= get_pose_from_T(T0)
            DeltaTime = time.time() - start
            with open('Drag_trajpath.csv','a') as f:
                f.write(str(T2)+", "+str(DeltaTime))
                f.write("\n")

        self.rexarm.get_feedback()

        #Go Home
        self.rexarm.open_gripper()
        time.sleep(0.5)
        self.rexarm.set_positions([waypoint_value1[count-1][0],0,0,0,0,0], update_now = True)
        self.rexarm.set_speeds_normalized([0.15, 0.15, 0.15, 0.15, 0.15, 0.15], update_now = True)

    def incline(self):
        self.status_message = "State: Incline"
        self.current_state = "incline"
        self.next_state = "idle"
        self.rexarm.set_speeds_normalized([0.05, 0.05, 0.05, 0.05, 0.05, 0.05], update_now = True)
        Array = self.kinect.inverseK
        print Array
        for block in Array:
            if str(block[5]) != str("BLACK") and str(block[5]) != str("PURPLE") :
                self.rexarm.open_gripper()
                wrist = -60*np.pi/180
                block[2] = block[2] + 40
                angles = IK([block[0],block[1],block[2],wrist],block[4])
                current = self.rexarm.get_positions()
                intermediate = np.array([current[0],0,0,0,0,0])
                intermediate2 = np.array([angles[0],0,0,0,0,0])
                block[2] = block[2] - 90
                wrist = -60*np.pi/180
                block_initial = IK([block[0],block[1],block[2],wrist],block[4])

                """
                self.tp.execute_plan(intermediate, "slow")
                self.tp.execute_plan(intermediate2, "slow")
                self.tp.execute_plan(angles, "slow")
                self.tp.execute_plan(block_initial,"slow")
                self.rexarm.close_gripper()
                time.sleep(0.5)

                self.tp.execute_plan(angles, "slow")
                self.tp.execute_plan(intermediate2, "slow")
                """
                self.rexarm.set_positions(intermediate, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(intermediate2, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(angles, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(block_initial, update_now = True)
                self.rexarm.moving()
                self.rexarm.close_gripper()
                time.sleep(0.5)                   

                self.rexarm.set_positions(angles, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(intermediate2, update_now = True)
                self.rexarm.moving()

        for block in Array:
            if str(block[5]) == str("BLACK") or str(block[5]) == str("PURPLE"):
                wrist = -60*np.pi/180
                block[2] = block[2] + 40
                angles = IK([block[0],block[1],block[2],wrist],block[4])
                current = self.rexarm.get_positions()
                intermediate = np.array([current[0],0,0,0,0,0])
                intermediate2 = np.array([angles[0],0,0,0,0,0])
                block[2] = block[2] - 60
                wrist = -60*np.pi/180
                block_initial = IK([block[0],block[1],block[2],wrist],block[4])

                """
                self.tp.execute_plan(intermediate, "slow")
                self.tp.execute_plan(intermediate2, "slow")
                self.tp.execute_plan(angles, "slow")
                self.tp.execute_plan(block_initial,"slow")
                self.rexarm.close_gripper()
                time.sleep(0.5)

                self.tp.execute_plan(angles, "slow")
                self.tp.execute_plan(intermediate2, "slow")
                """
                self.rexarm.set_positions(intermediate, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(intermediate2, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(angles, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(block_initial, update_now = True)
                self.rexarm.moving()
                self.rexarm.open_gripper()
                time.sleep(0.5)                   

                self.rexarm.set_positions(angles, update_now = True)
                self.rexarm.moving()
                self.rexarm.set_positions(intermediate2, update_now = True)
                self.rexarm.moving()
        self.rexarm.set_speeds_normalized([0.2, 0.2, 0.2, 0.2, 0.2, 0.2], update_now = True)


