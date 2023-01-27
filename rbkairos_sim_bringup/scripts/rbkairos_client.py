#!/usr/bin/env python3

import rospy
import actionlib
import math
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import GripperCommandAction,GripperCommandGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class RBKairosClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client_gripper = actionlib.SimpleActionClient('gripper/gripper_controller/gripper_cmd',GripperCommandAction)
        self.arm_pub = rospy.Publisher('arm/pos_traj_controller/command', JointTrajectory, queue_size=10)
        self.actions = rospy.get_param('~actions')
        self.arm_joints = rospy.get_param('~arm_joints')
        print("Waiting for action server to start.")
        self.client.wait_for_server()
        print("Action server started, sending goal.")
        
    def send_move_base_goal(self, x, y, theta, frame_id="robot_map"):

        # transformar theta a quaternion
        q = quaternion_from_euler(0.0, 0.0, theta)
          
        # crear mensaje move_base
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id    #el robot_map es el nombre del fixed frame que aparece en rviz al lanzar el launch del robot
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = q[3]

        # enviar el goal a move_base
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
  
        if not wait:
            rospy.logerr("Action server movebase not available!")
            rospy.signal_shutdown("Action server movebase not available!")
        else:
            return self.client.get_result()
 
    def send_move_gripper(self, position, max_effort):
   
        # crear mensaje move_gripper
        goal = GripperCommandGoal()

        goal.command.position = position
        goal.command.max_effort = max_effort

        # enviar el goal a move_gripper
        self.client_gripper.send_goal(goal)
        time.sleep(15)
        wait = self.client_gripper.wait_for_result()

        if not wait:
            rospy.logerr("Action server gripper not available!")
            rospy.signal_shutdown("Action server gripper not available!")
        else:
            return self.client_gripper.get_result()

    def send_move_arm(self, position):

        goal = JointTrajectory()
        p = JointTrajectoryPoint()
        goal.header.stamp = rospy.Time.now()

        p.positions = position
        p.time_from_start = rospy.Duration.from_sec(4)
        goal.joint_names = self.arm_joints   #se obtienen las articulaciones del brazo
        goal.points.append(p)

        self.arm_pub.publish(goal)
        time.sleep(15)
    

    def is_goto_valid(self, goto):

        if "x" not in goto:
            rospy.logerr("Variable 'x' is missing")
            return False

        if "y" not in goto:
            rospy.logerr("Variable 'y' is missing")
            return False

        if "theta" not in goto:
            rospy.logerr("Variable 'theta' is missing")
            return False
        
        if "frame_id" not in goto:
            goto["frame_id"] = "robot_map"  
            rospy.logwarn("Varible 'frame_id' is missing. The default value will be set")
        
        if type(goto["x"]) is not float:
            rospy.logerr("The type of the variable 'x' is not a float")
            return False

        if type(goto["y"]) is not float:
            rospy.logerr("The type of the variable 'y' is not a float")
            return False

        if type(goto["theta"]) is not float:
            rospy.logerr("The type of the variable theta is not a float")
            return False
        
        if type(goto["frame_id"]) is not str:
            rospy.logerr("The type of the variable 'frame_id' is not a string")
            return False

        return True          

    def is_movegripper_valid(self, movegripper):

        if "position" not in movegripper:
            rospy.logerr("Variable 'position' is missing")
            return False

        if "max_effort" not in movegripper:
            rospy.logerr("Variable 'max_effort' is missing")
            return False

        if type(movegripper["position"]) is not float:
            rospy.logerr("The type of the variable 'position' is not a float")
            return False

        if type(movegripper["max_effort"]) is not float:
            rospy.logerr("The type of the variable 'max_effort' is not a float")
            return False

    def is_movearm_valid(self, movearm):

        if "positions" not in movearm:
            rospy.logerr("Variable 'position' is missing")
            return False

        if len(movearm["positions"]) is not len(self.arm_joints):
            rospy.logerr("The size of the varible 'position' is not the same as the number of joints")
            return False
    
        if type(movearm["positions"]) is not list:
            rospy.logerr("The type of the variable 'position' is not a float")
            return False
        
        for i in movearm["positions"]:
            if type(i) is not float:
                rospy.logerr("The type of one of the list elements is not a float")
                return False
        
    def is_sequence_valid(self):

        for action in self.actions:
            if "type" not in action:
                rospy.logerr("The type of action is not defined")
                return False

            action_type = action["type"]

            action_valid = False
            if action_type == "GOTO":
                action_valid = self.is_goto_valid(action)
            if action_type == "MOVEGRIPPER":
                action_valid = self.is_movegripper_valid(action)
            if action_type == "MOVEARM":
                action_valid = self.is_movearm_valid(action)

            if action_valid == False:
                return False

        return True     

    def goto(self, goto):
        rbkairos_client.send_move_base_goal(goto["x"], goto["y"], goto["theta"], goto["frame_id"])

    def movegripper(self, movegripper):
        rbkairos_client.send_move_gripper(movegripper["position"],movegripper["max_effort"])
    
    def movearm(self, movearm):
        rbkairos_client.send_move_arm(movearm["positions"])
         
    def execute_sequence(self):

        for action in self.actions:
            action_type = action["type"]
            rospy.loginfo("An action " + action_type + " will be executed.")
            if action_type == "GOTO":
                action_valid = self.goto(action)    
            if action_type == "MOVEGRIPPER":
                action_valid = self.movegripper(action)      
            if action_type == "MOVEARM":
                action_valid = self.movearm(action) 
   

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        rbkairos_client = RBKairosClient()

        valid = rbkairos_client.is_sequence_valid()
    
        if valid == True:
            rospy.loginfo("The sequence is valid.")
            rbkairos_client.execute_sequence()
            rospy.loginfo("The sequence is over.")

        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
