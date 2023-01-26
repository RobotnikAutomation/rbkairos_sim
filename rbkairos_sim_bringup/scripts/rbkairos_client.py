#!/usr/bin/env python3

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class RBKairosClient:
    def __init__(self):
        # Variables de clase, crear cliente de move_base, etc
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.pos = rospy.get_param('~actions')
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
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()


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

    def is_sequence_valid(self):

        for action in self.pos:
            if "type" not in action:
                rospy.logerr("The type of action is not defined")
                return False

            action_type = action["type"]

            action_valid = False
            if action_type == "GOTO":
                action_valid = self.is_goto_valid(action)

            if action_valid == False:
                return False

        return True     

    def goto(self, goto):
        rbkairos_client.send_move_base_goal(goto["x"], goto["y"], goto["theta"], goto["frame_id"])

    def execute_sequence(self):

        for action in self.pos:
            action_type = action["type"]

            if action_type == "GOTO":
                action_valid = self.goto(action)       
         

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        rbkairos_client = RBKairosClient()

        valid = rbkairos_client.is_sequence_valid()

        if valid == True:
            rbkairos_client.execute_sequence()
         
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
