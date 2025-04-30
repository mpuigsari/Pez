#!/usr/bin/env python
# Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
#
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.


"""
@@>LogitechFX10Atlantis controler node.<@@
"""

import rospy
from cola2_control.joystickbase import JoystickBase
from cola2_ros import param_loader
from std_srvs.srv import Empty, Trigger, TriggerRequest, TriggerResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64
from controller_manager_msgs.srv import SwitchController
from cola2_msgs.srv import DigitalOutput, String
class LogitechFX10Atlantis(JoystickBase):
    """LogitechFX10Atlantis controler node."""

    """
        This class inherent from JoystickBase. It has to overload the
        method update_joy(self, joy) that receives a sensor_msgs/Joy
        message and fill the var self.joy_msg as described in the class
        JoystickBase.
        From this class it is also possible to call services or anything
        else reading the buttons in the update_joy method.
    """

    # JOYSTICK  DEFINITION:
    LEFT_JOY_HORIZONTAL = 0  # LEFT+, RIGHT-
    LEFT_JOY_VERTICAL = 1  # UP+, DOWN-
    LEFT_TRIGGER = 2  # NOT PRESS 1, PRESS -1
    RIGHT_JOY_HORIZONTAL = 3  # LEFT+, RIGHT-
    RIGHT_JOY_VERTICAL = 4  # UP+, DOWN-
    RIGHT_TRIGGER = 5  # NOT PRESS 1, PRESS -1
    CROSS_HORIZONTAL = 6  # LEFT+, RIGHT-
    CROSS_VERTICAL = 7  # UP+, DOWN-
    BUTTON_A = 0
    BUTTON_B = 1
    BUTTON_X = 2
    BUTTON_Y = 3
    BUTTON_LEFT = 4
    BUTTON_RIGHT = 5
    BUTTON_BACK = 6
    BUTTON_START = 7
    BUTTON_LOGITECH = 8
    BUTTON_LEFT_JOY = 9
    BUTTON_RIGHT_JOY = 10
    MOVE_UP = 1
    MOVE_DOWN = -1
    MOVE_LEFT = 1
    MOVE_RIGHT = -1

    def __init__(self, name):
        """Class constructor."""
        JoystickBase.__init__(self, name)
        rospy.loginfo("%s: LogitechFX10 constructor", name)

        # To select mode of operation
        self.mode = 0

        # To transform button into axis
        self.up_down = 0.0
        self.left_right = 0.0
        self.start_service = ""
        self.stop_service = ""
        
        namespace = rospy.get_namespace()
        self.get_config()

        # Create client to services:
        # ... start button service
        if self.start_service != "":
            done = False
            while not done and not rospy.is_shutdown():
                try:
                    rospy.wait_for_service(self.start_service, 10)
                    self.enable_keep_pose = rospy.ServiceProxy(self.start_service, Trigger)
                    done = True
                except rospy.ROSException as e:
                    rospy.logwarn("%s: Service call failed: %s", self.name, e)
                    rospy.sleep(1.0)

        # ... stop button service
        if self.stop_service != '':
            rospy.wait_for_service(self.stop_service, 10)
            try:
                self.disable_keep_pose = rospy.ServiceProxy(self.stop_service, Trigger)
            except rospy.ServiceException as e:
                rospy.logwarn("%s: Service call failed: %s", self.name, e)

        # ... enable thrusters service
        rospy.wait_for_service(
            namespace + 'teleoperation/enable_thrusters', 10)
        try:
            self.enable_thrusters = rospy.ServiceProxy(
                namespace + 'teleoperation/enable_thrusters', Trigger)
        except rospy.ServiceException as e:
            rospy.logwarn("%s: Service call failed: %s", self.name, e)

        # # ... disable thrusters service
        rospy.wait_for_service(
            namespace + 'teleoperation/disable_thrusters', 10)
        try:
            self.disable_thrusters = rospy.ServiceProxy(
                namespace + 'teleoperation/disable_thrusters', Trigger)
        except rospy.ServiceException as e:
            rospy.logwarn("%s: Service call failed: %s", self.name, e)


        namespace = rospy.get_namespace()
        self.get_config()

        # Create publisher to manipulator 
        bravo_ns = rospy.get_param("~bravo_ns")

        self.pub_bravoarm_desired_joint_velocity = rospy.Publisher(
            namespace + bravo_ns + '/joint_teleop_velocity_controller/command',
            Float64MultiArray,
            queue_size = 1)
        self.pub_bravoarm_desired_gripper_state = rospy.Publisher(
            namespace + bravo_ns + '/gripper_velocity_controller/command',
            Float64,
            queue_size = 1)
        

        self.bravoarm_joint_cmd = Float64MultiArray()
        self.bravoarm_joint_cmd.layout.dim = [MultiArrayDimension()]
        self.bravoarm_joint_cmd.layout.dim[0].label = "velocity"
        self.bravoarm_joint_cmd.layout.dim[0].size = 6
        self.bravoarm_joint_cmd.layout.dim[0].stride = 0
        self.bravoarm_joint_cmd.layout.data_offset = 0
        self.bravoarm_joint_cmd.data = [0.0]*6

        self.bravoarm_gripper_cmd = 0.0       

    def update_joy(self, joy):
        """Receive joystic raw data."""
        """Transform FX10 joy data into 12 axis data (pose + twist)
        and sets the buttons that especify if position or velocity
        commands are used in the teleoperation."""
        self.mutual_exclusion.acquire()

        self.joy_msg.header = joy.header

        
        # Enable/disable keep position
        if joy.buttons[self.BUTTON_START] == 1.0:
            rospy.loginfo("%s: Start button service called", self.name)
            res = self.enable_keep_pose(TriggerRequest())
            if not res.success:
                rospy.logwarn("%s: Impossible to enable keep position, captain response: %s", self.name, res.message)
        if joy.buttons[self.BUTTON_BACK] == 1.0:
            rospy.loginfo("%s: Stop button service called", self.name)
            res = self.disable_keep_pose(TriggerRequest())
            if not res.success:
                rospy.logwarn("%s: Impossible to disable keep position, captain response: %s", self.name, res.message)

        # Enable/disable thrusters
        if joy.axes[self.LEFT_TRIGGER] < -0.9 and joy.axes[self.RIGHT_TRIGGER] < -0.9:
            rospy.loginfo("%s: DISABLE THRUSTERS!", self.name)
            self.disable_thrusters()
            rospy.sleep(1.0)

        if joy.buttons[self.BUTTON_LEFT] == 1.0 and joy.buttons[self.BUTTON_RIGHT] == 1.0:
            rospy.loginfo("%s: ENABLE THRUSTERS!", self.name)
            self.enable_thrusters()
        
        # Set zero desired joint velocities
        self.bravoarm_joint_cmd.data = [0.0]*6
        self.bravoarm_gripper_cmd = 0.0

        old_mode = self.mode
        if joy.buttons[self.BUTTON_LOGITECH] == 1:
            rospy.loginfo("Digital output of bravo turned off")
            rospy.ServiceProxy('/girona500/main_control_board/digital_output', DigitalOutput)(digital_output=1, value=0)

        if joy.buttons[self.BUTTON_RIGHT] == 1:
            if joy.buttons[self.BUTTON_X] == 1.0: # Left arm (rightarm)
                rospy.loginfo("Switching to Predefined positions")
                rospy.ServiceProxy('/girona500/bravo/controller_manager/switch_controller', 
                                   SwitchController)(['joint_trajectory_controller'], ['joint_teleop_velocity_controller', 'joint_velocity_controller','gripper_position_controller','gripper_velocity_controller'],
                                                      1, True, 0.0)
                self.mode = 2 #predefined positions mode
            elif joy.buttons[self.BUTTON_B] == 1.0: # Right arm (leftarm)
                rospy.loginfo("Switching to bravo teleop controller")
                
                rospy.ServiceProxy('/girona500/bravo/controller_manager/switch_controller', 
                                   SwitchController)(['joint_teleop_velocity_controller', 'gripper_velocity_controller'], ['joint_velocity_controller', 'joint_trajectory_controller', 'gripper_position_controller'],
                                                      1, True, 0.0)
                self.mode = 1 #bravoarm
            elif joy.buttons[self.BUTTON_A] == 1.0: # AUV
                rospy.loginfo("Switching to AUV control")
                self.mode = 0

        if joy.buttons[self.BUTTON_RIGHT] == 1:
            if (joy.axes[self.CROSS_HORIZONTAL] == self.MOVE_RIGHT):
                rospy.loginfo("Switching to bravo ik controller")
                rospy.ServiceProxy('/girona500/bravo/controller_manager/switch_controller', 
                                   SwitchController)(['joint_velocity_controller','gripper_position_controller'], ['joint_teleop_velocity_controller','joint_trajectory_controller', 'gripper_velocity_controller'],
                                                      1, True, 0.0)

        if old_mode != self.mode: #Mode changed: send zeros once
            if old_mode == 1: #leftarm
                self.pub_bravoarm_desired_joint_velocity.publish(self.bravoarm_joint_cmd)
                self.pub_bravoarm_desired_gripper_state.publish(self.bravoarm_gripper_cmd)
                

        if self.mode == 0: #AUV mode

            # enable/disable z control position
            if joy.buttons[self.BUTTON_RIGHT] == 0: # not pushed
                self.joy_msg.buttons[JoystickBase.BUTTON_POSE_Z] = joy.buttons[self.BUTTON_A]
                self.joy_msg.buttons[JoystickBase.BUTTON_TWIST_W] = joy.buttons[self.BUTTON_Y]
                if joy.buttons[self.BUTTON_A] == 1.0:
                    self.up_down = 0.0
                    rospy.loginfo("%s: Reset up_down counter", self.name)
                # enable/disable yaw control position
                self.joy_msg.buttons[JoystickBase.BUTTON_POSE_YAW] = joy.buttons[self.BUTTON_B]
                self.joy_msg.buttons[JoystickBase.BUTTON_TWIST_R] = joy.buttons[self.BUTTON_X]
                if joy.buttons[self.BUTTON_B] == 1.0:
                    self.left_right = 0.0
                    rospy.loginfo("%s: Reset left_right counter", self.name)

            # Transform discrete axis (cross) to two 'analog' axis to control
            # depth and yaw in position.

            # up-down (depth control pose)
            if (joy.axes[self.CROSS_VERTICAL] == self.MOVE_DOWN):
                self.up_down = self.up_down + 0.05
                if self.up_down > 1.0:
                    self.up_down = 1.0
            elif (joy.axes[self.CROSS_VERTICAL] == self.MOVE_UP):
                self.up_down = self.up_down - 0.05
                if self.up_down < -1.0:
                    self.up_down = -1.0

            # left-right (yaw control pose)
            if (joy.axes[self.CROSS_HORIZONTAL] == self.MOVE_RIGHT):
                self.left_right = self.left_right + 0.05
                if self.left_right > 1.0:
                    self.left_right = -1.0
            elif (joy.axes[self.CROSS_HORIZONTAL] == self.MOVE_LEFT):
                self.left_right = self.left_right - 0.05
                if self.left_right < -1.0:
                    self.left_right = 1.0

            self.joy_msg.axes[JoystickBase.AXIS_POSE_Z] = self.up_down
            self.joy_msg.axes[JoystickBase.AXIS_POSE_YAW] = self.left_right
            self.joy_msg.axes[JoystickBase.AXIS_TWIST_U] = joy.axes[self.RIGHT_JOY_VERTICAL]
            self.joy_msg.axes[JoystickBase.AXIS_TWIST_V] = -joy.axes[self.RIGHT_JOY_HORIZONTAL]
            self.joy_msg.axes[JoystickBase.AXIS_TWIST_W] = -joy.axes[self.LEFT_JOY_VERTICAL]
            self.joy_msg.axes[JoystickBase.AXIS_TWIST_R] = -joy.axes[self.LEFT_JOY_HORIZONTAL]

        elif self.mode == 1: #bravoarm mode
            # Set zero desired AUV velocity and last pose
            self.joy_msg.axes[JoystickBase.AXIS_POSE_Z] = self.up_down
            self.joy_msg.axes[JoystickBase.AXIS_POSE_YAW] = self.left_right
            self.joy_msg.axes[JoystickBase.AXIS_TWIST_U] = 0.0
            self.joy_msg.axes[JoystickBase.AXIS_TWIST_V] = 0.0
            self.joy_msg.axes[JoystickBase.AXIS_TWIST_W] = 0.0
            self.joy_msg.axes[JoystickBase.AXIS_TWIST_R] = 0.0
            
            if joy.buttons[self.BUTTON_LEFT] == 0:
                self.bravoarm_joint_cmd.data[0] = joy.axes[self.LEFT_JOY_HORIZONTAL]*0.2
                self.bravoarm_joint_cmd.data[1] = -joy.axes[self.LEFT_JOY_VERTICAL]*0.2
            else:
                self.bravoarm_joint_cmd.data[5] = joy.axes[self.LEFT_JOY_HORIZONTAL]*0.5
                self.bravoarm_joint_cmd.data[4] = joy.axes[self.LEFT_JOY_VERTICAL]*0.2
            
            self.bravoarm_joint_cmd.data[2] = -joy.axes[self.RIGHT_JOY_VERTICAL]*0.2
            self.bravoarm_joint_cmd.data[3] = -joy.axes[self.RIGHT_JOY_HORIZONTAL]*0.2

            ## Open/Close gripper
            gripper_velocity = 0.004
            if joy.axes[self.RIGHT_TRIGGER] < 0.95:
                self.bravoarm_gripper_cmd = (2.0 -(joy.axes[self.RIGHT_TRIGGER] + 1.0))*gripper_velocity
            elif joy.axes[self.LEFT_TRIGGER] < 0.95:
                self.bravoarm_gripper_cmd = -(2.0 -(joy.axes[self.LEFT_TRIGGER] + 1.0))*gripper_velocity
            else:
                self.bravoarm_gripper_cmd = 0.0
        elif self.mode == 2:
            if joy.buttons[self.BUTTON_LEFT_JOY] == 1.0:
                rospy.loginfo("Calling /girona500/bravo/predefined_positions/do data= 'home'")
                rospy.ServiceProxy('/girona500/bravo/predefined_positions/do', String)(data='home')
            if joy.buttons[self.BUTTON_RIGHT_JOY] == 1.0:
                rospy.loginfo("Calling /girona500/bravo/predefined_positions/do data= 'look_down'")
                rospy.ServiceProxy('/girona500/bravo/predefined_positions/do', String)(data='look_down')

        self.mutual_exclusion.release()

    def iterate(self, event):
        """ This method is a callback of a timer. This is used to publish the
            output joy message """
        self.mutual_exclusion.acquire()
        # Publish message
        self.pub_map_ack_data.publish(self.joy_msg)
        if self.mode == 1: #leftarm
            self.pub_bravoarm_desired_joint_velocity.publish(self.bravoarm_joint_cmd)
            self.pub_bravoarm_desired_gripper_state.publish(self.bravoarm_gripper_cmd)

        self.mutual_exclusion.release()
        # Reset buttons

    def get_config(self):
        """ Read parameters from ROS Param Server """

        ns = rospy.get_namespace()

        param_dict = {'start_service': ('start_service', ''),
                      'stop_service': ('stop_service', '')
                     }

        param_loader.get_ros_params(self, param_dict)

if __name__ == '__main__':
    """ Initialize the logitech_fx10 node. """
    try:
        rospy.init_node('logitech_fx10_to_teleoperation')
        map_ack = LogitechFX10Atlantis(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass