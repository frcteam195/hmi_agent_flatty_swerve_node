"""
Class definition of the HMI agent node.
"""

from dataclasses import dataclass

import numpy as np
import rospy

from actions_node.ActionRunner import ActionRunner
from actions_node.game_specific_actions import AutomatedActions
from ck_ros_msgs_node.msg import HMI_Signals, Intake_Control, Led_Control, Arm_Goal
from nav_msgs.msg import Odometry

from ck_utilities_py_node.ckmath import *
from ck_utilities_py_node.geometry import *
from ck_utilities_py_node.joystick import Joystick
from ck_utilities_py_node.rosparam_helper import load_parameter_class
from frc_robot_utilities_py_node.frc_robot_utilities_py import robot_status, register_for_robot_updates, reset_robot_pose
from frc_robot_utilities_py_node.RobotStatusHelperPy import Alliance, BufferedROSMsgHandlerPy

from ck_ros_base_msgs_node.msg import Joystick_Status


@dataclass
class DriverParams:
    """
    Driver parameters. Must match the configuration YAML loaded.
    """
    drive_fwd_back_axis_id: int = -1
    drive_fwd_back_axis_inverted: bool = False

    drive_left_right_axis_id: int = -1
    drive_left_right_axis_inverted: bool = False

    drive_z_axis_id: int = -1
    drive_z_axis_inverted: bool = False

    drive_axis_deadband: float = 0.05
    drive_z_axis_deadband: float = 0.05

    reset_odometry_button_id: int = -1
    robot_orient_button_id: int = -1
    field_centric_button_id: int = -1


@dataclass
class OperatorParams:
    """
    Operator parameters. Must match the configuration YAML loaded.
    """
    outtake_axis_id: int = -1
    intake_axis_id: int = -1
    activation_threshold: float = 0

    ground_intake_button_id: int = -1
    hybrid_node_button_id: int = -1
    mid_node_button_id: int = -1
    high_node_button_id: int = -1
    in_bot_button_id: int = -1
    party_mode_button_id: int = -1
    operator_pinch_button_id: int = -1
    operator_unpinch_button_id: int = -1
    wrist_left_90_button_id: int = -1
    wrist_straight_button_id: int = -1
    wrist_left_180_button_id: int = -1

    led_control_pov_id: int = -1


@dataclass
class OperatorSplitParams:
    # Button Box
    home_button_id: int = -1
    shelf_button_id: int = -1
    low_button_id: int = -1

    high_cone_button_id: int = -1
    mid_cone_button_id: int = -1
    pickup_cone_button_id: int = -1
    pickup_dead_cone_button_id:  int = -1

    high_cube_button_id: int = -1
    mid_cube_button_id: int = -1
    pickup_cube_button_id: int = -1

    # Joystick
    intake_in_button_id: int = -1
    intake_out_button_id: int = -1

    intake_close_button_id: int = -1
    intake_open_button_id: int = -1

    joy_pickup_cube_button_id: int = -1
    joy_pickup_dead_cone_button_id: int = -1
    
    led_control_pov_id: int = -1


class HmiAgentNode():
    """
    The HMI agent node.
    """

    def __init__(self) -> None:
        register_for_robot_updates()

        self.action_runner = ActionRunner()

        self.driver_joystick = Joystick(0)
        # self.operator_controller = Joystick(1)

        self.operator_button_box = Joystick(1)
        self.operator_joystick = Joystick(2)

        self.driver_params = DriverParams()
        # self.operator_params = OperatorParams()
        self.operator_params = OperatorSplitParams()

        load_parameter_class(self.driver_params)
        load_parameter_class(self.operator_params)

        self.drivetrain_orientation = HMI_Signals.FIELD_CENTRIC

        self.led_control_message = Led_Control()
        self.led_timer = 0
        self.party_time = False

        self.heading = 0.0

        self.pinch_active = False

        self.hmi_publisher = rospy.Publisher(name="/HMISignals", data_class=HMI_Signals, queue_size=10, tcp_nodelay=True)
        self.odometry_publisher = rospy.Publisher(name="/ResetHeading", data_class=Odometry, queue_size=10, tcp_nodelay=True)
        self.intake_publisher = rospy.Publisher(name="/IntakeControl", data_class=Intake_Control, queue_size=10, tcp_nodelay=True)
        self.led_control_publisher = rospy.Publisher(name="/LedControl", data_class=Led_Control, queue_size=10, tcp_nodelay=True)

        self.arm_goal_publisher = rospy.Publisher(name="/ArmGoal", data_class=Arm_Goal, queue_size=10, tcp_nodelay=True)
        self.arm_goal = Arm_Goal()
        self.arm_goal.goal = Arm_Goal.HOME

        self.odometry_subscriber = BufferedROSMsgHandlerPy(Odometry)
        self.odometry_subscriber.register_for_updates("odometry/filtered")

        rospy.Subscriber(name="/JoystickStatus", data_class=Joystick_Status, callback=self.joystick_callback, queue_size=1, tcp_nodelay=True)
        rospy.spin()

    def joystick_callback(self, message: Joystick_Status):
        """
        Joystick callback function. This runs everytime a new joystick status message is received.
        """
        Joystick.update(message)

        hmi_update_message = HMI_Signals()
        hmi_update_message.drivetrain_brake = True

        #######################################################################
        ###                         DRIVER CONTROLS                         ###
        #######################################################################
        invert_axis_fwd_back = -1 if self.driver_params.drive_fwd_back_axis_inverted else 1
        invert_axis_left_right = -1 if self.driver_params.drive_left_right_axis_inverted else 1

        fwd_back_value = self.driver_joystick.getFilteredAxis(self.driver_params.drive_fwd_back_axis_id, self.driver_params.drive_axis_deadband)
        hmi_update_message.drivetrain_fwd_back = invert_axis_fwd_back * fwd_back_value

        left_right_value = self.driver_joystick.getFilteredAxis(self.driver_params.drive_left_right_axis_id, self.driver_params.drive_axis_deadband)
        hmi_update_message.drivetrain_left_right = invert_axis_left_right * left_right_value

        x = hmi_update_message.drivetrain_fwd_back
        y = hmi_update_message.drivetrain_left_right

        invert_axis_z = -1 if self.driver_params.drive_z_axis_inverted else 1
        z = invert_axis_z * self.driver_joystick.getFilteredAxis(self.driver_params.drive_z_axis_id, self.driver_params.drive_z_axis_deadband)

        r = hypotenuse(x, y)
        theta = polar_angle_rad(x, y)

        z = np.sign(z) * pow(z, 2)
        active_theta = theta
        if r > self.driver_params.drive_axis_deadband:
            active_theta = theta

        hmi_update_message.drivetrain_swerve_direction = active_theta
        hmi_update_message.drivetrain_swerve_percent_fwd_vel = limit(r, 0.0, 1.0)
        hmi_update_message.drivetrain_swerve_percent_angular_rot = z

        if self.driver_joystick.getButton(self.driver_params.robot_orient_button_id):
            self.drivetrain_orientation = HMI_Signals.ROBOT_ORIENTED
        elif self.driver_joystick.getButton(self.driver_params.field_centric_button_id):
            self.drivetrain_orientation = HMI_Signals.FIELD_CENTRIC

        hmi_update_message.drivetrain_orientation = self.drivetrain_orientation

        if self.driver_joystick.getButton(self.driver_params.reset_odometry_button_id):
            reset_robot_pose()

        #######################################################################
        ###                        OPERATOR CONTROLS                        ###
        #######################################################################
        self.process_intake_control()
        self.process_leds()

        # Determine the alliance station the robot is facing.
        if self.odometry_subscriber.get() is not None:
            odometry_message = self.odometry_subscriber.get()
            rotation = Rotation(odometry_message.pose.pose.orientation)
            yaw = rotation.yaw
            yaw = normalize_to_2_pi(yaw)
            self.heading = np.degrees(yaw)

        target_alliance = Alliance.RED if 90 < self.heading < 270 else Alliance.BLUE
        hmi_update_message.drivetrain_heading = self.heading

        ################################################################################
        ###                         CONTROL MAPPINGS                                 ###
        ################################################################################

        arm_action = None
        reverse_arm = target_alliance != robot_status.get_alliance()

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.home_button_id):
            self.arm_goal.goal = Arm_Goal.HOME

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.shelf_button_id):
            self.arm_goal.goal = Arm_Goal.SHELF_PICKUP_FRONT

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.high_cone_button_id):
            self.arm_goal.goal = Arm_Goal.HIGH_CONE_FRONT

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.mid_cone_button_id):
            self.arm_goal.goal = Arm_Goal.MID_CONE_FRONT

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.pickup_cone_button_id):
            self.arm_goal.goal = Arm_Goal.GROUND_CONE_FRONT

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.pickup_dead_cone_button_id) \
                or self.operator_joystick.getRisingEdgeButton(self.operator_params.joy_pickup_dead_cone_button_id):
            self.arm_goal.goal = Arm_Goal.GROUND_DEAD_CONE_FRONT

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.high_cube_button_id):
            self.arm_goal.goal = Arm_Goal.HIGH_CUBE_FRONT

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.mid_cube_button_id):
            self.arm_goal.goal = Arm_Goal.MID_CUBE_FRONT
        
        if self.operator_button_box.getRisingEdgeButton(self.operator_params.pickup_cube_button_id) \
                or self.operator_joystick.getRisingEdgeButton(self.operator_params.joy_pickup_cube_button_id):
            self.arm_goal.goal = Arm_Goal.GROUND_CUBE_FRONT

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.low_button_id):
            self.arm_goal.goal = Arm_Goal.LOW_SCORE_FRONT

        self.arm_goal_publisher.publish(self.arm_goal)

        # if self.operator_controller.getRisingEdgeButton(self.operator_params.high_node_button_id):
        #     if self.pinch_active:
        #         arm_action = AutomatedActions.HighConeAction(reverse_arm)
        #     else:
        #         arm_action = AutomatedActions.HighCubeAction(reverse_arm)

        # if self.operator_controller.getRisingEdgeButton(self.operator_params.mid_node_button_id):
        #     if self.pinch_active:
        #         arm_action = AutomatedActions.MidConeAction(reverse_arm)
        #     else:
        #         arm_action = AutomatedActions.MidCubeAction(reverse_arm)

        # if self.operator_controller.getRisingEdgeButton(self.operator_params.hybrid_node_button_id):
        #     arm_action = AutomatedActions.HybridAction(reverse_arm)

        # if self.operator_controller.getRisingEdgeButton(self.operator_params.in_bot_button_id):
        #     arm_action = AutomatedActions.InRobotAction()

        # if self.operator_controller.getRisingEdgeButton(self.operator_params.wrist_left_90_button_id):
        #     arm_action = AutomatedActions.WristLeft90()

        # if self.operator_controller.getRisingEdgeButton(self.operator_params.wrist_straight_button_id):
        #     arm_action = AutomatedActions.WristStraight()

        # if self.operator_controller.getRisingEdgeButton(self.operator_params.wrist_left_180_button_id):
        #     arm_action = AutomatedActions.WristLeft180()

        # if arm_action is not None:
        #     self.action_runner.start_action(arm_action)

        self.hmi_publisher.publish(hmi_update_message)
        self.action_runner.loop(robot_status.get_mode())

    def process_intake_control(self):
        """
        Handles all intake control.
        """
        intake_control = Intake_Control()
        intake_action = None

        if self.operator_joystick.getButton(self.operator_params.intake_open_button_id):
            self.pinch_active = True
        elif self.operator_joystick.getButton(self.operator_params.intake_close_button_id):
            self.pinch_active = False

        intake_control.pincher_solenoid_on = self.pinch_active

        if self.operator_joystick.getButton(self.operator_params.intake_in_button_id):
            intake_control.rollers_intake = True
            intake_control.rollers_outtake = False
        elif self.operator_joystick.getButton(self.operator_params.intake_out_button_id):
            intake_control.rollers_intake = False
            intake_control.rollers_outtake = True

        if intake_action is not None:
            self.action_runner.start_action(intake_action)

        self.intake_publisher.publish(intake_control)

    def process_leds(self):
        """
        Handles all the LED changes.
        """
        self.led_control_message.control_mode = Led_Control.ANIMATE
        self.led_control_message.number_leds = 8

        if not robot_status.is_connected():
            self.led_control_message.animation = Led_Control.STROBE
            self.led_control_message.speed = 0.3
            self.led_control_message.brightness = 0.5
            self.led_control_message.red = 255
            self.led_control_message.green = 0
            self.led_control_message.blue = 0

        else:
            if self.operator_joystick.getPOV(self.operator_params.led_control_pov_id) == 270:
                self.led_timer = rospy.get_time()
                self.led_control_message.animation = Led_Control.STROBE
                self.led_control_message.speed = 0.1
                self.led_control_message.brightness = 0.5
                self.led_control_message.red = 255
                self.led_control_message.green = 255
                self.led_control_message.blue = 0

            if self.operator_joystick.getPOV(self.operator_params.led_control_pov_id) == 90:
                self.led_timer = rospy.get_time()
                self.led_control_message.animation = Led_Control.STROBE
                self.led_control_message.speed = 0.1
                self.led_control_message.brightness = 0.5
                self.led_control_message.red = 255
                self.led_control_message.green = 0
                self.led_control_message.blue = 255

            # if self.operator_joystick.getRisingEdgeButton(self.operator_params.party_mode_button_id):
            #     self.party_time = not self.party_time

            if rospy.get_time() - self.led_timer > 3:
                if not self.party_time:
                    self.led_control_message.animation = Led_Control.LARSON
                    self.led_control_message.speed = 0.5
                    self.led_control_message.brightness = 0.5
                    self.led_control_message.red = 0
                    self.led_control_message.green = 255
                    self.led_control_message.blue = 0

                else:
                    self.led_control_message.animation = Led_Control.RAINBOW
                    self.led_control_message.speed = 1
                    self.led_control_message.brightness = 1

        self.led_control_publisher.publish(self.led_control_message)
