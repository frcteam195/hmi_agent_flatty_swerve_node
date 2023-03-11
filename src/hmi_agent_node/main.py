"""
Class definition of the HMI agent node.
"""

from dataclasses import dataclass

import numpy as np
import rospy
import typing

from actions_node.ActionRunner import ActionRunner
from actions_node.game_specific_actions import AutomatedActions
from ck_ros_msgs_node.msg import HMI_Signals, Intake_Control, Led_Control, Arm_Goal, Arm_Status
from nav_msgs.msg import Odometry

from ck_utilities_py_node.ckmath import *
from ck_utilities_py_node.geometry import *
from ck_utilities_py_node.joystick import Joystick
from ck_utilities_py_node.rosparam_helper import load_parameter_class
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import Alliance, BufferedROSMsgHandlerPy

from actions_node.game_specific_actions.PlaceHighConeAction import PlaceHighConeAction
from ck_ros_base_msgs_node.msg import Joystick_Status
from actions_node.game_specific_actions.Subsystem import Subsystem
from ck_utilities_py_node.pid_controller import PIDController
# import cProfile

num_leds = 50
color_purple = Led_Control(0, 0, 92, 6, 140, 0, 1, 0, num_leds)
color_yellow = Led_Control(0, 0, 255, 255, 0, 0, 1, 0, num_leds)

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
    drive_z_axis_min_value_after_deadband : float = 0

    reset_odometry_button_id: int = -1
    robot_align_to_grid: int = -1
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

    led_toggle_id: int = -1

    # Joystick
    intake_in_button_id: int = -1
    intake_out_button_id: int = -1

    intake_close_button_id: int = -1
    intake_open_button_id: int = -1

    pre_score_position_button_id: int = -1

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

        self.led_control_message = color_purple
        self.led_timer = 0
        self.party_time = False
        self.curr_color = False

        self.heading = 0.0

        self.pinch_active = True

        self.hmi_publisher = rospy.Publisher(name="/HMISignals", data_class=HMI_Signals, queue_size=10, tcp_nodelay=True)
        self.intake_publisher = rospy.Publisher(name="/IntakeControl", data_class=Intake_Control, queue_size=10, tcp_nodelay=True)
        self.led_control_publisher = rospy.Publisher(name="/LedControl", data_class=Led_Control, queue_size=10, tcp_nodelay=True)

        self.arm_goal_publisher = rospy.Publisher(name="/ArmGoal", data_class=Arm_Goal, queue_size=10, tcp_nodelay=True)
        self.arm_goal = Arm_Goal()
        self.arm_goal.goal = Arm_Goal.HOME
        self.arm_goal.wrist_goal = Arm_Goal.WRIST_ZERO

        self.odometry_subscriber = BufferedROSMsgHandlerPy(Odometry)
        self.odometry_subscriber.register_for_updates("odometry/filtered")

        self.arm_subscriber = BufferedROSMsgHandlerPy(Arm_Status)
        self.arm_subscriber.register_for_updates("/ArmStatus")

        self.orientation_helper = PIDController(kP=0.0047, kD=0.001, filter_r=0.6)

        rospy.Subscriber(name="/JoystickStatus", data_class=Joystick_Status, callback=self.joystick_callback, queue_size=1, tcp_nodelay=True)
        # profiler = cProfile.Profile()
        # profiler.enable()
        rospy.spin()
        # profiler.disable()
        # profiler.dump_stats("/mnt/working/hmi_agent_node.stats")


    def joystick_callback(self, message: Joystick_Status):
        """
        Joystick callback function. This runs everytime a new joystick status message is received.
        """

        #DO NOT REMOVE THIS CHECK!!!!!!!!!! DID YOU LEARN NOTHING FROM 2022?!
        if robot_status.get_mode() != RobotMode.TELEOP:
            self.process_leds()
            return

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
        z = invert_axis_z * self.driver_joystick.getFilteredAxis(self.driver_params.drive_z_axis_id, self.driver_params.drive_z_axis_deadband, self.driver_params.drive_z_axis_min_value_after_deadband)

        r = hypotenuse(x, y)
        theta = polar_angle_rad(x, y)

        z = np.sign(z) * pow(z, 2)
        active_theta = theta
        if r > self.driver_params.drive_axis_deadband:
            active_theta = theta

        hmi_update_message.drivetrain_swerve_direction = active_theta

        # Scale the drive power based on current arm position.
        arm_status_message = self.arm_subscriber.get()
        limited_forward_velocity, limited_angular_rotation = limit_drive_power(arm_status_message, r, z)

        hmi_update_message.drivetrain_swerve_percent_fwd_vel = limited_forward_velocity
        hmi_update_message.drivetrain_swerve_percent_angular_rot = limited_angular_rotation

        # Swap between field centric and robot oriented drive.
        if self.driver_joystick.getButton(self.driver_params.robot_orient_button_id):
            self.drivetrain_orientation = HMI_Signals.ROBOT_ORIENTED
        elif self.driver_joystick.getButton(self.driver_params.field_centric_button_id):
            self.drivetrain_orientation = HMI_Signals.FIELD_CENTRIC

        hmi_update_message.drivetrain_orientation = self.drivetrain_orientation

        if self.driver_joystick.getRisingEdgeButton(self.driver_params.reset_odometry_button_id):
            reset_robot_pose(robot_status.get_alliance())

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

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.home_button_id):
            self.arm_goal.goal = Arm_Goal.HOME

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.shelf_button_id):
            self.arm_goal.goal = Arm_Goal.SHELF_PICKUP

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.high_cone_button_id):
            self.arm_goal.goal = Arm_Goal.HIGH_CONE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.mid_cone_button_id):
            self.arm_goal.goal = Arm_Goal.MID_CONE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.pickup_cone_button_id):
            self.arm_goal.goal = Arm_Goal.GROUND_CONE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.pickup_dead_cone_button_id):
            self.arm_goal.goal = Arm_Goal.GROUND_DEAD_CONE

        if self.operator_joystick.getRisingEdgeButton(self.operator_params.pre_score_position_button_id):
            self.arm_goal.goal = Arm_Goal.PRE_SCORE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.high_cube_button_id):
            self.arm_goal.goal = Arm_Goal.HIGH_CUBE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.mid_cube_button_id):
            self.arm_goal.goal = Arm_Goal.MID_CUBE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.pickup_cube_button_id):
            self.arm_goal.goal = Arm_Goal.GROUND_CUBE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.low_button_id):
            self.arm_goal.goal = Arm_Goal.LOW_SCORE

        # TODO: Put this in the params
        if self.operator_button_box.getRisingEdgeButton(11):
            self.arm_goal.goal = Arm_Goal.PRE_DEAD_CONE

        pov_status, pov_dir = self.operator_joystick.getRisingEdgePOV(0)

        if pov_status:
            if pov_dir == 0:
                self.arm_goal.wrist_goal = Arm_Goal.WRIST_ZERO

            if pov_dir == 90:
                if self.arm_goal.wrist_goal == Arm_Goal.WRIST_180:
                    self.arm_goal.wrist_goal = Arm_Goal.WRIST_ZERO
                elif self.arm_goal.wrist_goal == Arm_Goal.WRIST_ZERO:
                    self.arm_goal.wrist_goal = Arm_Goal.WRIST_180

            if pov_dir == 180:
                self.arm_goal.wrist_goal = Arm_Goal.WRIST_180

            if pov_dir == 270:
                self.arm_goal.wrist_goal = Arm_Goal.WRIST_90

        reverse_arm = target_alliance != robot_status.get_alliance()

        if self.driver_joystick.getButton(self.driver_params.robot_align_to_grid): # or \
            # self.arm_goal.goal == Arm_Goal.PRE_SCORE: or \
            # self.arm_goal.goal == Arm_Goal.SHELF_PICKUP:
            #Do odometry align to grid
            odom_msg : Odometry = self.odometry_subscriber.get()
            alliance : Alliance = robot_status.get_alliance()
            if odom_msg is not None and alliance is not None:
                desired_heading : float = 0

                if reverse_arm:

                    if alliance == Alliance.RED:
                        desired_heading = 0
                    elif alliance == Alliance.BLUE:
                        desired_heading = 180
                else:
                    if alliance == Alliance.RED:
                        desired_heading = 180
                    elif alliance == Alliance.BLUE:
                        desired_heading = 0

                curr_pose = Pose(odom_msg.pose.pose)
                actual_heading = math.degrees(curr_pose.orientation.yaw)
                hmi_update_message.desired_heading = desired_heading
                hmi_update_message.actual_heading = actual_heading
                error = wrapMinMax(desired_heading - actual_heading, -180, 180)
                hmi_update_message.error = error
                output_val = limit(self.orientation_helper.update_by_error(error), -0.6, 0.6)
                hmi_update_message.initial_output_val = output_val
                output_val = normalizeWithDeadband(output_val, 3 * self.orientation_helper.kP, 0.08)
                hmi_update_message.second_output_val = output_val
                hmi_update_message.drivetrain_swerve_percent_angular_rot = output_val

        # arm should point away from our driver stattion for shelf pickup
        if self.arm_goal.goal is Arm_Goal.SHELF_PICKUP:
            reverse_arm = not reverse_arm

        if reverse_arm:
            # Robot is facing our driver station
            self.arm_goal.goal_side = Arm_Goal.SIDE_BACK
        else:
            self.arm_goal.goal_side = Arm_Goal.SIDE_FRONT

        if self.arm_goal.goal in (Arm_Goal.GROUND_CONE, Arm_Goal.GROUND_CUBE, Arm_Goal.GROUND_DEAD_CONE, Arm_Goal.PRE_DEAD_CONE):
            self.arm_goal.goal_side = Arm_Goal.SIDE_FRONT if not self.operator_joystick.getButton(3) else Arm_Goal.SIDE_BACK

        self.arm_goal_publisher.publish(self.arm_goal)

        self.hmi_publisher.publish(hmi_update_message)
        self.action_runner.loop(robot_status.get_mode())

    def process_intake_control(self):
        """
        Handles all intake control.
        """
        intake_control = Intake_Control()
        intake_action = None

        if Subsystem.INTAKE in self.action_runner.get_operated_systems():
            intake_control = None

        if self.operator_joystick.getButton(self.operator_params.intake_close_button_id):
            self.pinch_active = True
        elif self.operator_joystick.getButton(self.operator_params.intake_open_button_id):
            self.pinch_active = False
        elif self.arm_goal.goal in (Arm_Goal.GROUND_CUBE, Arm_Goal.PRE_DEAD_CONE):
            self.pinch_active = False
        elif self.arm_goal.goal in (Arm_Goal.GROUND_CONE, Arm_Goal.GROUND_DEAD_CONE):
            self.pinch_active = True

        if self.operator_joystick.getButton(self.operator_params.intake_in_button_id):
            intake_control.rollers_intake = True
            intake_control.rollers_outtake = False
        elif self.operator_joystick.getButton(self.operator_params.intake_out_button_id):
            intake_control.rollers_intake = False
            intake_control.rollers_outtake = True
            if self.arm_goal.goal == Arm_Goal.HIGH_CUBE:
                intake_control.speed = 0.3
            else:
                intake_control.speed = 0

        if intake_control is not None:
            intake_control.pinched = self.pinch_active

        self.intake_publisher.publish(intake_control)

        if intake_action is not None:
            self.action_runner.start_action(intake_action)

    def process_leds(self):
        """
        Handles all the LED changes.
        """

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.led_toggle_id):
            self.curr_color = not self.curr_color
            if self.curr_color:
                self.led_control_message = color_yellow
            else:
                self.led_control_message = color_purple
                

        # self.led_control_message.control_mode = Led_Control.ANIMATE
        # self.led_control_message.number_leds = 58

        # if not robot_status.is_connected():
        #     self.led_control_message.animation = Led_Control.STROBE
        #     self.led_control_message.speed = 0.3
        #     self.led_control_message.brightness = 0.5
        #     self.led_control_message.red = 255
        #     self.led_control_message.green = 0
        #     self.led_control_message.blue = 0

        # else:
        #     if self.operator_joystick.getPOV(self.operator_params.led_control_pov_id) == 270:
        #         self.led_timer = rospy.get_time()
        #         self.led_control_message.animation = Led_Control.STROBE
        #         self.led_control_message.speed = 0.1
        #         self.led_control_message.brightness = 0.5
        #         self.led_control_message.red = 255
        #         self.led_control_message.green = 255
        #         self.led_control_message.blue = 0

        #     if self.operator_joystick.getPOV(self.operator_params.led_control_pov_id) == 90:
        #         self.led_timer = rospy.get_time()
        #         self.led_control_message.animation = Led_Control.STROBE
        #         self.led_control_message.speed = 0.1
        #         self.led_control_message.brightness = 0.5
        #         self.led_control_message.red = 255
        #         self.led_control_message.green = 0
        #         self.led_control_message.blue = 255

        #     # if self.operator_joystick.getRisingEdgeButton(self.operator_params.party_mode_button_id):
        #     #     self.party_time = not self.party_time

        #     if rospy.get_time() - self.led_timer > 3:
        #         if not self.party_time:
        #             self.led_control_message.animation = Led_Control.LARSON
        #             self.led_control_message.speed = 0.5
        #             self.led_control_message.brightness = 0.5
        #             self.led_control_message.red = 0
        #             self.led_control_message.green = 255
        #             self.led_control_message.blue = 0

        #         else:
        #             self.led_control_message.animation = Led_Control.RAINBOW
        #             self.led_control_message.speed = 1
        #             self.led_control_message.brightness = 1

        self.led_control_publisher.publish(self.led_control_message)


def limit_drive_power(arm_status: Arm_Status, forward_velocity: float, angular_rotation: float) -> typing.Tuple[float, float]:
    """
    Limit the drive power depending on the current arm position.
    """
    forward_limit = 1.0
    angular_limit = 1.0

    if arm_status is not None:
        overall_arm_angle = abs(arm_status.arm_base_angle + arm_status.arm_upper_angle)
        overall_arm_angle = limit(overall_arm_angle, 0.0, 150)

        forward_limit = -0.006666667 * overall_arm_angle + 1.2
        angular_limit = -0.006666667 * overall_arm_angle + 1.2

        if arm_status.extended:
            forward_limit -= 0.1
            angular_limit -= 0.1

    forward_limit = limit(forward_limit, 0.0, 1.0)
    angular_limit = limit(angular_limit, 0.0, 1.0)

    return limit(forward_velocity, -forward_limit, forward_limit), limit(angular_rotation, -angular_limit, angular_limit)
