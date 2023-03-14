"""
Class definition of the HMI agent node.
"""

from dataclasses import dataclass

import numpy as np
import rospy
import typing

from ck_ros_msgs_node.msg import HMI_Signals, Intake_Control, Led_Control, Arm_Goal, Arm_Status
from nav_msgs.msg import Odometry

from ck_utilities_py_node.ckmath import *
from ck_utilities_py_node.geometry import *
from ck_utilities_py_node.joystick import Joystick
from ck_utilities_py_node.rosparam_helper import load_parameter_class
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import Alliance, BufferedROSMsgHandlerPy

from ck_ros_base_msgs_node.msg import Joystick_Status
from ck_utilities_py_node.pid_controller import PIDController
# import cProfile

NUM_LEDS = 50
solid_purple = Led_Control(Led_Control.SET_LED, 0, 57, 3, 87, 0, 1, 0, NUM_LEDS, "")
solid_yellow = Led_Control(Led_Control.SET_LED, 0, 255, 255, 0, 0, 1, 0, NUM_LEDS, "")
strobe_purple = Led_Control(Led_Control.ANIMATE, Led_Control.STROBE, 57, 3, 87, 0, 0, 0.25, NUM_LEDS, "")
strobe_red = Led_Control(Led_Control.ANIMATE, Led_Control.STROBE, 96, 0, 0, 0, 0, 0.10, NUM_LEDS, "")
larson_purple = Led_Control(Led_Control.ANIMATE, Led_Control.LARSON, 57, 3, 87, 0, 0, 0.20, NUM_LEDS, "")
fire_animation = Led_Control(Led_Control.ANIMATE, Led_Control.FIRE, 0, 0, 0, 0, 0.5, 0.5, NUM_LEDS, "")
rainbow = Led_Control(Led_Control.ANIMATE, Led_Control.RAINBOW, 0, 0, 0, 0, 0.5, 0.60, NUM_LEDS, "")
twinkle_purple = Led_Control(Led_Control.ANIMATE, Led_Control.TWINKLE_OFF, 57, 3, 87, 0, 0, 0.25, NUM_LEDS, "")

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


class HmiAgentNode():
    """
    The HMI agent node.
    """

    def __init__(self) -> None:
        register_for_robot_updates()

        self.driver_joystick = Joystick(0)

        self.driver_params = DriverParams()

        load_parameter_class(self.driver_params)

        self.drivetrain_orientation = HMI_Signals.FIELD_CENTRIC

        self.led_control_message = solid_purple
        self.led_timer = 0
        self.party_time = False

        self.current_color = solid_purple

        self.heading = 0.0

        self.pinch_active = True

        self.hmi_publisher = rospy.Publisher(name="/HMISignals", data_class=HMI_Signals, queue_size=10, tcp_nodelay=True)
        self.intake_publisher = rospy.Publisher(name="/IntakeControl", data_class=Intake_Control, queue_size=10, tcp_nodelay=True)
        self.led_control_publisher = rospy.Publisher(name="/LedControl", data_class=Led_Control, queue_size=10, tcp_nodelay=True)


        self.odometry_subscriber = BufferedROSMsgHandlerPy(Odometry)
        self.odometry_subscriber.register_for_updates("odometry/filtered")

        rospy.Subscriber(name="/JoystickStatus", data_class=Joystick_Status, callback=self.joystick_callback, queue_size=1, tcp_nodelay=True)
        rospy.spin()


    def joystick_callback(self, message: Joystick_Status):
        """
        Joystick callback function. This runs everytime a new joystick status message is received.
        """

        #DO NOT REMOVE THIS CHECK!!!!!!!!!! DID YOU LEARN NOTHING FROM 2022?!
        if robot_status.get_mode() != RobotMode.TELEOP:
            # self.process_leds()
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

        hmi_update_message.drivetrain_swerve_percent_fwd_vel = r
        hmi_update_message.drivetrain_swerve_percent_angular_rot = z

        # Swap between field centric and robot oriented drive.
        if self.driver_joystick.getButton(self.driver_params.robot_orient_button_id):
            self.drivetrain_orientation = HMI_Signals.ROBOT_ORIENTED
        elif self.driver_joystick.getButton(self.driver_params.field_centric_button_id):
            self.drivetrain_orientation = HMI_Signals.FIELD_CENTRIC

        hmi_update_message.drivetrain_orientation = self.drivetrain_orientation

        if self.driver_joystick.getRisingEdgeButton(self.driver_params.reset_odometry_button_id):
            reset_robot_pose(robot_status.get_alliance())

        self.hmi_publisher.publish(hmi_update_message)

        #######################################################################
        ###                        OPERATOR CONTROLS                        ###
        #######################################################################
        # self.process_leds()


    # def process_leds(self):
    #     """
    #     Handles all the LED changes.
    #     """

    #     if not robot_status.is_connected():
    #         self.led_control_message = strobe_red
    #     elif robot_status.get_mode() != RobotMode.TELEOP:
    #         self.led_control_message = larson_purple
    #     else:
    #         # if self.operator_button_box.getRisingEdgeButton(self.operator_params.led_toggle_id):
    #         #     self.current_color = solid_yellow if self.current_color == solid_purple else solid_purple
    #         self.led_control_message = self.current_color

    #     self.led_control_publisher.publish(self.led_control_message)
