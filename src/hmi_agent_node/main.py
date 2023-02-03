#!/usr/bin/env python3

import rospy
from dataclasses import dataclass
from ck_ros_base_msgs_node.msg import Joystick_Status, Robot_Status
from ck_ros_msgs_node.msg import HMI_Signals
from ck_utilities_py_node.joystick import Joystick
from ck_utilities_py_node.ckmath import *
from ck_utilities_py_node.geometry import *
from ck_ros_msgs_node.msg import Intake_Control, Led_Control
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from nav_msgs.msg import *
from actions_node.game_specific_actions import HighConeAction, HighCubeAction, MidConeAction, MidCubeAction, HybridAction, GroundAction, InBotAction
from actions_node.ActionRunner import ActionRunner
import numpy as np
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode, BufferedROSMsgHandlerPy
from hmi_agent_node.reset_odom_msg import get_reset_odom_msg
from ck_utilities_py_node.rosparam_helper import load_parameter_class

@dataclass
class DriveParams:
    drive_fwd_back_axis_id: int = -1
    drive_fwd_back_axis_inverted: bool = False

    drive_left_right_axis_id: int = -1
    drive_left_right_axis_inverted: bool = False

    drive_z_axis_id: int = -1
    drive_z_axis_inverted: bool = False

    drive_axis_deadband: float = 0.05
    drive_z_axis_deadband: float = 0.05

    driver_unpinch_button_id: int = -1
    driver_pinch_button_id: int = -1
    driver_intake_button_id: int = -1
    driver_outtake_button_id: int = -1

@dataclass
class OperatorParams:
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

    led_control_pov_id: int = -1



action_runner = ActionRunner()

drive_params = DriveParams()
operator_params = OperatorParams()

hmi_pub = None
odom_pub = None
odometry_subscriber = None
intake_pub = None
led_control_pub = None

led_timer = 0
party_time = True
led_control_msg = Led_Control()

pinch_active = False
recent_heading = 0

drive_joystick = Joystick(0)
operator_controller = Joystick(1)

drivetrain_orientation = HMI_Signals.ROBOT_ORIENTED

def process_leds():
    global led_control_msg
    global led_control_pub
    global led_timer
    global operator_controller
    global operator_params
    global party_time
    global robot_status
 

    led_control_msg.control_mode = Led_Control.ANIMATE
    led_control_msg.number_leds = 8
    
    if not robot_status.is_connected():
        led_control_msg.animation = Led_Control.STROBE
        led_control_msg.speed = 0.3
        led_control_msg.brightness = 0.5
        led_control_msg.red = 255
        led_control_msg.green = 0
        led_control_msg.blue = 0

    else:
        if operator_controller.getPOV(operator_params.led_control_pov_id) == 270:
            led_timer = rospy.get_time()
            led_control_msg.animation = Led_Control.STROBE
            led_control_msg.speed = 0.1
            led_control_msg.brightness = 0.5
            led_control_msg.red = 255
            led_control_msg.green = 255
            led_control_msg.blue = 0

        if operator_controller.getPOV(operator_params.led_control_pov_id) == 90:
            led_timer = rospy.get_time()
            led_control_msg.animation = Led_Control.STROBE
            led_control_msg.speed = 0.1
            led_control_msg.brightness = 0.5
            led_control_msg.red = 255
            led_control_msg.green = 0
            led_control_msg.blue = 255
        
        if operator_controller.getRisingEdgeButton(operator_params.party_mode_button_id):
            party_time = not party_time
            
        if rospy.get_time() - led_timer > 3:
            if not party_time:
                led_control_msg.animation = Led_Control.LARSON
                led_control_msg.speed = 0.5
                led_control_msg.brightness = 0.5
                led_control_msg.red = 0
                led_control_msg.green = 255
                led_control_msg.blue = 0
        
            else:
                led_control_msg.animation = Led_Control.RAINBOW
                led_control_msg.speed = 1
                led_control_msg.brightness = 1

    led_control_pub.publish(led_control_msg)

def process_intake_control():
    global drive_joystick
    global drive_params
    global operator_controller
    global operator_params
    global pinch_active
    global action_runner

    intake_control = Intake_Control()

    if drive_joystick.getButton(drive_params.driver_unpinch_button_id) or operator_controller.getButton(operator_params.operator_unpinch_button_id):
        pinch_active = False
    elif drive_joystick.getButton(drive_params.driver_pinch_button_id) or operator_controller.getButton(operator_params.operator_pinch_button_id):
        pinch_active = True

    intake_control.pincher_solenoid_on = pinch_active

    if drive_joystick.getButton(drive_params.driver_intake_button_id) or operator_controller.getRawAxis(operator_params.intake_axis_id) > operator_params.activation_threshold:
        intake_control.rollers_intake = True
        intake_control.rollers_outtake = False
    elif drive_joystick.getButton(drive_params.driver_outtake_button_id) or operator_controller.getRawAxis(operator_params.outtake_axis_id) > operator_params.activation_threshold:
        intake_control.rollers_intake = False 
        intake_control.rollers_outtake = True 
    
    intake_pub.publish(intake_control)

def joystick_callback(msg: Joystick_Status):
    global drivetrain_orientation
    global hmi_pub
    global odometry_subscriber
    global recent_heading

    global robot_status

    global drive_joystick
    global operator_controller

    global drive_params
    global operator_params
    global action_runner

    global pinch_active
    
    Joystick.update(msg)

    hmi_update_msg = HMI_Signals()

    hmi_update_msg.drivetrain_brake = True

    invert_axis_fwd_back = -1 if drive_params.drive_fwd_back_axis_inverted else 1
    invert_axis_left_right = -1 if drive_params.drive_left_right_axis_inverted else 1

    hmi_update_msg.drivetrain_fwd_back = invert_axis_fwd_back * drive_joystick.getFilteredAxis(drive_params.drive_fwd_back_axis_id, drive_params.drive_axis_deadband)

    hmi_update_msg.drivetrain_left_right = invert_axis_left_right * drive_joystick.getFilteredAxis(drive_params.drive_left_right_axis_id, drive_params.drive_axis_deadband)

    x = hmi_update_msg.drivetrain_fwd_back
    y = hmi_update_msg.drivetrain_left_right

    invert_axis_z = -1 if drive_params.drive_z_axis_inverted else 1
    z = invert_axis_z * drive_joystick.getFilteredAxis(drive_params.drive_z_axis_id, drive_params.drive_z_axis_deadband)

    r = hypotenuse(x, y)
    theta = polar_angle_rad(x, y)

    z = np.sign(z) * pow(z, 2)
    active_theta = theta
    if (r > drive_params.drive_axis_deadband):
        active_theta = theta

    hmi_update_msg.drivetrain_swerve_direction = active_theta
    hmi_update_msg.drivetrain_swerve_percent_fwd_vel = limit(r, 0.0, 1.0)
    hmi_update_msg.drivetrain_swerve_percent_angular_rot = z

    hmi_update_msg.drivetrain_orientation = drivetrain_orientation

    process_leds()
    process_intake_control()

    if odometry_subscriber.get() is not None:
        odometry_msg = odometry_subscriber.get()
        rotation = Rotation(odometry_msg.pose.pose.orientation)
        yaw = rotation.yaw
        yaw = normalize_to_2_pi(yaw)
        recent_heading = math.degrees(yaw)

    facing_alliance = Alliance.RED if 90 < recent_heading < 270 else Alliance.BLUE


    ################################################################################
    ###                         CONTROL MAPPINGS                                 ###
    ################################################################################

    arm_action = None

    if operator_controller.getRisingEdgeButton(operator_params.high_node_button_id):
        if pinch_active:
            arm_action = HighConeAction(reversed=facing_alliance != robot_status.get_alliance())
        else:
            arm_action = HighCubeAction(reversed=facing_alliance != robot_status.get_alliance())
        
    if operator_controller.getRisingEdgeButton(operator_params.mid_node_button_id):
        if pinch_active:
            arm_action = MidConeAction(reversed=facing_alliance != robot_status.get_alliance())
        else:
            arm_action = MidCubeAction(reversed=facing_alliance != robot_status.get_alliance())    
        
    if operator_controller.getRisingEdgeButton(operator_params.hybrid_node_button_id):
        arm_action = HybridAction(reversed=facing_alliance != robot_status.get_alliance())
 
    if operator_controller.getRisingEdgeButton(operator_params.in_bot_button_id):
        arm_action = InBotAction()

    # TODO: Which way ground go?
    
    if arm_action is not None:
        action_runner.start_action(arm_action)

    ################################################################################
    ###                         END CONTROL MAPPINGS                             ###
    ################################################################################



    if drive_joystick.getRisingEdgeButton(drive_params.driver_outtake_button_id):
        odom_pub.publish(get_reset_odom_msg())

    hmi_pub.publish(hmi_update_msg)
    action_runner.loop(robot_status.get_mode())


def init_params():
    global drive_params
    global operator_params


    load_parameter_class(drive_params)
    load_parameter_class(operator_params)

def ros_main(node_name):
    global hmi_pub
    global odom_pub
    global intake_pub
    global led_control_pub
    global odometry_subscriber

    rospy.init_node(node_name)
    init_params()

    register_for_robot_updates()

    hmi_pub = rospy.Publisher(name="/HMISignals", data_class=HMI_Signals, queue_size=10, tcp_nodelay=True)
    odom_pub = rospy.Publisher(name="/ResetHeading", data_class=Odometry, queue_size=10, tcp_nodelay=True)
    intake_pub = rospy.Publisher(name="/IntakeControl", data_class=Intake_Control, queue_size=10, tcp_nodelay=True)
    led_control_pub = rospy.Publisher(name="/LedControl", data_class=Led_Control, queue_size=10, tcp_nodelay=True)
    odometry_subscriber = BufferedROSMsgHandlerPy(Odometry)
    odometry_subscriber.register_for_updates("odometry/filtered")
    rospy.Subscriber(name="/JoystickStatus", data_class=Joystick_Status, callback=joystick_callback, queue_size=1, tcp_nodelay=True)
    rospy.spin()
