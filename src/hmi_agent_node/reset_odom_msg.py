import rospy
from nav_msgs.msg._Odometry import Odometry

def get_reset_odom_msg() -> Odometry:

    odom = Odometry()

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'

    odom.pose.pose.orientation.w = 1
    odom.pose.pose.orientation.x = 0
    odom.pose.pose.orientation.y = 0
    odom.pose.pose.orientation.z = 0
    odom.pose.pose.position.x = 0
    odom.pose.pose.position.y = 0
    odom.pose.pose.position.z = 0

    odom.twist.twist.linear.x = 0
    odom.twist.twist.linear.y = 0
    odom.twist.twist.linear.z = 0

    odom.twist.twist.angular.x = 0
    odom.twist.twist.angular.y = 0
    odom.twist.twist.angular.z = 0

    odom.pose.covariance = [
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.00001,
    ]

    odom.twist.covariance =[
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.001,
    ]

    return odom