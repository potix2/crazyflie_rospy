#!/usr/bin/env python

import threading
import sys
import rospy
from crazyflie_rospy.srv import AddCrazyflie, AddCrazyflieRequest, AddCrazyflieResponse
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


class CrazyflieROS(object):
    INITIAL_TARGET_HEIGHT=0.4
    MAX_TARGET_HEIGHT=1.0
    MIN_TARGET_HEIGHT=0.03
    INPUT_READ_PERIOD=0.01

    def __init__(self,
            link_url,
            tf_prefix,
            roll_trim,
            pitch_trim,
            enable_logging,
            enable_parameters,
            log_blocks,
            use_ros_time,
            enable_logging_imu,
            enable_logging_temperature,
            enable_logging_magnetic_field,
            enable_logging_pressure,
            enable_logging_battery,
            height_hold):

        self.link_url = link_url
        self.tf_prefix = tf_prefix
        self.roll_trim = roll_trim
        self.pitch_trim = pitch_trim
        self.enable_logging = enable_logging
        self.enable_parameters = enable_parameters
        self.log_blocks = log_blocks
        self.use_ros_time = use_ros_time
        self.enable_logging_imu = enable_logging_imu
        self.enable_logging_temperature = enable_logging_temperature
        self.enable_logging_magnetic_field = enable_logging_magnetic_field
        self.enable_logging_pressure = enable_logging_pressure
        self.enable_logging_battery = enable_logging_battery
        self.height_hold = height_hold

        self.send_setpoint = False
        self.target_height = INITIAL_TARGET_HEIGHT

        rospy.Subscriber(self.tf_prefix + '/cmd_vel', Twist, self.cmd_vel_changed)
        rospy.ServiceProxy(self.tf_prefix + '/emergency', Bool, self.emergency)

    def cmd_vel_changed(self, msg):
        if not self.is_emergency:
            roll = msg.linear.y + self.roll_trim
            pitch = - (msg.linear.x + self.pitch_trim)
            yawrate = msg.angular.z

            if not self.height_hold:
                thrust = min(max(msg.linear.z, 0.0), 60000)
                self.cf.send_setpoint(roll, pitch, yawrate, thrust)
                self.send_setpoint = true
            else:
                vz = (msg.linear.z - 32767) / 32767.0
                self.target_height += vz * INPUT_READ_PERIOD
                self.target_height = min(max(self.target_height, MIN_TARGET_HEIGHT), MAX_TARGET_HEIGHT)
                self.cf.send_zdistance_setpoint(roll, pitch, yawrate, self.target_height)

    def run(self):
        pass


def add_crazyflie(req):
    cf = CrazyflieROS(
            req.uri,
            req.tf_prefix,
            req.roll_trim,
            req.pitch_trim,
            req.enable_logging,
            req.enable_parameters,
            req.log_blocks,
            req.use_ros_time,
            req.enable_logging_imu,
            req.enable_logging_temperature,
            req.enable_logging_magnetic_field,
            req.enable_logging_pressure,
            req.enable_logging_battery,
            req.height_hold)
    return AddCrazyflieResponse()


def main(args):
    rospy.init_node('crazyflie_server')
    rospy.wait_for_service('add_crazyflie')
    s = rospy.Service('add_crazyflie', AddCrazyflie, add_crazyflie)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
