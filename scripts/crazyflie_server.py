#!/usr/bin/env python

from threading import Thread
import sys
import time
import rospy

from crazyflie_rospy.srv import AddCrazyflie, AddCrazyflieRequest, AddCrazyflieResponse
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty, EmptyResponse

import cflib
from cflib.crazyflie import Crazyflie


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

        self.link_url = str(link_url)
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

        self.sent_setpoint = False
        self.target_height = self.INITIAL_TARGET_HEIGHT
        self.is_emergency = False

        rospy.Subscriber(self.tf_prefix + '/cmd_vel', Twist, self.cmd_vel_changed)
        rospy.Service(self.tf_prefix + '/emergency', Empty, self.emergency)

        self.cf = Crazyflie()
        self.cf.connected.add_callback(self._connected)
        self.cf.disconnected.add_callback(self._disconnected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)
        self.cf.open_link(str(link_url))
        rospy.loginfo('Connecting to {} (len={})'.format(link_url, len(link_url)))


    def _connected(self, link_url):
        rospy.loginfo('Connected from: {}'.format(link_url))
        Thread(target=self.run).start()

    def _disconnected(self, link_url):
        rospy.loginfo('Disconnected from: {}'.format(link_url))

    def _connection_failed(self, link_url, msg):
        rospy.logerr('Connection to {} failed: {}'.format(link_url, msg))

    def _connection_lost(self, link_url, msg):
        rospy.logerr('Connection to {} lost: {}'.format(link_url, msg))

    def emergency(self, req):
        self.is_emergency = True
        return EmptyResponse()

    def cmd_vel_changed(self, msg):
        if not self.is_emergency:
            roll = msg.linear.y + self.roll_trim
            pitch = - (msg.linear.x + self.pitch_trim)
            yawrate = msg.angular.z

            if not self.height_hold:
                thrust = min(max(msg.linear.z, 0.0), 60000)
                self.cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
                self.sent_setpoint = True
            else:
                vz = (msg.linear.z - 32767) / 32767.0
                self.target_height += vz * self.INPUT_READ_PERIOD
                self.target_height = min(max(self.target_height, self.MIN_TARGET_HEIGHT), self.MAX_TARGET_HEIGHT)
                self.cf.commander.send_zdistance_setpoint(roll, pitch, yawrate, self.target_height)

    def run(self):
        # auto start = std::chrono::system_clock::now();
        if self.enable_parameters:
            # skip update parameters
            pass

        if self.enable_logging:
            # skip logging
            pass

        rospy.loginfo("Ready...");

        # Send 0 thrust initially for thrust-lock
        for i in range(100):
            self.cf.commander.send_setpoint(0, 0, 0, 0)

        while not self.is_emergency:
            # make sure we ping often enough to stream data out
            self.sent_setpoint = False
            time.sleep(0.001)

        # Make sure we turn the engines off
        for i in range(100):
            self.cf.commander.send_setpoint(0, 0, 0, 0)

        self.cf.close_link()


def add_crazyflie(req):
    rospy.loginfo('crazyflie_server: add_crazyflie(uri={})'.format(req.uri))
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
    cflib.crtp.init_drivers(enable_debug_driver=False)
    rospy.init_node('crazyflie_server')
    s = rospy.Service('/add_crazyflie', AddCrazyflie, add_crazyflie)
    rospy.loginfo('ready to add crazyflie!')
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
