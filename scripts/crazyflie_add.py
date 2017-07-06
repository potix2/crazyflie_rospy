#!/usr/bin/env python

import sys
import rospy
from crazyflie_rospy.srv import AddCrazyflie, AddCrazyflieRequest, AddCrazyflieResponse

def main(args):
    rospy.init_node('crazyflie_add')

    uri = rospy.get_param("~uri")
    tf_prefix = rospy.get_param("~tf_prefix")
    roll_trim = rospy.get_param("~roll_trim", 0.0)
    pitch_trim = rospy.get_param("~pitch_trim", 0.0)
    enable_logging = rospy.get_param("~enable_logging", True)
    enable_parameters = rospy.get_param("~enable_parameters", True)
    use_ros_time = rospy.get_param("~use_ros_time", True)
    enable_logging_imu = rospy.get_param("~enable_logging_imu", True)
    enable_logging_temperature = rospy.get_param("~enable_logging_temperature", True)
    enable_logging_magnetic_field = rospy.get_param("~enable_logging_magnetic_field", True)
    enable_logging_pressure = rospy.get_param("~enable_logging_pressure", True)
    enable_logging_battery = rospy.get_param("~enable_logging_battery", True)
    height_hold = rospy.get_param("~height_hold", False)

    rospy.loginfo("wait_for_service add_crazyflie...")
    rospy.wait_for_service('/add_crazyflie')
    rospy.loginfo("done")
    try:
        add_crazyflie = rospy.ServiceProxy('/add_crazyflie', AddCrazyflie)
        req = AddCrazyflieRequest(
            uri,
            tf_prefix,
            roll_trim,
            pitch_trim,
            enable_logging,
            enable_parameters,
            [],
            use_ros_time,
            enable_logging_imu,
            enable_logging_temperature,
            enable_logging_magnetic_field,
            enable_logging_pressure,
            enable_logging_battery,
            height_hold
        )
        add_crazyflie(req)
    except rospy.ServiceException as exc:
        rospy.logerr('Service did not process request: ' + str(exc))


if __name__ == '__main__':
    main(sys.argv)
