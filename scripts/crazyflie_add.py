#!/usr/bin/env python

import sys
import rospy

def main(args):
    rospy.init_node('crazyflie_add')
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
