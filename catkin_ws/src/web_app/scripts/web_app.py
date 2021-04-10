#!/usr/bin/env python
# A dash server along with a ROS node that handles and logs user actions

import rospy

from web_pkg.web import Dashboard


def main():
    rospy.init_node('turtle_dashboard')
    dashboard = Dashboard()
    dashboard.start()
    # The start actually spins internally using dash


if __name__ == '__main__':
    main()
