#!/usr/bin/env python3

import rospy
import rospkg
import os


from director import Director


if __name__ == '__main__':

    rospy.init_node('mh5_director', log_level=rospy.INFO)

    # portfolio directory
    if rospy.has_param('~portfolio_directory'):
        portfolio_path = rospy.get_param('~portfolio_directory')
    else:
        portfolio_path = os.path.join(rospkg.RosPack().get_path('mh5_director'), 'portfolio')
    rospy.loginfo(f'[mh5_director] using path: {portfolio_path}')

    director = Director(portfolio_path)

    director.load_scripts()
    director.setup_services()
    director.setup_action_client()

    # main loop
    rospy.loginfo('[mh5_director] wating for orders')
    rospy.spin()
