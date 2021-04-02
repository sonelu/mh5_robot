#!/usr/bin/env python3

import rospy
import rospkg
import actionlib
import os
import yaml
import math

from control_msgs.msg import FollowJointTrajectoryAction
from mh5_director.msg import RunScriptAction, RunScriptResult, RunScriptFeedback
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


def do_run_script(goal):
    rospy.loginfo(f'goal received:\n{goal}')
    # load the script;
    try:
        with open(os.path.join(script_path, goal.script_name), 'r') as f:
            script_def = yaml.load(f, Loader=yaml.FullLoader)
        units = script_def.get('units', 'rad')
        if units == 'deg':
            conv = 180.0 / math.pi
        else:
            conv = 1.0
        joint_names = script_def['joint_names']
        poses = script_def['poses']
        scenes = script_def['scenes']
        script = script_def['script']

        request = FollowJointTrajectoryGoal()
        if type(joint_names) is not list:
            raise TypeError('joint_names should be a list of joints')
        request.trajectory.joint_names = joint_names
        duration_from_start = rospy.Duration(0, 0)
        if type(script) is not list:
            raise TypeError('script should be a list of scenes')
        for scene_name in script:
            scene = scenes[scene_name]
            if type(scene) is not list:
                raise TypeError(f'scene {scene_name} should be a list of poses')
            for pose_name, duration in scene:
                pose = poses[pose_name]
                point = JointTrajectoryPoint()
                point.time_from_start = duration_from_start + rospy.Duration.from_sec(duration)
                if len(pose) != len(joint_names):
                    raise ValueError(f'values for pose {pose_name} do not match the number of joints')
                point.positions = [float(val) / conv for val in pose]
                request.trajectory.points.append(point)
                duration_from_start = point.time_from_start
        start_time = rospy.get_rostime()
        client.send_goal(request, feedback_cb=feedback_follow_joint_trajectory)

    except Exception as e:
        response = RunScriptResult(e.__repr__(), rospy.Duration())
        server.set_aborted(response)
        return

    client.wait_for_result()
    result = client.get_result()
    exec_time = rospy.get_rostime() - start_time
    if result.error_code != 0:
        response = RunScriptResult(f'Script {goal.script_name} failed.\n{result.error_string}', exec_time)
        server.set_aborted(response)
    else:
        response = RunScriptResult(f'Script {goal.script_name} completed\n{result.error_string}', exec_time)
        server.set_succeeded(response)


def feedback_follow_joint_trajectory(feedback):
    pass

if __name__ == '__main__':

    rospy.init_node('mh5_director', log_level=rospy.INFO)

    if rospy.has_param('~script_directory'):
        script_path = rospy.get_param('~script_directory')
    else:
        script_path = os.path.join(rospkg.RosPack().get_path('mh5_director'), 'scripts')
    rospy.loginfo(f'Using path: {script_path}')
    # setup the client
    rospy.loginfo('Setting up the follow_joint_trajectory client...')
    client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo('Waiting for server...')
    client.wait_for_server()
    rospy.loginfo('Server avialable')

    # setup the server
    rospy.loginfo('Setting up the RunScript server...')
    server = actionlib.SimpleActionServer('director', RunScriptAction, do_run_script, False)
    rospy.loginfo('Starting the server...')
    server.start()
    rospy.loginfo('Server started successfully')

    # main loop
    rospy.loginfo('Wating for orders')
    rospy.spin()
