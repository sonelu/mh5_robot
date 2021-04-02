
"""
    A Director ...
"""

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction

from mh5_director.msg import RunScriptAction, RunScriptResult, RunScriptFeedback
from script import Script

class Director:
    """The Director class
    """

    def __init__(self, script_path):
        self.script_path = script_path
        self.joint_controllers = {}
        self.clients = {}
        self.server = None


    def find_trajectory_controllers(self):
        """Looks for TrajectoryControllers defined in the param server
        and retrieves information about the joints used.
        """
        all_params = rospy.get_param_names()

        for p in all_params:
            if rospy.has_param(p+"/type") and rospy.has_param(p+"joints"):
                if "JointTrajectoryController"  in rospy.get_param(p+"/type"):
                    self.joint_controllers[p] = rospy.get_param(p+"/joints")
                    rospy.loginfo(f'Controller {p} detected with joints: '
                                  '{self.joint_controllers[p]}')
        if self.joint_controllers == {}:
            rospy.logerror('No JointTrajectoryControllers detected.')
            return False
        else:
            return True


    def setup_clients(self):
        for c in self.joint_controllers.keys():
            client = actionlib.SimpleActionClient(c + '/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo(f'Wating for controller: {c}/follow_joint_trajectory')
            client.wait_for_server()
            self.clients[c] = client
            rospy.loginfo(f'...Controller {c} available')


    def setup_server(self):
        rospy.loginfo('Setting up the RunScript server...')
        self.server = actionlib.SimpleActionServer('director', RunScriptAction, self.do_run_script, False)
        rospy.loginfo('Starting the server...')
        self.server.start()
        rospy.loginfo('Server started successfully')


    def do_run_script(goal):
        rospy.loginfo(f'goal received:\n{goal}')

        try:
            script = Script.from_file(goal.script_name, [self.script_path])


        except Exception as e:
            response = RunScriptResult(e.__repr__(), rospy.Duration())
            server.set_aborted(response)
            return

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



    client.wait_for_result()
    result = client.get_result()
    exec_time = rospy.get_rostime() - start_time
    if result.error_code != 0:
        response = RunScriptResult(f'Script {goal.script_name} failed.\n{result.error_string}', exec_time)
        server.set_aborted(response)
    else:
        response = RunScriptResult(f'Script {goal.script_name} completed\n{result.error_string}', exec_time)
        server.set_succeeded(response)