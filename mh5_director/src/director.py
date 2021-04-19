
"""
    A Director ...
"""
from os import listdir
from os.path import isfile, join

import rospkg
import rospy
import actionlib

import xml.etree.ElementTree as ET

from control_msgs.msg import FollowJointTrajectoryAction
from mh5_director.msg import RunScript

# from mh5_director.msg import RunScriptAction, RunScriptResult, RunScriptFeedback
from portfolio import Portfolio

class Director:
    """The Director class

    Reads script definitions from YAML files and can execute them by passing
    the pose information to the ``dynamixel_control/follow_joint_trajectory``
    action server.

    Listens to the ``director/run`` topic for commands to execute a script.
    """

    def __init__(self, portfolio_path=None):
        """Initializes the director.

        If the path is not provided in the call, the method
        uses the ``portfolio`` directory in the ``mh5_director`` package.

        :param portfolio_path: the path to use for loading script definctions
        , defaults to None
        :type portfolio_path: string, optional
        """
        if not portfolio_path:
            self.portfolio_path = '/portfolio'
        else:
            self.portfolio_path = portfolio_path
        self.portfolios =  {}
        # self.joint_controllers = {}
        # self.clients = {}
        self.run_server = None
        self.action_client = None


    def load_scripts(self):
        """Loads XML definitions from the param server and stores them
        in the ``portfolios`` attribute.

        """

        root = ET.fromstring(rospy.get_param(self.portfolio_path))

        for p in root:
            if p.tag != 'portfolio':
                raise ValueError(f'>>> unexpected {p.tag} tag; only <portfolio> tag should be used')

            if 'name' not in p.attrib:
                raise ValueError('[mh5_director] missing name attribute in portfolio definition')

            name = p.attrib['name']
            rospy.loginfo(f'[mh5_director] loading portfolio {name}')
            portfolio = Portfolio.from_xml(p)

            if (portfolio):
                self.portfolios[portfolio.name] = portfolio
                for script in portfolio.scripts:
                    rospy.loginfo(f"[mh5_director] ... script {script} available in portfolio {portfolio.name}")

    def setup_services(self):
        """Starts the subscriptions.

        Director subscribes to:
        - ``director/run`` - used to trigger the execution of a script
        - ...
        """
        self.run_server = rospy.Subscriber('director/run', RunScript, self.run_script_callback)
        rospy.loginfo('[mh5_director] director/run waiting for commands...')

    def setup_action_client(self):
        """Sets up the subscription to the ``dynamixel_control/follow_joint_trajectory``
        action server.

        The function will wait for the action server to become available.
        """
        self.action_client = actionlib.SimpleActionClient('dynamixel_control/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('[mh5_director] wating for action server: dynamixel_control/follow_joint_trajectory')
        self.action_client.wait_for_server()
        rospy.loginfo('[mh5_director]...action server available')

    def run_script_callback(self, msg):
        """Callback for ``director/run``

        The request script should be in the form: <portfolio.script>.
         Will log errors if the requested portfolio or script in that portfolio
         doesn't exist.

         The information in the script is converted into a JointTrajectoryGoal
         and passed to the action server. If the ``feedback`` attribute in the
         message is True, the script_feedback_callback() will be also submitted
         to the send_goal() method of the action client. If the ``wait``
         attribute in the message is True the method will wait for the 
         action server to finish before completing.

        :param msg: message received 
        :type msg: RunScript
        """
        combo_name = msg.script
        port_name, scr_name = combo_name.split('.')[:2]

        if port_name not in self.portfolios:
            rospy.logerr(f'[mh5_director] portfolio {port_name} does not exist')
            return
        if scr_name not in self.portfolios[port_name].scripts:
            rospy.logerr(f'[mh5_director] script {scr_name} does not exist in portfolio {port_name}')
            return

        rospy.loginfo(f'[mh5_director] running script: {scr_name} in portfolio {port_name}')

        goal = self.portfolios[port_name].to_joint_trajectory_goal(scr_name, msg.playback_speed)

        if msg.feedback:
            self.action_client.send_goal(goal, feedback_cb=self.script_feedback_callback)
        else:
            self.action_client.send_goal(goal)

        if msg.wait:
            self.action_client.wait_for_result()
            rospy.loginfo(f'[mh5_director] script {port_name}.{scr_name} completed')
            # print some results
            # ...
        

    def script_feedback_callback(self, feedback):
        """Provides feedback while running the script.

        :param feedback: the feedback provided by the action server
        :type feedback: FollowJointTrajectoryFeedback
        """
        num_joints = len(feedback.joint_names)
        # rospy.loginfo(f'[mh5_director] {feedback.actual.time_from_start.to_sec()}')
        rospy.loginfo(f'[director] avg error: {sum(feedback.error.positions)/num_joints:.2f}, max error: {max(feedback.error.positions):.2f}')
        # detail feedback



