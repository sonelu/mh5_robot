from mh5_director.msg import RunScriptAction, RunScriptGoal
from mh5_controller.srv import ChangeTorque

import rospy
import actionlib


rospy.init_node('script_user')

rospy.loginfo('Setting torque on...')
torque_server = rospy.ServiceProxy('change_torque', ChangeTorque)
result = torque_server(True, [], ['all'])
rospy.loginfo(f'Result\n{result.results}')

client = actionlib.SimpleActionClient('director', RunScriptAction)
client.wait_for_server()

command = RunScriptGoal('stand.yml')
client.send_goal(command)
client.wait_for_result()
rospy.loginfo(f'result\n{client.get_result()}')

_ = input('Press when done')

rospy.loginfo('Testing knees in 5 secs...')
command = RunScriptGoal('t1.yml')
client.send_goal(command)
client.wait_for_result()
rospy.loginfo(f'result\n{client.get_result()}')

_ = input('Press when done')

rospy.loginfo('Setting torque off in 5 secs...')
rospy.sleep(5.0)
torque_server = rospy.ServiceProxy('change_torque', ChangeTorque)
result = torque_server(False, [], ['all'])
rospy.loginfo(f'Result\n{result.results}')
