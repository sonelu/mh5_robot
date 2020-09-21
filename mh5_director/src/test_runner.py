from mh5_director.msg import RunScriptAction, RunScriptGoal
from mh5_controller.srv import ChangeTorque

import rospy
import actionlib
import time


rospy.init_node('script_user')

rospy.loginfo('Setting torque on...')
torque_server = rospy.ServiceProxy('change_torque', ChangeTorque)
result = torque_server(True, [], ['all'])
rospy.loginfo(f'Result\n{result.results}')

client = actionlib.SimpleActionClient('director', RunScriptAction)
client.wait_for_server()

_ = input('Press when ready')
time.sleep(2.0)
rospy.loginfo('Executing stand...')
# _ = input('Press when ready')
command = RunScriptGoal('stand.yml')
client.send_goal(command)
client.wait_for_result()
rospy.loginfo(f'result\n{client.get_result()}')

rospy.loginfo('Executing wave...')
# _ = input('Press when ready')
command = RunScriptGoal('wave.yml')
client.send_goal(command)
client.wait_for_result()
rospy.loginfo(f'result\n{client.get_result()}')

rospy.loginfo('Executing sit...')
# _ = input('Press when ready')
command = RunScriptGoal('sit.yml')
client.send_goal(command)
client.wait_for_result()
rospy.loginfo(f'result\n{client.get_result()}')

rospy.loginfo('Setting torque off...')
# _ = input('Press when ready')
torque_server = rospy.ServiceProxy('change_torque', ChangeTorque)
result = torque_server(False, [], ['all'])
rospy.loginfo(f'Result\n{result.results}')
