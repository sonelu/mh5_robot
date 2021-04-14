import yaml
from os.path import join

import rospy
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class Portfolio():
    """A portfolio of scripts.

    A Portfolio is a collection of scripts that can share between them
    elements like scenes and poses.

    A portfolio is composed of the following elements:

    - ``joint_names``: a list of joints that the scenes in the portfolio can
      access, in that specific order. To avoid having to list them again
      and again for each pose it is expected that all joints have to be used
      precisely in the order they are listed in the ``joint_names``.

    .. code-block:: YAML

        joint_names: [a, b, c, d]

    - ``poses`` is a dictionary of poses, each with its own name, followed
      by a list of joint positions - for all joints in the ``joint_names`` in
      that order.

    .. code-block:: YAML

        poses:
            pose1 : [1.0, 1.0, 0.0, 0.0]
            pose2 : [2.0, 1.0, 1.0, 1.0]
            ...

    - ``scenes`` is a dictionary of scenes, each scene containing a list of
      poses to be reached together with the duration needed to reach that pose.

    .. code-block:: YAML

        scenes:
            sceneA:
                - {pose: pose1, duration: 2.0}
            sceneB:
                - {pose: pose2, duration: 1.0}
                - {pose: pose1, duration: 1.0}

    - ``scripts`` is a distionary of scripts, each containing a list of scenes
      and allowing to specify a number of repeats and if the exeecution should be
      in reverse.

    .. code-block:: YAML

        scripts:
            scriptX:
                - {scene: A, inverse: false}
                - {scene: B, inverse: false, repeat: 5}
                - {scene: A, inverse: true}

    """

    @classmethod
    def from_file(cls, path, file_name):
        """Constructs a ``Portfolio`` object by reading a YAML defintion
        file and parsing it.

        Raises
        ------
        ValueError:
            if it cannot find the requested file or the data is
            incorrect. Addtional details are provided in the exception
            text.
        """
        with open(join(path, file_name), 'r') as f:
            content = yaml.load(f, Loader=yaml.FullLoader)
        return Portfolio(name=file_name, **content)
        
    def __init__(self, name='dummy_portfolio', units='rad', joints=[], poses={}, scenes={}, scripts={}, **kwargs):
        self.name = name

        # check units
        if units not in ['rad', 'deg']:
            raise ValueError(f'>>> Portfolio file {name} units should be "rad" or "deg" only')
        self.units = units

        # check joints
        if not joints:
            raise ValueError(f'>>> Portfolio file {name} does not contain any joints')
        self.joints = joints

        # check poses
        if not poses:
            raise ValueError(f'>>> Portfolio {name} does not contain any poses')
        for pose_name, pose_details in poses.items():
            if isinstance(pose_details, list):
                # default joints to be used
                if not len(pose_details) == len(joints):
                    raise ValueError(f'>>> Pose {pose_name} must specify positions for all joints')
                for value in pose_details:
                    if not isinstance(value, (int, float)):
                        raise ValueError(f'>>> Pose {pose_name} contains value {value} that is not int or float')
            
            elif isinstance(pose_details, dict):
                # subset list of joints
                if 'joints' not in pose_details:
                    raise ValueError(f'>>> Pose {pose_name} if using subset of joins you need to use "joints" key')
                if 'positions' not in pose_details:
                    raise ValueError(f'>>> Pose {pose_name} if using subset of joins you need to use "positions" key')
                if not isinstance(pose_details['joints'], list):
                    raise ValueError(f'>>> Pose {pose_name} "joints" should be a list')
                for joint in pose_details['joints']:
                    if joint not in self.joints:
                        raise ValueError(f'>>> Pose {pose_name} joint {joint} is not listed in the "joints" in the portfolio')
                if not isinstance(pose_details['positions'], list):
                    raise ValueError(f'>>> Pose {pose_name} "positions" should be a list')
                for value in pose_details['positions']:
                    if not isinstance(value, (int, float)):
                        raise ValueError(f'>>> Pose {pose_name} contains value {value} that is not int or float')
                if not len(pose_details['joints']) == len(pose_details['positions']):
                    raise ValueError(f'>>> Pose {pose_name} list of "joints" has different length than "positions')

            else:
                raise ValueError(f'>>> Pose {pose_name} must specify a list of postions or a "joints" + "positions" set of lists')

        self.poses = poses

        # check scenes
        if not scenes:
            raise ValueError(f'>>> Portfolio {name} does not contain any scenes')
        for scene_name, scene_details in scenes.items():
            if not isinstance(scene_details, list):
                raise ValueError(f'>>> Scene {scene_name} must specify a list of poses')
            for index, pose in enumerate(scene_details):
                if not isinstance(pose, dict):
                    raise ValueError(f'>>> Scene {scene_name} item {index+1} must be a dictionary pose:<pose name>, duration: <duration>')
                if 'pose' not in pose:
                    raise ValueError(f'>>> Scene {scene_name} item {index+1} must be a dictionary pose:<pose name>, duration: <duration>')
                if pose['pose'] not in self.poses:
                    raise ValueError(f'>>> Scene {scene_name} item {index+1} pose {pose["pose"]} does not exist')
                if 'duration' not in pose:
                    raise ValueError(f'>>> Scene {scene_name} item {index+1} must be a dictionary pose:<pose name>, duration: <duration>')
                if not isinstance(pose['duration'],(int, float)):
                    raise ValueError(f'>>> Scene {scene_name} item {index+1} duration {pose["duration"]} should be an int or float')
        self.scenes = scenes

        # check scripts
        if not scripts:
            raise ValueError(f'>>> Portfolio file {name} does not contain any scripts')
        for script_name, script_details in scripts.items():
            if not isinstance(script_details, list):
                raise ValueError(f'>>> Script {script_name} must specify a list of scenes')
            for index, scene in enumerate(script_details):
                if not isinstance(scene, dict):
                    raise ValueError(f'>>> Script {script_name} item {index+1} must be a dictionary scene:<scene name>, inverse: <True/False>, repeat: <repeats>')
                if 'scene' not in scene:
                    raise ValueError(f'>>> Script {script_name} item {index+1} must be a dictionary scene:<scene name>, inverse: <True/False>, repeat: <repeats>')
                if scene['scene'] not in self.scenes:
                    raise ValueError(f'>>> Script {script_name} item {index+1} scene {scene["scene"]} does not exist')
                if 'inverse' not in scene:
                    scene['inverse'] = False
                elif not isinstance(scene['inverse'], bool):
                    raise ValueError(f'>>> Script {script_name} item {index+1} inverse {scene["inverse"]} should be an bool')
                if 'repeat' not in scene:
                    scene['repeat'] = 1
                elif not isinstance(scene['repeat'], int):
                    raise ValueError(f'>>> Script {script_name} item {index+1} inverse {scene["repeat"]} should be an int')
        self.scripts = scripts

    def get_script_names(self):
        return list(self.scripts.keys())

    def to_joint_trajectory_goal(self, script_name, speed):
        if not script_name in self.scripts:
            return None

        if speed <= 0.0:
            speed = 1.0
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joints

        running_duration = 0
        pos = {}
        for joint in self.joints:
            pos[joint] = 0.0

        if self.units == 'deg':
            fact = 57.295779513
        else:
            fact = 1.0

        script_steps = self.scripts[script_name]
        for script_step in script_steps:
            # TODO handle inverse
            scene_name = script_step['scene']
            inverse = script_step['inverse']
            repeat = script_step['repeat']

            for _ in range(repeat):
                
                scene_steps = self.scenes[scene_name]

                for scene_step in scene_steps:

                    pose_name = scene_step['pose']
                    duration = scene_step['duration'] / speed

                    if isinstance(self.poses[pose_name], list):
                        # full list of joints
                        for index, joint in enumerate(self.joints):
                            pos[joint] = self.poses[pose_name][index]
                    else:
                        # subset of joints
                        for index, joint in enumerate(self.poses[pose_name]['joints']):
                            pos[joint] = self.poses[pose_name]['positions'][index]

                    point = JointTrajectoryPoint()
                    point.positions = [pos[joint] / fact for joint in self.joints]
                    point.time_from_start = rospy.Duration.from_sec(running_duration + duration)
                    goal.trajectory.points.append(point)

                    running_duration = running_duration + duration

        return goal