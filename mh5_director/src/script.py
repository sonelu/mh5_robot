from pathlib import Path
import yaml


class Script():

    @classmethod
    def find_script_file(cls, file_name, script_directories=[]):
        """Tries to find a file with name ``file_name`` in one of the
        directories listed in ``script_directories``. The ``file_name`` can
        be fully qualified, in which case the ``script_directories`` can be
        empty. In that case the method will return ``file_name`` if that file
        exists. If ``script_directories`` exists, the method will try them
        in order and will prepend each to the ``file_name`` before checking
        if the file exists. If one match is found it will be returned.

        Returns
        -------
        string
            the file with full path that matches the given parameters or
            None if no file can be found
        """
        # no addtional directories proviced
        if not script_directories:
            path = Path(file_name)
            if path.is_file():
                return path
            else:
                return None
        # addtional directories provided
        for directory in script_directories:
            path = Path(directory, file_name)
            if path.is_file():
                return path
        # no suitable file found
        return None                
    
    @classmethod
    def from_file(cls, file_name, script_directories=[]):
        """Constructs a ``Script`` object by reading a script defintion
        file and parsing it.

        Raises
        ------
        ValueError:
            if it cannot find the requested script file or the data is
            incorrect. Addtional details are provided in the exception
            text.
        """
        file = Script.find_script_file(file_name, script_directories)
        if not file:
            raise ValueError(f'File {file_name} could not be found')
        with open(file, 'r') as f:
            content = yaml.load(f, Loader=yaml.FullLoader)
        return Script(name=file_name, **content)
        
    def __init__(self, name='dummy_script', units='rad', joints={}, poses={}, scenes={}, scripts={}):
        self.name = name
        self.units = units
        # check joints
        if not joints:
            raise ValueError(f'Script file {name} does not contain any joints')
        for joint_group, joint_group_joints in joints.items():
            if not isinstance(joint_group_joints, list):
                raise ValueError(f'Joint group {joint_group} must use a list to define the joints')
            if not joint_group_joints:
                raise ValueError(f'Joint group {joint_group} has no joints')
        self.joints = joints
        # check poses
        if not poses:
            raise ValueError(f'Script {name} does not contain any poses')
        for pose_name, pose_values in poses.items():
            if 'positions' not in pose_values:
                raise ValueError(f'Pose {pose_name} must include "positions"')
            for value in pose_values:
                if not isinstance(value, (int, float)):
                    raise ValueError(f'Pose {pose_name} contains value {value} that is not int or float')
        self.poses = poses
        # check scenes
        if not scenes:
            raise ValueError(f'Script file {name} does not contain any scenes')
        for scene_name, scene_details in scenes.items():
            for keyword in ['joints', 'poses', 'durations']
                if keyword not in scene_details:
                    raise ValueError(f'Scene {scene_name} does not contain "{keyword}"')
            # check joint group names
            for joint_group in scene_details['joints']:
                if joint_group not self.joints:
                    raise ValueError(f'Joint group {joint_group} used in scene {scene_name} does not exist')
            # check poses
            n_groups = len(scene_details['joints'])
            for pose in scene_details['poses']:
                if not isinstance(pose)
            if not isinstance(scene_details, (tuple, list, set)):
                raise ValueError(f'Scene {scene_name} should be a list of poses but is {type(scene_details)} instead')
            for index, pose_details in enumerate(scene_details):
                if not isinstance(pose_details, list) or len(pose_details) != 2:
                    raise ValueError(f'Item {index+1} in scene {scene_name} is not a list; items should be a list [pose_name, duration]')
                if pose_details[0] not in self.poses:
                    raise ValueError(f'Unknown pose {pose_details[0]} used in the item {index+1} in scene {scene_name}')
                if not isinstance(pose_values[1], (int, float)):
                    raise ValueError(f'Duration {pose_values[1]} should be an int or float')
        self.scenes = scenes
        # check scripts
        if not scripts:
            raise ValueError(f'Script file {name} does not contain any scripts')
        for script_name, script_details in scripts.items():
