import yaml
from os.path import join

import rospy
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class Defaults():

    def __init__(self):
        self.joints = []
        self.duration = None

    @classmethod
    def from_xml(cls, xml_elem, portfolio):

        defaults = Defaults()

        if 'joints' in xml_elem.attrib:
            if isinstance(xml_elem.attrib['joints'], str):
                defaults.joints = xml_elem.attrib['joints'].split(' ')
            else:
                raise ValueError(f'>>> Portfolio {portfolio.name} "joints" should be a string of joint names')

        if 'duration' in xml_elem.attrib:
            try:
                defaults.duration = float(xml_elem.attrib['duration'])
            except:
                raise ValueError(f'>>> Portfolio {portfolio.name} default duration should be "int" or "float"')

        return defaults



class Pose():

    def __init__(self): 
        self.name = ''
        self.joints = []
        self.positions = []

    @classmethod
    def from_xml(cls, xml_elem, portfolio):

        pose = Pose()
        
        err = f'>>> Portfolio {portfolio.name} '
        assert xml_elem.tag == 'pose', err + 'only <pose> tags should be used in <poses>'

        assert 'name' in xml_elem.attrib, err + '"name" attribute expected in <pose> definition'
        pose.name = xml_elem.attrib['name']

        err = err + f'scene {pose.name} '
        assert 'positions' in xml_elem.attrib, err + '"positions" attribute expected'
        positions = xml_elem.attrib['positions']
        assert isinstance(positions, str), err + '"positions" should be a string of positions'
        positions = positions.split(' ')
        try:
            pose.positions = [float(pos) for pos in positions]
        except:
            raise ValueError(err + 'failed to convert positions to floats')

        if 'joints' in xml_elem.attrib:
            assert isinstance(xml_elem.attrib['joints'], str), err + '"joints" should be a string of joint names'
            pose.joints = xml_elem.attrib['joints'].split(' ')
        else:
            pose.joints = portfolio.defaults.joints
        assert len(pose.joints) == len(pose.positions), err + 'list of "joints" has different length than "positions"'

        return pose


class Scene():

    def __init__(self):
        self.name = ''
        self.poses = []
        self.durations = []


    @classmethod
    def from_xml(cls, xml_elem, portfolio):

        scene = Scene()

        err = f'>>> Portfolio {portfolio.name} '
        assert xml_elem.tag == 'scene', err + 'only <scene> tags should be used in <scenes>'

        assert 'name' in xml_elem.attrib, err + '"name" attribute expected in <scene> definition'
        scene.name = xml_elem.attrib['name']

        for pose in list(xml_elem):
            
            err = err + f'scene {scene.name} '
            assert pose.tag == 'pose', err + 'only <pose> tags should be used in <scene>'
            
            assert 'name' in pose.attrib, err + '"name" for pose missing'
            p_name = pose.attrib['name']
            assert p_name in portfolio.poses, err + f'pose {p_name} unknown'
            scene.poses.append(p_name)

            if 'duration' in pose.attrib:
                try:
                    scene.durations.append(float(pose.attrib['duration']))
                except:
                    raise ValueError(err + 'failed to convert duration to float')
            else:
                if portfolio.defaults.duration:
                    scene.durations.append(portfolio.defaults.duration)
                else:
                    raise ValueError(err + 'no duration specified')
            
        return scene


class Script():

    def __init__(self):
        self.name = ''
        self.scenes = []
        self.inverse = []
        self.repeat = []

    @classmethod
    def from_xml(cls, xml_elem, portfolio):

        script = Script()

        err = f'>>> Portfolio {portfolio.name} '
        assert xml_elem.tag == 'script', err + 'only <script> tags should be used in <scripts>'

        assert 'name' in xml_elem.attrib, err + '"name" attribute expected in <script> definition'
        script.name = xml_elem.attrib['name']

        for scene in list(xml_elem):
            
            err = err + f'script {script.name} '
            assert scene.tag == 'scene', err + 'only <scene> tags should be used in <script>'
            
            assert 'name' in scene.attrib, err + '"name" for scene missing'
            s_name = scene.attrib['name']
            assert s_name in portfolio.scenes, err + f'scene {s_name} unknown'
            script.scenes.append(s_name)

            if 'inverse' in scene.attrib:
                if scene.attrib['inverse'] in ['False', 'false']:
                    script.inverse.append(False)
                elif scene.attrib['inverse'] in ['True', 'true']:
                    script.inverse.append(True)
                else:
                    raise ValueError(err + '"inverse" cannot be parse')
            
            if 'repeat' in scene.attrib:
                try:
                    script.repeat.append(int(scene.attrib['repeat']))
                except:
                    raise ValueError(err + '"repeat" should be an "int"')
            else:
                script.repeat.append(1)

        return script


class Portfolio():
    """A portfolio of scripts.

    A Portfolio is a collection of scripts that can share between them
    elements like scenes and poses.

    A portfolio is composed of the following elements:

    

    """
        
    def __init__(self):
        self.name = 'dummy'
        self.units = 'rad'
        self.defaults = None
        self.poses = {}
        self.scenes = {}
        self.scripts = {}


    @classmethod
    def from_xml(cls, xml_elem):
        """Constructs a ``Portfolio`` object by reading an XML defintion
        and parsing it.

        Raises
        ------
        ValueError:
            if the data is incorrect. Additional details are provided in 
            the exception text.
        :param xml_elem: XML element with the structure of the portfolio
        :type xml_elem: xml Element
        """
        portfolio = Portfolio()
        
        assert 'name' in xml_elem.attrib, '>>> Missing "name" attribute in portfolio defintion'
        portfolio.name = xml_elem.attrib['name']
        
        portfolio.units = xml_elem.attrib.get('units', 'rad')
        # check units
        assert portfolio.units in ['rad', 'deg'], f'>>> Portfolio file {portfolio.name} units should be "rad" or "deg" only'

        for child in xml_elem:

            if child.tag == 'defaults':
                portfolio.defaults = Defaults.from_xml(child, portfolio)

            if child.tag == 'poses':
                for pose_xml in child:
                    pose = Pose.from_xml(pose_xml, portfolio)
                    portfolio.poses[pose.name] = pose

            if child.tag == 'scenes':
                for scene_xml in child:
                    scene = Scene.from_xml(scene_xml, portfolio)
                    portfolio.scenes[scene.name] = scene

            if child.tag == 'scripts':
                for script_xml in child:
                    script = Script.from_xml(script_xml, portfolio)
                    portfolio.scripts[script.name] = script

        return portfolio


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
            factor = 57.295779513
        else:
            factor = 1.0

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
                            position = self.poses[pose_name][index]
                            if isinstance(position, str):
                                position = self.variables[position]
                            pos[joint] = position
                    else:
                        # subset of joints
                        for index, joint in enumerate(self.poses[pose_name]['joints']):
                            position = self.poses[pose_name]['positions'][index]
                            if isinstance(position, str):
                                position = self.variables[position]
                            pos[joint] = position

                    point = JointTrajectoryPoint()
                    point.positions = [pos[joint] / factor for joint in self.joints]
                    point.time_from_start = rospy.Duration.from_sec(running_duration + duration)
                    goal.trajectory.points.append(point)

                    running_duration = running_duration + duration

        return goal