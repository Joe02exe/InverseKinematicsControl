import tf
from typing import List
import numpy as np
from scipy.spatial.transform import Rotation
import rospy
import moveit_commander
import geometry_msgs.msg

NUM_CONTAINERS = 3
NUM_OBJECTS = 3
NUM_OBSTACLES = 4
NUM_TABLES = 2

class TfObject():
    """
    The TfObject class represents the transformation object with a name, position, and orientation in a 3D space.
    It includes default values for the planning frame and dimensions, with special adjustments
    based on the object's name.
    """
    def __init__(self, name: str, position, orientation):
        self.name = name
        self.position = position
        self.orientation = orientation
        self.planning_frame = "panda_link0"
        self.dimensions = [1.0, 1.0, 1.0]

        if name.startswith("Obstacle"):
            position[2] = 0.42
        elif name.startswith("Table"):
            position[2] = 0.11
        elif name.startswith("Object"):
            position[2] = 0.27
        elif name.startswith("Container"):
            position[2] = 0.31

    def get_as_PoseStamped(self) -> geometry_msgs.msg.PoseStamped:
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = self.position[0]
        pose.pose.position.y = self.position[1]
        pose.pose.position.z = self.position[2]
        pose.pose.orientation.x = self.orientation[0]
        pose.pose.orientation.y = self.orientation[1]
        pose.pose.orientation.z = self.orientation[2]
        pose.pose.orientation.w = self.orientation[3]
        return pose


class Inverse_Kinematric_Control_Node():
    """
    This class contains all the logic: from placing the object to moving the objects
    """
    def __init__(self):
        rospy.init_node('inverse_kinematics_control')
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.panda_arm = moveit_commander.MoveGroupCommander("panda_arm")
        self.panda_hand = moveit_commander.MoveGroupCommander("panda_hand")
        self.tf_listener = tf.TransformListener()

    def load_objects_from_tf(self):
        """
        method to load the objects from coppelia into the moveit scene
        """
        self.tables = self._get_object_from_tf("Table", NUM_TABLES)
        for table in self.tables:
            table.dimensions = [1.0, 0.6, 0.25]
            self.add_object_to_scene(table)

        self.objects = self._get_object_from_tf("Object", NUM_OBJECTS)
        for obj in self.objects:
            obj.dimensions = [0.05, 0.05, 0.05]
            self.add_object_to_scene(obj)

        self.containers = self._get_object_from_tf("Container", NUM_CONTAINERS)
        for container in self.containers:
            container.dimensions = [0.12927, 0.12927, 0.05265]
            self.add_object_to_scene(container)

        self.obstacles = self._get_object_from_tf("Obstacle", NUM_OBSTACLES)
        for obstacle in self.obstacles:
            obstacle.dimensions = [0.05, 0.05, 0.35]
            self.add_object_to_scene(obstacle)

    def _get_object_from_tf(self, obj_name: str, object_count: int):
        """ 
        method to get the object from the tf message
        :param name: the name of the object
        :param object_count: The amount of objects in total
        """
        rate = rospy.Rate(50.0)

        objects = []
        for i in range(0, object_count):
            object_name: str = f"{obj_name}_{i + 1}"
            while True:
                try:
                    (pos, rot) = self.tf_listener.lookupTransform("/reference_frame", object_name, rospy.Time(0))
                    objects.append(TfObject(name=object_name, position=pos, orientation=rot))
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rate.sleep()
        return objects

    def move_arm_to_object(self, obj: TfObject, offset=True) -> bool:
        """
        method that moves the panda arm to the object
        :param obj: the goal object to move to
        :param offset: tells us if the moving process is combined with a grappign process
        """
        pose = obj.get_as_PoseStamped()
        pose.pose.position.z += 0.18 if offset else 0.12
        return self.move_to_pose(self.panda_arm, pose)

    def add_object_to_scene(self, tf_object: TfObject):
        """
        method to add an object to the scene
        :param tf_object: the object that needs to be added to the scene
        """
        self.scene.add_box(tf_object.name, tf_object.get_as_PoseStamped(), tf_object.dimensions)

    def open_hand(self, open: bool):
        """
        method to open the hand of the gripper
        :param open: tells us if the gripper should be opened
        """
        val = [0.04, 0.04] if open else [0.02, 0.02]
        self.panda_hand.set_joint_value_target(val)
        return self.panda_hand.go()

    def pick_and_place_objects(self):
        """
        method to move the cubes from table1 to table2
        """
        for i in range(NUM_OBJECTS):
            self.open_hand(True)

            if not self.move_arm_to_object(self.objects[i]):
                continue

            # Move down to cube and close gripper
            self.scene.remove_world_object(self.objects[i].name)
            if not self.move_arm_to_object(self.objects[i], offset=False):
                continue
            self.open_hand(False)

            if not self.move_arm_to_object(self.objects[i]):
                self.open_hand(True)
                continue
            if not self.move_arm_to_object(self.containers[i]):
                self.open_hand(True)
                continue

            self.open_hand(True)

    def move_to_pose(self, move_group: any, target_pose: geometry_msgs.msg.PoseStamped, ) -> bool:
        """
        method to move to a specific pose
        :param move_group: the group that needs to be moved to the pose
        :param target_pose: the pose to move to
        """
        quaternion = Rotation.from_quat([target_pose.pose.orientation.x, target_pose.pose.orientation.y,
                                         target_pose.pose.orientation.z, target_pose.pose.orientation.w])
        quaternion = quaternion * Rotation.from_euler('x', np.pi)
        quaternion = quaternion * Rotation.from_euler('z', np.pi / 4)
        orientation = quaternion.as_quat()
        target_pose.pose.orientation.x = orientation[0]
        target_pose.pose.orientation.y = orientation[1]
        target_pose.pose.orientation.z = orientation[2]
        target_pose.pose.orientation.w = orientation[3]
        move_group.set_pose_target(target_pose)

        return move_group.go()

if __name__ == '__main__':
    moveit = Inverse_Kinematric_Control_Node()

    moveit.load_objects_from_tf()

    moveit.pick_and_place_objects()
