import tf
from typing import List
import numpy as np
from scipy.spatial.transform import Rotation
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs

NUM_CONTAINERS = 3
NUM_OBJECTS = 3
NUM_OBSTACLES = 4
NUM_TABLES = 2


class TfObject():
    """Wrapper for the tf object from coppelia - makes it easier to use"""

    def __init__(self,
                 name: str,
                 position: List[float],
                 orientation: List[float],
                 dimensions: List[float] = [1.0, 1.0, 1.0],
                 planning_frame: str = "panda_link0"):
        self.name = name
        self.position: List[float] = position
        self.orientation: List[float] = orientation
        self.planning_frame: str = planning_frame
        self.loaded_in_scene: bool = False
        self.is_attached_to_gripper: bool = False
        self.dimensions: List[float] = dimensions

        # TODO: remove this hack, this is just a hack to make the objects stand on the table
        if name.startswith("Obstacle"):
            position[2] += 0.08
        elif name.startswith("Table"):
            position[2] -= 0.25 / 2
        elif name.startswith("Object"):
            position[2] -= 0.07
        elif name.startswith("Container"):
            position[2] -= 0.09

    def __str__(self) -> str:
        ret = "TfObject: (" + \
            f"name: \"{self.name}\", " + \
            f"position: {self.position}, " + \
            f"orientation: {self.orientation})"
        return ret

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


class MoveGroup():
    """ Wrapper for the moveit move group - makes it easier to use"""

    def __str__(self) -> str:
        ret = "MoveGroup: (" + \
            f"name: \"{self.group_name}\", " + \
            f"planning_frame: \"{self.planning_frame}\", " +\
            f"eef_link: \"{self.eef_link}\")"
        return ret

    def __init__(self, group_name: str) -> None:
        self.group_name: str = group_name
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.planning_frame: str = self.move_group.get_planning_frame()
        self.eef_link: str = self.move_group.get_end_effector_link()

    def move_to_pose(self, target_pose: geometry_msgs.msg.PoseStamped) -> bool:
        quaternion = Rotation.from_quat([target_pose.pose.orientation.x, target_pose.pose.orientation.y,
                                         target_pose.pose.orientation.z, target_pose.pose.orientation.w])
        quaternion = quaternion * Rotation.from_euler('x', np.pi)
        quaternion = quaternion * Rotation.from_euler('z', np.pi / 4)
        orientation = quaternion.as_quat()
        target_pose.pose.orientation.x = orientation[0]
        target_pose.pose.orientation.y = orientation[1]
        target_pose.pose.orientation.z = orientation[2]
        target_pose.pose.orientation.w = orientation[3]
        self.move_group.set_pose_target(target_pose)
        
        plan = self.move_group.go()
        if plan is False:
            return False
        return True

    def move_to_joint_values(self, joint_values) -> bool:
        rospy.loginfo(f"MoveGroup \"{self.group_name}\" received target. Trying to move to joint_values {joint_values} ...")
        self.move_group.set_joint_value_target(joint_values)
        plan = self.move_group.go(wait=True)
        if plan is False:
            return False
        return True


class Inverse_Kinematric_Control_Node():

    def __init__(self):
        rospy.init_node('move_group', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_names: List[str] = self.robot.get_group_names()
        self.panda_arm: MoveGroup = MoveGroup("panda_arm")
        self.panda_hand: MoveGroup = MoveGroup("panda_hand")
        self.tf_listener = tf.TransformListener()
        self.touch_links = self.robot.get_link_names(group=self.panda_hand.group_name)

    def load_objects_from_tf(self):
        self.table_list = self._get_object_from_tf("Table", NUM_TABLES)
        for table in self.table_list:
            table.dimensions = [1.0, 0.6, 0.25]
            self.add_object_to_scene(table)

        self.object_list = self._get_object_from_tf("Object", NUM_OBJECTS)
        for object in self.object_list:
            object.dimensions = [0.05, 0.05, 0.05]
            self.add_object_to_scene(object)

        self.container_list = self._get_object_from_tf(
            "Container", NUM_CONTAINERS)
        for container in self.container_list:
            container.dimensions = [0.12927, 0.12927, 0.05265]
            self.add_object_to_scene(container)

        self.obstacle_list = self._get_object_from_tf(
            "Obstacle", NUM_OBSTACLES)
        for obstacle in self.obstacle_list:
            obstacle.dimensions = [0.05, 0.05, 0.35]
            self.add_object_to_scene(obstacle)

    def _get_object_from_tf(self, obj_name: str, object_count: int, reference_frame_name: str = "/reference_frame"):
        rate = rospy.Rate(50.0)
        
        objects = []
        for i in range(0, object_count):
            object_name: str = f"{obj_name}_{i + 1}"
            while True:
                try:
                    (pos, rot) = self.tf_listener.lookupTransform(reference_frame_name, object_name, rospy.Time(0))
                    objects.append(TfObject(name=object_name, position=pos, orientation=rot))
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rate.sleep()
        return objects

    def clean_scene(self):
        if self.table_list is not None:
            for table in self.table_list:
                if table.loaded_in_scene:
                    self.scene.remove_world_object(table.name)
        if self.object_list is not None:
            for object in self.object_list:
                if object.loaded_in_scene:
                    self.scene.remove_world_object(object.name)
        if self.container_list is not None:
            for container in self.container_list:
                if container.loaded_in_scene:
                    self.scene.remove_world_object(container.name)
        if self.obstacle_list is not None:
            for obstacle in self.obstacle_list:
                if obstacle.loaded_in_scene:
                    self.scene.remove_world_object(obstacle.name)
        rospy.loginfo("cleanup finished")

    def move_arm_to_object(self, object: TfObject, z_offset=0.0) -> bool:
        """move the robot to the object, with a z offset. Returns if the planning was successful"""
        rospy.loginfo(
            f"Move \"{self.panda_arm.group_name}\" received target {object.name} with z_offset {z_offset}. Trying ...")

        pose = object.get_as_PoseStamped()
        pose.pose.position.z += z_offset
        return self.panda_arm.move_to_pose(pose)

    def add_object_to_scene(self, tf_object: TfObject):
        self.scene.add_box(
            tf_object.name,
            tf_object.get_as_PoseStamped(),
            tf_object.dimensions)

        tf_object.loaded_in_scene = True

    def remove_object_from_scene(self, object):
        self.scene.remove_world_object(object.name)
        object.loaded_in_scene = False
        
    def open_hand(self, open: bool):
        val = [0.04, 0.04] if open else [0.02, 0.02]
        self.panda_hand.move_to_joint_values(val)

    def pick_and_place_objects(self):
        for i in range(NUM_OBJECTS):
            self.open_hand(True)
            
            if not self.move_arm_to_object(self.object_list[i], z_offset=0.2):
                continue
            self.remove_object_from_scene(self.object_list[i])
            if not self.move_arm_to_object(self.object_list[i], z_offset=0.12):
                continue
            
            self.open_hand(False)
            
            if not self.move_arm_to_object(self.object_list[i], z_offset=0.2):
                continue
            if not self.move_arm_to_object(self.container_list[i], z_offset=0.2):
                continue
            
            self.open_hand(True)
            
            # TODO Check if cube is in bowl else retry
            
            
if __name__ == '__main__':
    moveit = Inverse_Kinematric_Control_Node()

    moveit.load_objects_from_tf()

    moveit.pick_and_place_objects()
