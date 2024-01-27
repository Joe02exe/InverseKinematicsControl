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

    def __init__(self, group_name: str, planning_time=10.0, planning_attemp_num=10) -> None:
        self.group_name: str = group_name
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.planning_frame: str = self.move_group.get_planning_frame()
        self.eef_link: str = self.move_group.get_end_effector_link()

        # set planning time and number of planning attempts
        self.move_group.set_planning_time(planning_time)
        self.move_group.set_num_planning_attempts(planning_attemp_num)

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
        rospy.loginfo(
            f"MoveGroup \"{self.group_name}\" received target. Trying to move to joint_values {joint_values} ...")
        self.move_group.set_joint_value_target(joint_values)
        plan = self.move_group.go(wait=True)
        if plan is False:
            return False
        return True


class Moveit_main_node():

    def __init__(self):
        rospy.init_node('move_group', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # Get the name of the groups in the Panda robot -> should be "panda_arm" and "panda_hand"
        self.group_names: List[str] = self.robot.get_group_names()
        self.panda_arm: MoveGroup = MoveGroup("panda_arm")
        self.panda_hand: MoveGroup = MoveGroup("panda_hand")
        self.tf_listener = tf.TransformListener()
        self.touch_links = self.robot.get_link_names(
            group=self.panda_hand.group_name)
        # self.scene.attach_box(eef_link, box_name, touch_links=touch_links)

    def load_objects_from_tf(self):
        self._get_tables()
        self._get_objects()
        self._get_containers()
        self._get_obstacles()

    def _get_tables(self):
        self.table_list = self._get_object_from_tf("Table", NUM_TABLES)
        for table in self.table_list:
            table.dimensions = [1.0, 0.6, 0.25]
            self.add_object_to_scene(table)

    def _get_objects(self):
        self.object_list = self._get_object_from_tf("Object", NUM_OBJECTS)
        for object in self.object_list:
            object.dimensions = [0.05, 0.05, 0.05]
            self.add_object_to_scene(object)

    def _get_containers(self):
        self.container_list = self._get_object_from_tf(
            "Container", NUM_CONTAINERS)
        for container in self.container_list:
            container.dimensions = [0.12927, 0.12927, 0.05265]
            self.add_object_to_scene(container)

    def _get_obstacles(self):
        self.obstacle_list = self._get_object_from_tf(
            "Obstacle", NUM_OBSTACLES)
        for obstacle in self.obstacle_list:
            obstacle.dimensions = [0.05, 0.05, 0.35]
            self.add_object_to_scene(obstacle)

    def _get_object_from_tf(self,
                            object_name_prefix: str,
                            object_count: int,
                            reference_frame_name: str = "/reference_frame"):

        rate = rospy.Rate(10.0)
        object_list: List[TfObject] = []

        for i in range(1, object_count + 1):
            object_name: str = f"{object_name_prefix}_{i}"
            rospy.loginfo(
                f"Waiting for Transfrom: \"{reference_frame_name}\" - \"{object_name}\"")
            while True:
                try:
                    (trans, rot) = self.tf_listener.lookupTransform(
                        reference_frame_name, object_name, rospy.Time(0))
                    object = TfObject(name=object_name,
                                      position=trans,
                                      orientation=rot)
                    object_list.append(object)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rate.sleep()
                    continue
                break
        return object_list

    def clean_scene(self):
        rospy.loginfo("starting cleanup of scene ...")
        if self.table_list is not None:
            for table in self.table_list:
                if table.loaded_in_scene:
                    self.scene.remove_world_object(table.name)
        if self.object_list is not None:
            for object in self.object_list:
                if object.loaded_in_scene:
                    if object.is_attached_to_gripper:
                        self.detach_object_from_gripper(object)
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

    def attach_object_to_gripper(self, object):
        rospy.loginfo(f"attaching {object.name} to gripper")
        if not object.loaded_in_scene:
            rospy.logerr(
                f"object {object.name} is not loaded in scene, cannot attach it")
            return
        if object.is_attached_to_gripper:
            rospy.logerr(
                f"object {object.name} is already attached to gripper, cannot attach it again")
            return
        self.scene.attach_box(self.panda_arm.eef_link,
                              object.name, touch_links=self.touch_links)
        object.is_attached_to_gripper = True

    def detach_object_from_gripper(self, object):
        if not object.is_attached_to_gripper:
            rospy.log_err(
                f"object {object.name} is not attached to gripper, cannot detach")
            return
        rospy.loginfo(f"detaching object {object.name} from gripper")
        self.scene.remove_attached_object(
            self.panda_arm.eef_link, name=object.name)
        object.is_attached_to_gripper = False

    def add_object_to_scene(self, tf_object: TfObject):
        self.scene.add_box(
            tf_object.name,
            tf_object.get_as_PoseStamped(),
            tf_object.dimensions)

        tf_object.loaded_in_scene = True

    def remove_object_from_scene(self, object):
        self.scene.remove_world_object(object.name)
        object.loaded_in_scene = False

    def open_fingers(self):
        self.panda_hand.move_to_joint_values([0.04, 0.04])

    def close_fingers(self):
        self.panda_hand.move_to_joint_values([0.02, 0.02])

    def init_upright_path_constrains(self, pose):
        """not used should make the robot upright, at the moment we just rotate the object"""
        self.upright_constraints = moveit_msgs.msg.Constraints()
        self.upright_constraints.name = "upright"
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        orientation_constraint.header = pose.header
        orientation_constraint.link_name = self.panda_arm.eef_link
        orientation_constraint.orientation = pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.4
        orientation_constraint.absolute_y_axis_tolerance = 0.4
        orientation_constraint.absolute_z_axis_tolerance = 0.4
        orientation_constraint.weight = 1.0

        self.upright_constraints.orientation_constraints.append(
            orientation_constraint)

        self.panda_arm.move_group.set_path_constraints(
            self.upright_constraints)

    def abort_pick_and_place(self, i):
        rospy.logerr(
            f"move to object failed, skipping the rest of the pick and place routine for object {i + 1}")
        self.open_fingers()

    def try_pick_with_gripper(self, object: TfObject):
        self.panda_arm.move_group.set_support_surface_name(
            self.table_list[0].name)
        self.panda_hand.move_group.set_support_surface_name(
            self.table_list[0].name)

        grasps = []
        grasp = moveit_msgs.msg.Grasp()
        grasp.grasp_pose.header.frame_id = "panda_link0"

        pose = self.object_list[0].get_as_PoseStamped()
        quaternion = Rotation.from_quat([pose.pose.orientation.x, pose.pose.orientation.y,
                                         pose.pose.orientation.z, pose.pose.orientation.w])
        quaternion = quaternion * Rotation.from_euler('x', np.pi)
        quaternion = quaternion * Rotation.from_euler('z', np.pi / 4)
        orientation = quaternion.as_quat()
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]

        grasp.grasp_pose = pose

        grasp.pre_grasp_approach.direction.header.frame_id = "panda_link0"
        grasp.pre_grasp_approach.direction.vector.z = 1.0
        grasp.pre_grasp_approach.min_distance = 0.095
        grasp.pre_grasp_approach.desired_distance = 0.25

        grasp.post_grasp_retreat.direction.vector.z = 1.0
        grasp.post_grasp_retreat.min_distance = 0.1
        grasp.post_grasp_retreat.desired_distance = 0.25

        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(0.04)
        point.positions.append(0.04)
        point.time_from_start = rospy.Duration(0.5)
        grasp.pre_grasp_posture.points.append(point)
        grasp.pre_grasp_posture.joint_names.append("panda_leftfinger")
        grasp.pre_grasp_posture.joint_names.append("panda_rightfinger")

        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(0.02)
        point.positions.append(0.02)
        point.time_from_start = rospy.Duration(0.5)
        grasp.grasp_posture.points.append(point)
        grasp.grasp_posture.joint_names.append("panda_leftfinger")
        grasp.grasp_posture.joint_names.append("panda_rightfinger")

        grasps.append(grasp)

        self.panda_arm.move_group.pick('Object_1', grasps)
        return

    def pick_and_place_objects(self):
        for i in range(NUM_OBJECTS):
            print("")
            rospy.loginfo(
                f"#### start pick and place routine for object {i + 1} ####")
            self.close_fingers()
            if not self.move_arm_to_object(self.object_list[i], z_offset=0.2):
                self.abort_pick_and_place(i)
                continue
            self.open_fingers()
            self.remove_object_from_scene(self.object_list[i])
            if not self.move_arm_to_object(self.object_list[i], z_offset=0.12):
                self.abort_pick_and_place(i)
                continue
            self.add_object_to_scene(self.object_list[i])
            self.attach_object_to_gripper(self.object_list[i])
            self.close_fingers()
            if not self.move_arm_to_object(self.object_list[i], z_offset=0.2):
                self.abort_pick_and_place(i)
                continue
            if not self.move_arm_to_object(self.container_list[i], z_offset=0.2):
                self.abort_pick_and_place(i)
                continue
            self.detach_object_from_gripper(self.object_list[i])
            self.open_fingers()


if __name__ == '__main__':

    print("\n#########\n# start #\n#########\n")

    # Init our Node that will handle the moveit
    moveit: Moveit_main_node = Moveit_main_node()

    # Load the objects from the tf
    moveit.load_objects_from_tf()

    # Start the pick and place routine
    moveit.pick_and_place_objects()

    input("Press Enter to continue...")

    # Clean the scene - remove all objects from the scene
    moveit.clean_scene()

    print("\n#######\n# end #\n#######\n")