import sys
import copy
from typing import List
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs
from std_msgs.msg import Float64
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import CollisionObject
import tf

NUM_CONTAINERS = 3
NUM_OBJECTS = 3
NUM_OBSTACLES = 4
NUM_TABLES = 2


def all_close(goal, actual, tolerance):
    """
    Convenience metehd for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class TfObject():

    def __init__(self, name: str, position, orientation, planning_frame: str = "panda_link0"):
        self.name = name
        self.position = position
        self.orientation = orientation
        self.planning_frame = planning_frame

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

    def __str__(self) -> str:
        ret = "MoveGroup: (" + \
            "name: \"{self.group_name}\", " + \
            "planning_frame: \"{self.planning_frame}\", " +\
            "eef_link: \"{self.eef_link}\")"
        return ret

    def __init__(self, group_name: str) -> None:
        self.group_name: str = group_name
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()

    def move_to_pose(self, target_pose):
        print(
            f"MoveGroup \"{self.group_name}\" received target pose:\n{target_pose}")
        # self.move_group.set_pose_target(target_pose)
        # success = self.move_group.go(target_pose)

        plan = self.move_group.plan(target_pose)
        print(
            f"MoveGroup \"{self.group_name}\" move_to_pose success: {plan}")
        self.move_group.execute(plan, wait=True)

        self.move_group.stop()
        # self.move_group.execute(plan)
        return

        plan = self.move_group.go(target_pose, wait=True)
        if plan is False:
            rospy.logerr("============ move failed ============")
        else:
            rospy.loginfo("============ Visualizing plan ============")
            print(f"plan: {plan}")
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(target_pose, current_pose, 0.01)


class Moveit_main_node():

    def __init__(self):
        rospy.init_node('move_group', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        print("Moveit_main_node init")
        group_names = self.robot.get_group_names()
        print(f"Available Planning Groups: {self.robot.get_group_names()}")

        self.panda_arm = MoveGroup("panda_arm")
        print(f"panda_arm = {self.panda_arm}")
        # self.panda_hand = MoveGroup("panda_hand")
        #   print(f"panda_hand = {self.panda_hand}")

        self.group_names = group_names
        self.tf_listener = tf.TransformListener()

        self.get_tables()
        self.get_objects()
        self.get_containers()
        self.get_obstacles()

    def get_tables(self):
        self.table_list = self._get_object_from_tf("Table", NUM_TABLES)
        for table in self.table_list:
            self.add_box_to_scene(table, (1.0, 0.6, 0.25))

    def get_objects(self):
        self.object_list = self._get_object_from_tf("Object", NUM_OBJECTS)
        for object in self.object_list:
            self.add_box_to_scene(object, (0.05, 0.05, 0.05))

    def get_containers(self):
        self.container_list = self._get_object_from_tf(
            "Container", NUM_CONTAINERS)
        for container in self.container_list:
            self.add_box_to_scene(container, (0.11835, 0.11835, 0.01037))

    def get_obstacles(self):
        self.obstacle_list = self._get_object_from_tf(
            "Obstacle", NUM_OBSTACLES)
        for obstacle in self.obstacle_list:
            self.add_box_to_scene(obstacle, (0.05, 0.05, 0.35))

    def _get_object_from_tf(self,
                            object_name_prefix: str,
                            object_count: int,
                            reference_frame_name: str = "/reference_frame"):

        rate = rospy.Rate(10.0)
        object_list: List[TfObject] = []

        for i in range(1, object_count + 1):
            object_name: str = f"{object_name_prefix}_{i}"
            print(
                f"\nWaiting for Transfrom: \"{reference_frame_name}\" -  \"{object_name}\"")
            while True:
                try:
                    (trans, rot) = self.tf_listener.lookupTransform(
                        reference_frame_name, object_name, rospy.Time(0))
                    object = TfObject(name=object_name,
                                      position=trans, orientation=rot)
                    print(object)
                    object_list.append(object)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rate.sleep()
                    continue
                break
        return object_list

    def clean_scene(self):
        print("cleanup started")
        for table in self.table_list:
            self.scene.remove_world_object(table.name)
        for object in self.object_list:
            self.scene.remove_world_object(object.name)
        for container in self.container_list:
            self.scene.remove_world_object(container.name)
        for obstacle in self.obstacle_list:
            self.scene.remove_world_object(obstacle.name)
        print("cleanup finished")

    def add_box_to_scene(self, tf_object: TfObject, size=(0.1, 0.1, 0.1)):
        pose_stamp: geometry_msgs.msg.PoseStamped = tf_object.get_as_PoseStamped()
        if tf_object.name.startswith("Obstacle"):
            pose_stamp.pose.position.z += 0.08
        elif tf_object.name.startswith("Table"):
            pose_stamp.pose.position.z -= size[2] / 2
        elif tf_object.name.startswith("Object"):
            pose_stamp.pose.position.z -= 0.07
        elif tf_object.name.startswith("Container"):
            pose_stamp.pose.position.z -= 0.09
        self.scene.add_box(tf_object.name, pose_stamp, size)
        print(f"Added \"{tf_object.name}\" to scene")


if __name__ == '__main__':
    # try:
    moveit: Moveit_main_node = Moveit_main_node()
    pose_goal = geometry_msgs.msg.PoseStamped()
    pose_goal.header.frame_id = 'panda_link0'
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.0
    pose_goal.pose.position.y = 0.0
    pose_goal.pose.position.z = 1.0
    moveit.panda_arm.move_to_pose(pose_goal)
    input("Press Enter to continue...")
    moveit.clean_scene()

    print("\n###########\n# finshed #\n###########\n")
