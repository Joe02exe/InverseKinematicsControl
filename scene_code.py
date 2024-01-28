# python
import random, os
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose

NUM_TABLES = 2
NUM_CONTAINERS = 3
NUM_OBJECTS = 3
NUM_OBSTACLES = 4


def sysCall_init():
    place_scene_objects()

    global fake_subscriber
    publish_scene_objects()
    fake_subscriber = simROS.subscribe(
        "/move_group/fake_controller_joint_states",
        "sensor_msgs/JointState",
        "joint_state_callback",
    )

def sendTfMessage(objHandle, object_name):
  base_frame = sim.getObject("/reference_frame")
  orientation = sim.getObjectQuaternion(objHandle,base_frame)
  position = sim.getObjectPosition(objHandle,base_frame)
  msg = {
    'header': {
      'stamp': simROS.getTime(),
      'frame_id': '/reference_frame'
    },
    'child_frame_id': object_name,
    'transform': {
      'rotation': {'x': orientation[0], 'y': orientation[1], 'z': orientation[2], 'w': orientation[3]},
      'translation': {'x': position[0], 'y': position[1], 'z': position[2]}
    }
  }
  simROS.sendTransform(msg)


def publish_scene_objects():  
    # get tf of tables
    for i in range(1, NUM_TABLES+1):
        name = f"Table_{i}"
        sendTfMessage(scene_objects['tables'][name], name)
    
    # get tf of obstacles
    for i in range(1, NUM_OBSTACLES+1):
        name = f"Obstacle_{i}"
        sendTfMessage(scene_objects['obstacles'][name], name)

    # get tf of containers (only the upper part)
    for i in range(1, NUM_CONTAINERS+1):
        name = f"Container_{i}"
        upperpart_container= f"/{name}/upperPart"
        sendTfMessage(sim.getObjectHandle(upperpart_container), name)

    # get tf of objects
    for i in range(1, NUM_OBJECTS+1):
        name = f"Object_{i}"
        sendTfMessage(scene_objects['objects'][name], name) 

def joint_state_callback(object):
    for i in range(len(object['name'])):
        handle = sim.getObjectHandle(object['name'][i])
        sim.setJointPosition(handle, object['position'][i])

        # adjust finger joints
        if object['name'][i] == "panda_finger_joint1":
            handle = sim.getObjectHandle("panda_finger_joint2")
            sim.setJointPosition(handle, object['position'][i])

# for adding a noise when placing objects on the table
def get_noisy_position(position, noise=0.05):
    x, y, z = position
    x = x + random.uniform(-noise, +noise)
    y = y + random.uniform(-noise, +noise)

    return [x, y, z]


def place_scene_objects():
    global scene_objects

    scene_objects = {
        "objects": {},
        "containers": {},
        "obstacles": {},
        "tables": {}
    }

    # Add tables to scene_objects
    table0 = sim.getObject(f"/Table[0]")
    table1 = sim.getObject(f"/Table[1]")

    scene_objects["tables"]["Table_1"] = table0
    scene_objects["tables"]["Table_2"] = table1

    # Create raster for possible object locations
    raster_size = 0.2

    # Possible positions for table 0
    possible_positions_t0 = []

    for x in np.arange(-0.3, 0.5, raster_size+0.1):
        for y in np.arange(-0.1, 0.3, raster_size):
            possible_positions_t0.append([x, y, 0.1])

    actual_positions_t0 = random.sample(
        possible_positions_t0, NUM_OBJECTS+int(NUM_OBSTACLES/2))
    sample_number_t0 = 0

    # Possible positions for table 1
    possible_positions_t1 = []

    for x in np.arange(-0.3, 0.5, raster_size+0.1):
        for y in np.arange(-0.15, 0.15, raster_size):
            possible_positions_t1.append([x, y, 0.1])

    actual_positions_t1 = random.sample(
        possible_positions_t1, NUM_CONTAINERS+int(NUM_OBSTACLES/2))
    sample_number_t1 = 0

    # Randomly place objects
    for i in range(1, NUM_OBJECTS+1):
        object_name = f"Object_{i}"
        object = sim.getObject(f"/{object_name}")
        scene_objects["objects"][object_name] = object

        position = get_noisy_position(actual_positions_t0[sample_number_t0])

        sim.setObjectPosition(object, table0, position)
        
        #randomize orientation for the cubes
        rand_orientation = random.uniform(0,2*np.pi)
        sim.setObjectOrientation(object, table0, [0,0,rand_orientation])
        
        print(f"Placed object {i} at {position} w.r.t. Table[0]")
        sample_number_t0 += 1

    # Randomly place containers
    for i in range(1, NUM_CONTAINERS+1):
        container_name = f"Container_{i}"
        container = sim.getObject(f"/{container_name}")
        scene_objects["containers"][container_name] = container

        position = get_noisy_position(actual_positions_t1[sample_number_t1], 0.02)

        sim.setObjectPosition(container, table1, position)
        print(f"Placed container {i} at {position} w.r.t. Table[1]")
        sample_number_t1 += 1

    # Randomly place obstacles
    for i in range(1, NUM_OBSTACLES+1):
        obstacle_name = f"Obstacle_{i}"
        obstacle = sim.getObject(f"/{obstacle_name}")
        table_name = "Table[0]" if (i % 2 == 0) else "Table[1]"
        table = table0 if (i % 2 == 0) else table1
        sample_number = sample_number_t0 if (i % 2 == 0) else sample_number_t1
        actual_positions = actual_positions_t0 if (
            i % 2 == 0) else actual_positions_t1
        scene_objects["obstacles"][obstacle_name] = obstacle

        position = get_noisy_position(actual_positions[sample_number])

        sim.setObjectPosition(obstacle, table, position)
        print(f"Placed obstacle {i} at {position} w.r.t. {table_name}")

        if (i % 2 == 0):
            sample_number_t0 += 1
        else:
            sample_number_t1 += 1

def sysCall_actuation():
    pass

def sysCall_sensing():
    pass

def sysCall_cleanup():
    pass