import yaml
import math
import numpy as np
import os

YAML_FILE = r"Downloads\object_poses.yaml"

def rpy_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return qx, qy, qz, qw

if not os.path.exists(YAML_FILE):
    with open(YAML_FILE, "w") as f:
        yaml.dump({"objects": []}, f)

def fetch(object_id):
    with open(YAML_FILE, "r") as f:
        data = yaml.safe_load(f)
    for obj in data.get("objects", []):
        if obj["object_id"] == object_id:
            return obj["pose"]
    return None

def update(object_id, new_pose):
    with open(YAML_FILE, "r") as f:
        data = yaml.safe_load(f)
    objects = data.get("objects", [])
    objects = [obj for obj in objects if obj["object_id"] != object_id]
    objects.append({"object_id": object_id, "pose": new_pose})
    with open(YAML_FILE, "w") as f:
        yaml.safe_dump({"objects": objects}, f)

def find_close_match(pose, object_pose_map):
    for obj_id, obj_pose in object_pose_map.items():
        dist = np.linalg.norm(np.array(pose[:3]) - np.array(obj_pose[:3]))
        if dist < 0.1:
            return obj_id
    return None

if __name__ == "__main__":
    with open(r"Downloads\input.yaml", "r") as file:
        data = yaml.safe_load(file)
    poses = data["Poses"]
    targets = data["targets"]
    pose_dict = {}
    for pose_name, pose_data in poses.items():
        position = (pose_data["x"], pose_data["y"], pose_data["z"])
        orientation = rpy_to_quaternion(pose_data["roll"], pose_data["pitch"], pose_data["yaw"])
        pose_dict[pose_name] = (*position, *orientation)
    stack = []
    object_pose_map = {}
    for target in targets:
        if target == "grasp":
            if stack:
                last_pose = stack.pop()
                object_id = find_close_match(last_pose, object_pose_map)
                if object_id:
                    print(f"Grasp detected. Close match found for {object_id}.")
                else:
                    print("Grasp detected. No close match found.")
        elif target == "release":
            if stack:
                new_pose = stack.pop()
                object_id = find_close_match(new_pose, object_pose_map)
                if object_id:
                    object_pose_map[object_id] = new_pose
                    update(object_id, {"position": new_pose[:3], "orientation": new_pose[3:]})
                    print(f"Release detected. Updated pose for {object_id}: {new_pose}.")
        elif target in pose_dict:
            stack.append(pose_dict[target])
        else:
            print(f"Unknown target: {target}")
    print("Final object poses:", object_pose_map)
