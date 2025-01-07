import yaml

def update_object_poses(object_poses_file, update_data_file):
    with open(object_poses_file, 'r') as file:
        object_poses = yaml.safe_load(file)

    with open(update_data_file, 'r') as file:
        update_data = yaml.safe_load(file)

    for update in update_data['aruco_objects']:
        aruco_id = update['aruco_id']
        updated_pose = update['pose']

        for obj in object_poses['objects']:
            if obj['aruco_id'] == aruco_id:
                obj['pose'] = updated_pose  # Only update pose
                break

    with open(object_poses_file, 'w') as file:
        yaml.dump(object_poses, file, default_flow_style=False)

update_object_poses('object_poses.yaml', 'aruco_poses.yaml')
