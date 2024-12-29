import pandas as pd

CSV_FILE = r"Downloads\object_poses.csv"


def fetch(object_id):
    """
    Fetch the pose of the object by its ID.

    Args:
        object_id (str): The unique ID of the object.

    Returns:
        tuple: The pose of the object as a 7-tuple (x, y, z, qx, qy, qz, qw), or None if not found.
    """
    df = pd.read_csv(CSV_FILE)
    obj_row = df[df['object_id'] == object_id]

    if not obj_row.empty:
        pose = tuple(obj_row.iloc[0, 1:].values)
        return pose
    return None

def update(object_id, new_pose):
    """
    Update the pose of the object by its ID. If the object ID does not exist, it will be added.

    Args:
        object_id (str): The unique ID of the object.
        new_pose (tuple): The new pose as a 7-tuple (x, y, z, qx, qy, qz, qw).
    """
    df = pd.read_csv(CSV_FILE)

    df = df[df['object_id'] != object_id]

    new_row = pd.DataFrame([[object_id] + list(new_pose)], columns=df.columns)
    df = pd.concat([df, new_row], ignore_index=True)

    df.to_csv(CSV_FILE, index=False)

print(fetch('o3'))
update('o3',(1,2,3,4,5,6,7))