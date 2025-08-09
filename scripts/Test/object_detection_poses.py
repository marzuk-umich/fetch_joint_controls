#!/usr/bin/env python3

import rospy
import tf
import json
import numpy as np
from geometry_msgs.msg import PoseStamped

def load_all_points(json_file_path):
    """Load object poses from a JSON file."""
    with open(json_file_path, 'r') as f:
        data = json.load(f)
    return data

def load_grasp_positions(grasp_file_path):
    """Load all grasp positions from a 7D pose file (XYZ + quaternion)."""
    grasps = []
    with open(grasp_file_path, 'r') as f:
        for line in f:
            parts = list(map(float, line.strip().split()))
            if len(parts) == 7:
                grasps.append(np.array(parts))  # ← Save all 7 values, not just xyz
    return grasps

def transform_point(x, y, z, source_frame='head_camera_rgb_optical_frame', target_frame='base_link'):
    """Transform a 3D point from source_frame to target_frame using tf."""
    listener = tf.TransformListener()
    rospy.sleep(1.0)

    pose = PoseStamped()
    pose.header.frame_id = source_frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0  # identity orientation

    listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
    transformed = listener.transformPose(target_frame, pose)

    pt = transformed.pose.position
    return np.array([pt.x, pt.y, pt.z])

def find_closest_grasp(point, grasp_list):
    """Find the closest grasp (by position only) to the transformed point."""
    distances = [np.linalg.norm(point - g[:3]) for g in grasp_list]  # only xyz used for distance
    idx = np.argmin(distances)
    return idx, distances[idx]

def main():
    rospy.init_node('closest_grasp_finder')

    # Step 1: Load all objects from JSON
    json_file = '/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/Test/all_points.json'
    all_objects = load_all_points(json_file)

    # Step 2: Print available objects
    print("\nAvailable objects to grasp:")
    for idx, obj in enumerate(all_objects, 1):
        print(f"{idx}: {obj['class']} (object_id {obj['object_id']})")

    # Step 3: Ask user for selection
    choice = int(input("\nEnter the number of the object you want to grasp (1-6): "))
    assert 1 <= choice <= len(all_objects), "Invalid choice."

    selected_object = all_objects[choice - 1]
    camera_point = selected_object['pose']

    print(f"\nSelected object: {selected_object['class']} at {camera_point}")

    # Step 4: Transform the selected camera_point
    transformed_point = transform_point(*camera_point)
    print(f"📍 Transformed point in base_link: {transformed_point}")

    # Step 5: Load all grasps
    grasp_file = '/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/fetch_grasps.txt'
    grasp_list = load_grasp_positions(grasp_file)

    # Step 6: Find closest grasp
    idx, dist = find_closest_grasp(transformed_point, grasp_list)

    # Step 7: Output result
    print("\n🎯 Closest Grasp:")
    print(f"Index: {idx + 1}")
    print(f"Grasp pose (xyz qx qy qz qw): {grasp_list[idx]}")
    print(f"Distance: {dist:.4f} meters")

    # Step 8: Save the full 7D pose (xyz + quaternion) to file
    output_path = '/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/object_detection_pose.txt'
    with open(output_path, 'w') as f:
        pose = grasp_list[idx]
        f.write(' '.join(map(str, pose)) + '\n')  # Save all 7 numbers
    print(f"\n✅ Saved selected grasp pose to {output_path}")

if __name__ == "__main__":
    main()
