import os
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

# --- User-defined grasp pose (position + quaternion) ---
grasp_pose = [-0.03295749, 0.12495121, 0.98400003,
              -0.34845210, -0.39651984, -0.56064764, 0.63798697]

# --- Scene and output paths ---
scene_path = "/home/marzuk/fetch_images/pcds/scene.pcd"  # Change if needed
output_path = "/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/Test/scene_with_custom_grasp.pcd"

def visualize_custom_grasp(scene_pcd_path, pose, save_path=None):
    # Load point cloud
    scene = o3d.io.read_point_cloud(scene_pcd_path)
    if len(scene.points) == 0:
        raise ValueError("Loaded point cloud is empty!")

    # Create a simple gripper (box)
    # Create a blue "gripper-like" shape using a stretched box
    gripper = o3d.geometry.TriangleMesh.create_box(width=0.06, height=0.015, depth=0.02)
    gripper.paint_uniform_color([0.0, 0.3, 1.0])  # Blue
    gripper.compute_vertex_normals()

    gripper.rotate(R.from_euler('z', 90, degrees=True).as_matrix(), center=(0, 0, 0))

    # Unpack pose
    position = np.array(pose[:3])
    quat = np.array(pose[3:])  # [qx, qy, qz, qw]
    rotation_matrix = R.from_quat(quat).as_matrix()

    # Build 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = position

    # Transform gripper
    gripper.transform(T)

    # Sample gripper mesh to point cloud
    gripper_pcd = gripper.sample_points_uniformly(number_of_points=300)

    # Merge and save/view
    merged = scene + gripper_pcd

    if save_path:
        o3d.io.write_point_cloud(save_path, merged)
        print(f"✅ Saved merged scene with custom grasp to {save_path}")

    # Visualize
    o3d.visualization.draw_geometries([scene, gripper], window_name="Scene with Custom Grasp")

if __name__ == "__main__":
    visualize_custom_grasp(scene_path, grasp_pose, output_path)
