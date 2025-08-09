import open3d as o3d
import numpy as np
import os
import argparse

def rotate_point_cloud(input_pcd_path, output_pcd_path, angle_degrees):
    """
    Rotates a point cloud about the X-axis by a given angle and saves it.
    """

    # 1. Load point cloud
    print(f"Loading point cloud from {input_pcd_path}...")
    pcd = o3d.io.read_point_cloud(input_pcd_path)
    
    # 2. Define rotation matrix around X-axis
    angle_radians = np.deg2rad(angle_degrees)
    R = np.array([
        [1, 0, 0],
        [0, np.cos(angle_radians), -np.sin(angle_radians)],
        [0, np.sin(angle_radians),  np.cos(angle_radians)]
    ])

    # 3. Apply rotation around the center
    center = pcd.get_center()
    print(f"Rotating around center: {center} by {angle_degrees} degrees about X-axis")
    pcd.rotate(R, center=center)

    # 4. Save rotated point cloud
    os.makedirs(os.path.dirname(output_pcd_path), exist_ok=True)
    o3d.io.write_point_cloud(output_pcd_path, pcd, write_ascii=True)
    print(f"✅ Saved rotated point cloud to {output_pcd_path}")

    # 5. (Optional) Visualize
    o3d.visualization.draw_geometries([pcd])

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_pcd', type=str, required=True, help='Path to input PCD or PLY file')
    parser.add_argument('--output_pcd', type=str, required=True, help='Path to save rotated PCD or PLY file')
    parser.add_argument('--angle', type=float, default=90.0, help='Rotation angle in degrees (default=90)')

    args = parser.parse_args()

    rotate_point_cloud(args.input_pcd, args.output_pcd, args.angle)
