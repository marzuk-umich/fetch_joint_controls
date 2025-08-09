#!/bin/bash

# --- Variables ---
LOCAL_DIR="/home/marzuk/fetch_images/pcds"
REMOTE_USER="marzukkp"
REMOTE_IP="141.212.52.139"
REMOTE_PATH="/home/marzukkp/fetch_grasp/Latest_marzukk/ "
CONTAINER_ID="4d08d867ff9c"
CONTAINER_DEST_PATH="/root/anygrasp_sdk/grasp_detection/scene.pcd"
GRASP_SCRIPT_PATH="/root/anygrasp_sdk/grasp_detection/demo_with_pcd.py"
CHECKPOINT_PATH="/root/anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar"
REMOTE_TMP_DIR="/home/marzukkp/fetch_grasp/anygrasp_results"
LOCAL_SAVE_DIR="/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/Test"
LOCAL_VIEW_SCRIPT="/home/marzuk/Project Files And Research/DeepRob/finalProject/test_data/view_pcd.py"

echo " Dont Forget Ctrl Shift F"
echo " Dont Forget Ctrl Shift F"

# --- Step 1: Change to target directory ---
cd "$LOCAL_DIR" || exit 1

# --- Step 2: Remove all files ---
echo " Cleaning up old files..."
rm -f *

# --- Step 3: Run rosrun command to capture PCD ---
echo " Capturing point cloud..."
rosrun pcl_ros pointcloud_to_pcd input:=/head_camera/depth_registered/points _prefix:=scene_ &

# --- Step 4: Wait for 3 seconds ---
sleep 5

# --- Step 5: Kill rosrun if still running ---
pkill -f pointcloud_to_pcd

# --- Step 6: Find captured PCD ---
second_latest_file=$(ls -t | sed -n '2p')

# --- Step 7: Rename it to scene.pcd ---
if [ -n "$second_latest_file" ]; then
    mv "$second_latest_file" scene.pcd
    echo " Renamed $second_latest_file to scene.pcd"
else
    echo " No point cloud file found!"
    exit 1
fi

# --- Step 8: View local scene.pcd ---
echo " Viewing captured scene locally..."
python3 "$LOCAL_VIEW_SCRIPT" "$LOCAL_DIR/scene.pcd"

# --- Step 8.5: Ask user if they want to continue ---
read -p "Do you want to continue uploading and running grasp detection? (y/n): " user_input

if [[ "$user_input" != "y" && "$user_input" != "Y" ]]; then
    echo " Exiting as per user choice."
    exit 0
fi

# --- Step 9: SCP scene.pcd to remote machine ---
echo "🚀 Copying scene.pcd to remote machine..."
scp scene.pcd $REMOTE_USER@$REMOTE_IP:$REMOTE_PATH

ssh -X marzukkp@141.212.52.139 "bash -lc 'source /home/marzukkp/fetch_grasp/Latest_marzukk/.venv/bin/activate && python3 /home/marzukkp/fetch_grasp/Latest_marzukk/sevenv.py --input_pcd /home/marzukkp/fetch_grasp/latest_marzukk/scene.pcd'"

scp marzukkp@141.212.52.139:/home/marzukkp/fetch_grasp/Latest_marzukk/color_analysis_results/all_points.json /home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/Test/all_points.json

# (Optional fetching results could be added here if you want later)
echo "Full pipeline completed!"
