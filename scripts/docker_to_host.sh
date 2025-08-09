#!/bin/bash

# --- Define variables ---
REMOTE_USER="marzukkp"
REMOTE_IP="141.212.52.139"
CONTAINER_ID="4d08d867ff9c"
REMOTE_TMP_DIR="/home/marzukkp/fetch_grasp/anygrasp_results"
LOCAL_SAVE_DIR="/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/Test"
LOCAL_SCRIPT_PATH="/home/marzuk/Project Files And Research/DeepRob/finalProject/test_data/view_pcd.py"
echo " Dont Forget Ctrl Shift F"
echo " Dont Forget Ctrl Shift F"
# --- Step 1: Copy all_scores.txt from container to remote host ---
ssh $REMOTE_USER@$REMOTE_IP "docker cp $CONTAINER_ID:/root/anygrasp_sdk/grasp_detection/example_data/all_scores.txt $REMOTE_TMP_DIR/"

# --- Step 2: SCP all_scores.txt to local machine ---
scp $REMOTE_USER@$REMOTE_IP:$REMOTE_TMP_DIR/all_scores.txt $LOCAL_SAVE_DIR/

# --- Step 3: Copy all_grasps.txt from container to remote host ---
ssh $REMOTE_USER@$REMOTE_IP "docker cp $CONTAINER_ID:/root/anygrasp_sdk/grasp_detection/example_data/all_grasps.txt $REMOTE_TMP_DIR/"

# --- Step 4: SCP all_grasps.txt to local machine ---
scp $REMOTE_USER@$REMOTE_IP:$REMOTE_TMP_DIR/all_grasps.txt $LOCAL_SAVE_DIR/

# --- Step 5: Copy grasp_results.pcd from container to remote host ---
ssh $REMOTE_USER@$REMOTE_IP "docker cp $CONTAINER_ID:/root/anygrasp_sdk/grasp_detection/example_data/grasp_results.pcd $REMOTE_TMP_DIR/"

# --- Step 6: SCP grasp_results.pcd to local machine ---
scp $REMOTE_USER@$REMOTE_IP:$REMOTE_TMP_DIR/grasp_results.pcd $LOCAL_SAVE_DIR/

# --- Step 7: Run local Python visualization script ---
python3 "$LOCAL_SCRIPT_PATH" "$LOCAL_SAVE_DIR/grasp_results.pcd"

# --- Step 13: Set ROS environment variables ---
echo "🛠 Setting ROS environment variables..."
export ROS_MASTER_URI=http://fetch38:11311
export ROS_IP=10.42.43.103

# --- Step 14: Run grasp clustering script ---
echo " Running grasp clustering script..."
python3 /home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/Test/grasp_cluster.py


# --- Step 14.5: Copy selected grasps to fetch grasps file ---
echo " Copying selected grasps..."
cp /home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/Test/selected_grasps.txt /home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/fetch_grasps.txt


# --- Step 15: Start markers publisher ---
echo " Launching grasp markers in ROS..."
rosrun fetch_joint_controls markers.py &

# --- Step 16: Open RViz with custom config ---
echo " Opening RViz with custom DeepRob config..."
rosrun rviz rviz -d /home/marzuk/Music/deeprob.rviz &

echo "Full grasp pipeline is up and running!"


echo " All done!"

