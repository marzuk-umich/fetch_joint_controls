#!/bin/bash

set -e  # Exit immediately if any command fails

# --- Step 1: Go to folder ---
cd ~/fetch_images/rgd_depth

# --- Step 2: Clean old images ---
echo "🧹 Cleaning old 'depth*' and 'rgb*' files..."
rm -f depth*.png rgb*.png depth*.npy

# --- Step 3: Capture new RGB and Depth ---
echo "📸 Capturing new RGB and Depth images..."
rosrun fetch_joint_controls save_rgb_depth.py &
# --- Step 4: Wait for capture to complete ---
sleep 3

# --- Step 5: Find latest saved files (BEFORE the newest one) ---
echo " Finding latest rgb and depth images..."
# List all matching files, sorted by time, and pick the second-last one (before newest)
RGB_FILE=$(ls -t rgb*.png | head -n 1)
DEPTH_PNG_FILE=$(ls -t depth*.png | head -n 1)
DEPTH_NPY_FILE=$(ls -t depth*.npy | head -n 1)
# --- Step 6: Rename them to standard names ---
echo " Renaming to image.png, depth.png, depth.npy..."
cp "$RGB_FILE" image.png
cp "$DEPTH_PNG_FILE" depth.png
cp "$DEPTH_NPY_FILE" depth.npy

# --- Step 7: SCP files to remote machine ---
REMOTE_USER=marzukkp
REMOTE_IP=141.212.52.139
CONTAINER_ID=4d08d867ff9c
REMOTE_PATH=/home/marzukkp/fetch_grasp/TransCG_test

echo "🚚 Copying files to remote machine $REMOTE_IP..."
scp ~/fetch_images/rgd_depth/image.png ~/fetch_images/rgd_depth/depth.png ${REMOTE_USER}@${REMOTE_IP}:${REMOTE_PATH}/

# --- Step 8: Copy files into Docker container ---
echo "🐳 Copying files into Docker container..."
ssh ${REMOTE_USER}@${REMOTE_IP} "docker cp ${REMOTE_PATH}/image.png ${CONTAINER_ID}:/root/TransCG/image/test_image/image.png"
ssh ${REMOTE_USER}@${REMOTE_IP} "docker cp ${REMOTE_PATH}/depth.png ${CONTAINER_ID}:/root/TransCG/image/test_image/depth.png"

# --- Step 9: Run TransCG Inference ---
echo "⚡ Running TransCG Depth Completion..."
ssh ${REMOTE_USER}@${REMOTE_IP} "docker exec -i ${CONTAINER_ID} python3 /root/TransCG/new_inference.py"

# --- Step 10: Create Refined PCD ---
echo "📦 Creating refined point cloud..."
ssh ${REMOTE_USER}@${REMOTE_IP} "docker exec -i ${CONTAINER_ID} python3 /root/TransCG/create_refined_pcd.py"

# --- Step 11: Run Grasp Detection ---
echo "🤖 Running grasp detection inside Docker container..."
GRASP_SCRIPT_PATH=/root/anygrasp_sdk/grasp_detection/demo_with_pcd.py  # <-- Update if different
CHECKPOINT_PATH=/root/anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar  # <-- Update with your checkpoint

ssh ${REMOTE_USER}@${REMOTE_IP} "docker exec -i ${CONTAINER_ID} python3 ${GRASP_SCRIPT_PATH} --checkpoint_path ${CHECKPOINT_PATH} --top_down_grasp --debug"

# --- Step 12: Final ---
sleep 3
echo "✅ Full Transparent Object Grasping Pipeline Completed!"
