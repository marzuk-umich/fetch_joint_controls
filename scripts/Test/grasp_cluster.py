import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R

# --- Load grasps ---
# Assume grasps.txt has: x, y, z, qx, qy, qz, qw
# And scores.txt has corresponding scores

grasps = np.loadtxt('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/Test/all_grasps.txt')  # (N, 7)
scores = np.loadtxt('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/Test/all_scores.txt')  # (N, )

assert grasps.shape[0] == scores.shape[0], "Grasps and scores size mismatch!"

# --- Feature for clustering ---
# You can cluster only on position, or position + orientation
# Here: use (x, y, z) for clustering

positions = grasps[:, :3]

# --- Clustering ---
# Parameters to tune:
eps = 0.05  # distance threshold (in meters) between grasps to be considered same cluster
min_samples = 2  # minimum grasps to form a cluster

clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(positions)

labels = clustering.labels_  # -1 means noise

unique_labels = set(labels) - {-1}  # ignore noise

print(f"Found {len(unique_labels)} clusters.")

# --- Select Best grasp per cluster ---
selected_grasps = []

for label in unique_labels:
    indices = np.where(labels == label)[0]
    cluster_grasps = grasps[indices]
    cluster_scores = scores[indices]

    best_idx = np.argmax(cluster_scores)
    best_grasp = cluster_grasps[best_idx]

    selected_grasps.append(best_grasp)

selected_grasps = np.array(selected_grasps)

# --- Save selected grasps ---
np.savetxt('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/Test/selected_grasps.txt', selected_grasps, fmt='%.8f')
print(f"✅ Saved {len(selected_grasps)} best grasps (one per cluster) to 'selected_grasps.txt'.")
