import matplotlib.pyplot as plt
import csv

# Read commanded joint positions
cmd_time, cmd_pos = [], []
with open("/home/marzuk/catkin_ws/src/fetch_joint_controls/test/commanded_joint0.csv", "r") as f:
    reader = csv.reader(f)
    next(reader)  # skip header
    for row in reader:
        cmd_time.append(float(row[0]))
        cmd_pos.append(float(row[1]))

# Read actual joint positions
act_time, act_pos = [], []
with open("/home/marzuk/catkin_ws/src/fetch_joint_controls/test/actual_joint0.csv", "r") as f:
    reader = csv.reader(f)
    next(reader)  # skip header
    for row in reader:
        act_time.append(float(row[0]))
        act_pos.append(float(row[1]))

# Normalize time to start at zero
t0 = min(cmd_time[0], act_time[0])
cmd_time = [t - t0 for t in cmd_time]
act_time = [t - t0 for t in act_time]

# Plot
plt.figure(figsize=(10, 5))
plt.plot(cmd_time, cmd_pos, label="Commanded", linewidth=2)
plt.plot(act_time, act_pos, label="Actual", linewidth=2, linestyle='--')
plt.xlabel("Time (s)")
plt.ylabel("Joint 0 Position (rad)")
plt.title("Joint 0: Commanded vs Actual Trajectory")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
