import matplotlib.pyplot as plt
import csv

def read_csv(filename):
    with open(filename, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        time = []
        joint_data = [[] for _ in range(len(header) - 1)]
        for row in reader:
            time.append(float(row[0]))
            for i in range(1, len(row)):
                joint_data[i - 1].append(float(row[i]))
    return time, joint_data

# Load commanded and actual data
cmd_time, cmd_joints = read_csv("/home/marzuk/catkin_ws/src/fetch_joint_controls/test/commanded_all_joints.csv")
act_time, act_joints = read_csv("/home/marzuk/catkin_ws/src/fetch_joint_controls/test/actual_all_joints.csv")

# Align start time
t0 = min(cmd_time[0], act_time[0])
cmd_time = [t - t0 for t in cmd_time]
act_time = [t - t0 for t in act_time]

# Plot all 7 joints
plt.figure(figsize=(14, 10))
for i in range(7):
    plt.subplot(4, 2, i + 1)
    plt.plot(cmd_time, cmd_joints[i], label="Commanded", linewidth=2)
    plt.plot(act_time, act_joints[i], label="Actual", linestyle='--')
    plt.title(f"Joint {i}")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
    plt.grid(True)
    plt.legend()

plt.tight_layout()
plt.show()
