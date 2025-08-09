import matplotlib.pyplot as plt
import pandas as pd
import os

# Directory where your CSVs are saved
LOG_DIR = "/home/marzuk/catkin_ws/src/fetch_joint_controls/test"

# File mappings
FILES = {
    "Position": {
        "cmd": "commanded_all_joints.csv",
        "act": "actual_all_joints.csv"
    },
    "Velocity": {
        "cmd": "commanded_all_joints_velocity.csv",
        "act": "actual_all_joints_velocity.csv"
    },
    "Acceleration": {
        "cmd": "commanded_all_joints_accln.csv",
        "act": "actual_all_joints_accln.csv"
    },
    "FailFlag": "fail_flags.csv"
}                                                         

# Joint names
joint_labels = [f"Joint_{i}" for i in range(1,8)]


def plot_joint_data(cmd_file, act_file, title, ylabel):
    df_cmd = pd.read_csv(cmd_file)
    df_act = pd.read_csv(act_file)

    # Align timelines
    t_start = df_cmd["Time"].iloc[0]
    df_act = df_act[df_act["Time"] >= t_start]

    fig, axes = plt.subplots(4, 2, figsize=(14, 10), sharex=True)
    axes = axes.flatten()

    for i in range(7):
        joint = joint_labels[i]
        ax = axes[i]

        if joint in df_cmd.columns:
            cmd_series = pd.to_numeric(df_cmd[joint], errors='coerce')
            ax.plot(df_cmd["Time"].to_numpy(), cmd_series.to_numpy(), label="Commanded", color='tab:blue')

        if joint in df_act.columns:
            act_series = pd.to_numeric(df_act[joint], errors='coerce')
            ax.plot(df_act["Time"].to_numpy(), act_series.to_numpy(), label="Actual", color='tab:orange', linestyle='--')

        ax.set_title(joint)
        ax.set_ylabel(ylabel)
        ax.grid(True)
        ax.legend(loc="upper right")

    # Hide 8th subplot
    fig.delaxes(axes[-1])

    # Label time axis for bottom plots
    axes[-3].set_xlabel("Time (s)")
    axes[-2].set_xlabel("Time (s)")

    fig.suptitle(title)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


def plot_fail_flag(file_path):
    df = pd.read_csv(file_path)
    plt.figure(figsize=(10, 3))
    plt.step(df["Time"], df["FailFlag"], where='post', label="Fail Flag", color='red')
    plt.title("Fail Flag Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Fail Flag")
    plt.grid(True)
    plt.tight_layout()
    plt.legend()
    plt.show()


def main():
    # Position plot
    pos_cmd = os.path.join(LOG_DIR, FILES["Position"]["cmd"])
    pos_act = os.path.join(LOG_DIR, FILES["Position"]["act"])
    if os.path.exists(pos_cmd) and os.path.exists(pos_act):
        plot_joint_data(pos_cmd, pos_act, "Joint Position (Commanded vs Actual)", "Position (rad)")

    # Velocity plot
    vel_cmd = os.path.join(LOG_DIR, FILES["Velocity"]["cmd"])
    vel_act = os.path.join(LOG_DIR, FILES["Velocity"]["act"])
    if os.path.exists(vel_cmd) and os.path.exists(vel_act):
        plot_joint_data(vel_cmd, vel_act, "Joint Velocity (Commanded vs Actual)", "Velocity (rad/s)")

    # Acceleration plot
    acc_cmd = os.path.join(LOG_DIR, FILES["Acceleration"]["cmd"])
    acc_act = os.path.join(LOG_DIR, FILES["Acceleration"]["act"])
    if os.path.exists(acc_cmd) and os.path.exists(acc_act):
        plot_joint_data(acc_cmd, acc_act, "Joint Acceleration (Commanded vs Actual)", "Acceleration (rad/s²)")

    # Fail flags
    fail_file = os.path.join(LOG_DIR, FILES["FailFlag"])
    if os.path.exists(fail_file):
        plot_fail_flag(fail_file)


if __name__ == "__main__":
    main()
