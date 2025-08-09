#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

# (linear_x [m/s], angular_z [rad/s]) — each held for exactly 0.5 s
VELS = [
    (1.25000, -1.04720),
    (1.25000, -0.18187),
    (1.25000, -0.00068),
    (1.25000,  0.50774),
    (1.25000,  0.53188),
    (1.25000,  0.21578),
    (0.98109,  0.02367),
    (0.30444,  0.02052),
    (0.08905, -0.04338),
    (0.01181,  0.30546),
]

def main():
    rospy.init_node("fetch_velocity_sequence_discrete")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # each step lasts exactly dt seconds
    dt = rospy.Duration.from_sec(0.5)
    # republish during each step (controller expects continuous cmd_vel)
    hold_rate = rospy.Rate(20)  # 20 Hz

    rospy.sleep(0.5)  # let pub connect
    twist = Twist()
    rospy.sleep(10) 

    for i, (lin, ang) in enumerate(VELS):
        start = rospy.Time.now()
        end = start + dt
        twist.linear.x = lin
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = twist.angular.y = 0.0
        twist.angular.z = ang

        rospy.loginfo("Step %d/%d: v=%.5f m/s, w=%.5f rad/s",
                      i+1, len(VELS), lin, ang)

        # keep publishing the same command until 0.5 s elapses
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            pub.publish(twist)
            hold_rate.sleep()

    # hard stop at the end (publish a few times)
    twist.linear.x = twist.angular.z = 0.0
    for _ in range(5):
        pub.publish(twist)
        hold_rate.sleep()
    rospy.loginfo("Sequence complete, robot stopped.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
