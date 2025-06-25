#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def main():
    rospy.init_node('wheel_velocity_test_node')

    # Publishers to left and right wheel velocity controllers
    left_pub = rospy.Publisher('/balance_bot/left_shaft_velocity_controller/command', Float64, queue_size=10)
    right_pub = rospy.Publisher('/balance_bot/right_shaft_velocity_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    # Example: command 1.0 rad/s to both wheels for 5 seconds, then stop
    start_time = rospy.Time.now()
    duration = rospy.Duration(2)  # seconds

    while not rospy.is_shutdown():
        elapsed = rospy.Time.now() - start_time
        if elapsed < duration:
            left_pub.publish(1.0)   # set left wheel velocity to 1 rad/s
            right_pub.publish(1.0)  # set right wheel velocity to 1 rad/s
        else:
            left_pub.publish(0.0)   # stop left wheel
            right_pub.publish(0.0)  # stop right wheel
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
