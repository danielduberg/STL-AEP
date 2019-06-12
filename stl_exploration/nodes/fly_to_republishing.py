#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

last_pose = PoseStamped()

def callback(new_pose):
    global last_pose
    last_pose = new_pose

def fly_to():
    global last_pose
    pub = rospy.Publisher(
        "/mavros/setpoint_position/local", PoseStamped, queue_size=100)
    rospy.init_node("fly_to_republish", anonymous=True)

    sub = rospy.Subscriber("/fly_to_pose", PoseStamped, callback)

    rate = rospy.Rate(20)
    while True:
        last_pose.header.stamp = rospy.Time.now()
        pub.publish(last_pose)

        rate.sleep()


if __name__ == '__main__':
    try:
        fly_to()
    except rospy.ROSInterruptException:
        pass
