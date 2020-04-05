#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def talker():
    rospy.sleep(10)
    rospy.init_node('joint_trajectory_publisher', anonymous=True)
    pub = rospy.Publisher('position_joint_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(0.5)

    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
    msg.points = [JointTrajectoryPoint() for i in range(1)]
    msg.points[0].positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    msg.points[0].time_from_start = rospy.Time(1.0)

    pub.publish(msg)
    rospy.sleep(0.5)

if __name__ == '__main__':
    talker()
