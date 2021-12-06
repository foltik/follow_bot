#!/usr/bin/env python

import rospy
import tf
import numpy as np

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

class Follower:

    def __init__(self):
        rospy.init_node('follower')

        self.pub = rospy.Publisher("/follow_bot/ackermann_steering_controller/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, lambda msg: self.callback(msg))

        self.ready = False
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.ready:
                self.tick()
            rate.sleep()

    def tick(self):
        PI = np.pi

        # Get shortest angle to target
        delta = self.target_angle - self.angle
        delta = delta + 2*PI if delta < -PI else delta
        delta = delta - 2*PI if delta > PI else delta

        # Clamp angle and send steering input
        cmd = Twist()
        cmd.angular.z = max(-np.pi / 2, min(np.pi / 2, delta))

        # Move towards target until closer than 1 meter
        if self.dist > 1:
            cmd.linear.x = 1

        self.pub.publish(cmd)

    def callback(self, msg):
        if len(msg.pose) != 3:
            return

        self.ready = True
        steer = msg.pose[1]
        follow = msg.pose[2]

        # Get follower robot angle
        o = follow.orientation
        quat = (o.x, o.y, o.z, o.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        self.angle = euler[2]

        steer_pos = np.array([steer.position.x, steer.position.y])
        follow_pos = np.array([follow.position.x, follow.position.y])

        # Get target robot distance and angle
        delta = (steer_pos - follow_pos)
        self.dist = np.linalg.norm(delta)
        unit = delta / self.dist
        self.target_angle = np.arctan2(unit[1], unit[0])


if __name__ == '__main__':
    Follower()
