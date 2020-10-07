#!/usr/bin/env python
"""
Blueprint node for launching using xml launch files
Author: Shilpaj Bhalerao
Date: Oct 04, 2020
"""
import math
import random
import sys
import rospy
import tf

from std_msgs.msg import Int64
# from ROSAPI.Turtle import *


class Frame:
    """
    Class whose multiple instances has to be initiated
    """
    def __init__(self):
        # Initialize a node
        rospy.init_node('turtle', anonymous=False)

        # Log node status
        rospy.loginfo("Node Initialized")

        # Broadcaster
        self.broadcast = tf.TransformBroadcaster()

        # Variables for position and orientation of turtles
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0

        # Rate
        self.rate = rospy.Rate(10.0)

        # Object of a Turtle class to interact with turtles
        self.turtle = None

        # Spawn turtles randomly
        # self.random_spawn()

        # Initiate topic for instance and publisher to publish data
        topic_name = '/Node/topic'
        self.pub = rospy.Publisher(topic_name, Int64, queue_size=1)

        while not rospy.is_shutdown():
            # Broadcast tf
            self.dynamic_frame()

            # Publish data
            self.pub.publish()
            self.rate.sleep()

    # def random_spawn(self):
    #     """
    #     Method to spawn only a single instance of a turtle
    #     """
    #     self.turtle = Turtle()
    #
    #     self.rand_pos()
    #
    #     self.turtle.spawn(self.x_pos, self.y_pos, self.theta)
    #     print(self.turtle.get_name())

    # def random_spawn_test(self):
    #     """
    #     Method to spawn multiple instances of a turtle
    #     """
    #     collection = []
    #     for i in range(self._instance_number):
    #         collection.append(Turtle(i))
    #
    #         self.rand_pos()
    #
    #         collection[i].spawn(self.x_pos, self.y_pos, self.theta)
    #         print(collection[i].get_name())

    def rand_pos(self):
        """
        Method to set a random position and orientation of a turtle in a turtle-sim
        """
        self.x_pos = random.randint(0, 11)
        self.y_pos = random.randint(0, 11)
        self.theta = random.random()

    def dynamic_frame(self):
        """
        Method to broadcast the dynamic transform
        """
        time_now = rospy.Time.now().to_sec() * math.pi
        self.broadcast.sendTransform((2.0 * math.sin(time_now), 2.0 * math.cos(time_now), 0.0),
                                     (0.0, 0.0, 0.0, 1.0),
                                     rospy.Time.now(),
                                     "turtle",
                                     "world")


def main():
    """
    Main Function
    """
    try:
        # reset_sim()
        Frame()
    except KeyboardInterrupt:
        sys.exit()


if __name__ == '__main__':
    main()
