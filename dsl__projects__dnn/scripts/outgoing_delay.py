#!/usr/bin/env python2

"""
# Written By: Felix Berkenkamp
# Original version from: Adrian Esser and David Wu

The DelayNode class takes in messages and outputs delayed messages.

When executing this file as a node, all incoming commands sent from the DSL
controller are delayed. The node stores sent commands in a queue and
publishes them with the delay specified in the quadrocopter_classes.py file.

When running this node:

SUBSCRIBED TOPICS
/cmd_vel

PUBLISHED TOPICS
/cmd_vel_delayed
"""

from __future__ import print_function, division

import rospy
import Queue

from dsl__projects__dnn import Parameters
from geometry_msgs.msg import Twist


__all__ = ['DelayNode']


class DelayNode:
    """
    Delays messages by a certain number of samples

    Parameters:
    -----------
    delay: integer
        The number of samples that a message is delayed
    in_topic: string
        The name of the topic we subscribe to
    out_topic: string
        The name of the topic we publish to
    msg_class: class
        The message class
    queue_size: integer (optional, defaults to 0)
        The queue size for the publisher
    """
    def __init__(self, delay, in_topic, out_topic, msg_class, queue_size=0):

        # Time [s] that represents the comp->quad delay
        self.delay = delay

        # List to store all of the incoming commands
        self.queue = Queue.Queue(maxsize=200)

        # pass on the command
        self.publisher = rospy.Publisher(out_topic, msg_class,
                                         queue_size=queue_size)

        # The accuracy in seconds up to which we can reasonably work
        self.accuracy = 100 * 1e-6

        if self.delay <= self.accuracy:
            # Without delay, just pass on the message
            self.subscriber = rospy.Subscriber(in_topic, msg_class,
                                               self.pass_on_command)
        else:
            # With delay, add message to queue
            self.subscriber = rospy.Subscriber(in_topic, msg_class,
                                               self.read_in_command)

            # Keep resending the messages with delay
            self.keep_resending()

    def read_in_command(self, msg):
        """Adds incoming messages to the queue."""
        # Stamp the incoming command with the ROS time
        self.queue.put([rospy.get_time(), msg])

    def pass_on_command(self, msg):
        """Passes on the message directly."""
        self.publisher.publish(msg)

    def keep_resending(self):
        """Gets msg from queue and sends it with the defined delay."""

        # Keep doing this forever
        while not rospy.is_shutdown():
            # Get the last item, wait if there are none
            try:
                msg = self.queue.get(timeout=0.3)
            except Queue.Empty:
                continue

            # Compute the remaining delay time
            delta = msg[0] + self.delay - rospy.get_time()
            if delta > self.accuracy:
                # Wait until the delay has passed
                rospy.sleep(delta)

            # Send the next message
            self.publisher.publish(msg[1])


if __name__ == "__main__":
    rospy.init_node('outgoing_delay')

    delay = Parameters().outgoing_delay * 1e-6
    delay_node = DelayNode(delay, 'cmd_vel', 'cmd_vel_delayed', Twist)

    rospy.spin()
