#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Example Python node to publish on a specific topic.
"""

# Import required Python code.
import rospy

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

# Import custom message data and dynamic reconfigure variables.
from node_example.msg import NodeExampleData
from node_example.cfg import nodeExampleConfig as ConfigType

class NodeExample(object):
    '''
    Node example class.
    '''
    def __init__(self):
        # Get the private namespace parameters from the parameter server:
        # set from either command line or launch file.
        rate = rospy.get_param('~rate', 1.0)
        rospy.loginfo('rate = %f', rate)
        # Create a dynamic reconfigure server.
        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure_cb)
        # Create a publisher for our custom message.
        self.pub = rospy.Publisher('example', NodeExampleData, queue_size=10)
        # Initialize message variables.
        self.enable = rospy.get_param('~enable', True)
        # Define int_a and int_b also as private variables
        # such that they can be specified from the command line
        self.int_a = rospy.get_param('~a', 1)
        self.int_b = rospy.get_param('~b', 2)
        self.message = rospy.get_param('~message', 'hello')

        # Add ros log information display
        rospy.loginfo('a = %d', self.int_a)
        rospy.loginfo('b = %d', self.int_b)
        rospy.loginfo('message = %s', self.message)

        # Create a timer to go to a callback. This is more accurate than
        # sleeping for a specified time.
        rospy.Timer(rospy.Duration(1 / rate), self.timer_cb)

        # Allow ROS to go to all callbacks.
        rospy.spin()

    def timer_cb(self, _event):
        """
        Called at a specified interval. Publishes message.
        """
        if not self.enable:
            return

        # Set the message to publish as our custom message.
        msg = NodeExampleData()
        # Fill in custom message variables with values updated from dynamic
        # reconfigure server.
        # Change message expressions such that they can be also specified using
        # $ rosparam set /pytalker/a 100
        # $ rosparam set /pytalker/b 200
        # $ rosparam set /pytalker/message "Hi"
        msg.message = rospy.get_param('~message', self.message)
        msg.a = rospy.get_param('~a', self.int_a)
        msg.b = rospy.get_param('~b', self.int_b)

        # If changes are detected, display ros log informing the changes
        if self.message != msg.message:
            rospy.loginfo('new message = %s', msg.message)
        if self.int_a != msg.a:
            rospy.loginfo('new a = %d', msg.a)
        if self.int_b != msg.b:
            rospy.loginfo('new b = %d', msg.b)

        # Assign private variables to newly assigned values such that
        # the notification about change won't come up in every callback
        self.message = msg.message
        self.int_a = msg.a
        self.int_b = msg.b

        # Publish our custom message.
        self.pub.publish(msg)

    def reconfigure_cb(self, config, dummy):
        '''
        Create a callback function for the dynamic reconfigure server.
        '''
        # Fill in local variables with values received from dynamic reconfigure
        # clients (typically the GUI).
        self.message = config["message"]
        self.int_a = config["a"]
        self.int_b = config["b"]
        self.enable = config["enable"]
        # Return the new variables.
        return config

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pytalker')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        NodeExample()
    except rospy.ROSInterruptException:
        pass
