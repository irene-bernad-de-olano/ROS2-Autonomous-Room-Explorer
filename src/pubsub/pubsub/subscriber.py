#!/usr/bin/env python3


import rclpy

from std_msgs.msg import String


def main():

    rclpy.init()

    subscribernode = rclpy.create_node('myfirstsubscriber')

    subscribernode.create_subscription(String, 'myfirsttopic', mysubcallback, 10)

    try:

        rclpy.spin(subscribernode)

    except KeyboardInterrupt:

        pass

    subscribernode.destroy_node()

    rclpy.shutdown()


def mysubcallback(msg):

    print('You said: ', msg.data)


if __name__ == '__main__':

    main()