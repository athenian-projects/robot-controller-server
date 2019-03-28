#!/usr/bin/env python

import argparse
import logging
import os
from threading import Thread

import rospy
from current_twist import CurrentTwist
from flask import Flask
from flask import Response
from flask_httpauth import HTTPBasicAuth
from geometry_msgs.msg import Twist
from utils import setup_logging

PORT = 'port'
LOG_LEVEL = 'loglevel'

logger = logging.getLogger(__name__)

# Initialize node in main thread
rospy.init_node('controller_server')

current_twist = CurrentTwist()

auth = HTTPBasicAuth()
http = Flask(__name__)


@http.route('/')
def root():
    return Response('Read the README page', mimetype='text/plain')


@http.route('/linear/<float:val>', methods=['GET'])
def linear(val):
    print("Publishing linear")
    current_twist.setLinear(val)
    return current_twist.json()


@http.route('/angular/<float:val>', methods=['GET'])
def angular(val):
    print("Publishing angular")
    current_twist.setAngular(val)
    return current_twist.json()


@http.route('/forward')
def forward():
    print("Publishing forward")
    current_twist.forward()
    return current_twist.json()


@http.route('/backward')
def backward():
    print("Publishing backward")
    current_twist.backward()
    return current_twist.json()


@http.route('/left')
def left():
    print("Publishing left")
    current_twist.left()
    return current_twist.json()


@http.route('/right')
def right():
    print("Publishing right")
    current_twist.right()
    return current_twist.json()


@http.route('/stop')
def stop():
    print("Publishing stop")
    current_twist.stop()
    return current_twist.json()


def publish():
    # Setup publisher and rate
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(current_twist.twist_msg())
        rate.sleep()


def main():
    # Parse CLI args
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--port', dest=PORT, default=8080, help='HTTP port [8080]')
    parser.add_argument('-v', '--verbose', dest=LOG_LEVEL, default=logging.INFO, action='store_const',
                        const=logging.DEBUG, help='Enable debugging info')
    args = vars(parser.parse_args())

    # Setup logging
    setup_logging(level=args[LOG_LEVEL])

    port = int(os.environ.get('PORT', args[PORT]))
    logger.info("Starting customer server listening on port {}".format(port))
    print("Starting customer server listening on port {}".format(port))

    Thread(target=publish).start()

    http.run(debug=False, port=port, host='0.0.0.0')


if __name__ == "__main__":
    main()
