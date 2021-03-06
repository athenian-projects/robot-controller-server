#!/usr/bin/env python

import argparse
import logging
import os

import rospy
from current_twist import CurrentTwist
from flask import Flask
from flask import Response
from flask import request
from flask_httpauth import HTTPBasicAuth
from geometry_msgs.msg import Twist
from utils import setup_logging

PORT = 'port'
LOG_LEVEL = 'loglevel'
PULSE = 'pulse'

logger = logging.getLogger(__name__)

# Initialize node in main thread
rospy.init_node('controller_server')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

current_twist = CurrentTwist()

auth = HTTPBasicAuth()
http = Flask(__name__)


@http.route('/')
def root():
    return Response('Read the README page', mimetype='text/plain')


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


@http.route('/linear')
def linear():
    val = request.args.get('val')
    print("Publishing linear: " + val)
    current_twist.setLinear(float(val))
    return current_twist.json()


@http.route('/angular')
def angular():
    val = request.args.get('val')
    print("Publishing angular: " + val)
    current_twist.setAngular(float(val))
    return current_twist.json()


@http.route('/dual')
def dual():
    linear = request.args.get('linear')
    angular = request.args.get('angular')
    print("Publishing dual: " + linear + " " + angular)
    current_twist.setLinear(float(linear))
    current_twist.setAngular(float(angular))
    return current_twist.json()


@http.route('/pulse')
def pulse():
    linear = request.args.get('linear')
    angular = request.args.get('angular')
    print("Publishing pulse: " + linear + " " + angular)
    current_twist.setLinear(float(linear))
    current_twist.setAngular(float(angular))
    pub.publish(current_twist.twist_msg())
    return current_twist.json()


def publish():
    # Setup publisher and rate
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
    parser.add_argument('--pulse', dest=PULSE, default=False, action="store_true", help='Enable pulse of twist values')
    args = vars(parser.parse_args())

    # Setup logging
    setup_logging(level=args[LOG_LEVEL])

    port = int(os.environ.get('PORT', args[PORT]))
    logger.info("Starting ROS controller server listening on port {}".format(port))
    print("Starting ROS controller server listening on port {}".format(port))

    # if (not args[PULSE]):
    #    Thread(target=publish).start()

    http.run(debug=False, port=port, host='0.0.0.0')


if __name__ == "__main__":
    main()
