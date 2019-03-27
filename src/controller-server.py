#!/usr/bin/env python

import argparse
import logging
import os
from threading import Thread

import rospy
from flask import Flask
from flask import Response
from flask import jsonify
from flask_httpauth import HTTPBasicAuth
from geometry_msgs.msg import Twist
from utils import new_twist
from utils import setup_logging

PORT = 'port'
LOG_LEVEL = 'loglevel'

logger = logging.getLogger(__name__)

# Initialize node
rospy.init_node('controller-server')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)

auth = HTTPBasicAuth()
http = Flask(__name__)


class CurrentTwist(object):
    def __init__(self):
        self.linear = 0.0
        self.angular = 0.0

    def forward(self):
        self.linear += .1

    def backward(self):
        self.linear -= .1

    def left(self):
        self.angular += .1

    def right(self):
        self.angular -= .1

    def stop(self):
        self.linear = 0
        self.angular = 0

    def twist_msg(self):
        return new_twist(self.linear, self.angular)


current_twist = CurrentTwist()


@http.route('/')
def root():
    return Response('Read the README page', mimetype='text/plain')


@http.route('/forward')
def forward():
    print("Publishing forward")
    current_twist.forward()
    return jsonify({'status': 'success'})


@http.route('/backward')
def backward():
    print("Publishing backward")
    current_twist.backward()
    return jsonify({'status': 'success'})


@http.route('/left')
def left():
    print("Publishing left")
    current_twist.left()
    return jsonify({'status': 'success'})


@http.route('/right')
def right():
    print("Publishing right")
    current_twist.right()
    return jsonify({'status': 'success'})


@http.route('/stop')
def stop():
    print("Publishing stop")
    current_twist.stop()
    return jsonify({'status': 'success'})


def publish():
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

    t = Thread(target=publish)
    t.start()

    http.run(debug=False, port=port, host='0.0.0.0')


if __name__ == "__main__":
    main()
