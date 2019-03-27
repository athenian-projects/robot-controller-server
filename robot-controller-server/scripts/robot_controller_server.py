import argparse
import logging
import os

import rospy
from flask import Flask
from flask import Response
from flask import jsonify
from flask_httpauth import HTTPBasicAuth
from geometry_msgs.msg import Twist
from scripts.utils import new_twist
from scripts.utils import setup_logging

PORT = 'port'
LOG_LEVEL = 'loglevel'

logger = logging.getLogger(__name__)

# Initialize node
rospy.init_node('robot_controller_server')
pub = rospy.Publisher('counter', Twist, queue_size=10)

auth = HTTPBasicAuth()
http = Flask(__name__)


@http.route('/')
def root():
    return Response('Read the README page', mimetype='text/plain')


@http.route('/forward')
def forward():
    print("Publishing Twist Value")
    t = new_twist(0.5, 0.0)
    pub.publish(t)
    return jsonify({'status': 'success'})


@http.route('/backward')
def backward():
    print("Publishing Twist Value")
    t = new_twist(-0.5, 0.0)
    pub.publish(t)
    return jsonify({'status': 'success'})


@http.route('/left')
def left():
    print("Publishing Twist Value")
    t = new_twist(0.0, 0.5)
    pub.publish(t)
    return jsonify({'status': 'success'})


@http.route('/right')
def right():
    print("Publishing Twist Value")
    t = new_twist(0.0, -0.5)
    pub.publish(t)
    return jsonify({'status': 'success'})


@http.route('/stop')
def stop():
    print("Publishing Twist Value")
    t = new_twist(0.0, 0.0)
    pub.publish(t)
    return jsonify({'status': 'success'})


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

    http.run(debug=False, port=port, host='0.0.0.0')


if __name__ == "__main__":
    main()
