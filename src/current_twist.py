from flask import jsonify
from utils import new_twist


class CurrentTwist(object):
    def __init__(self):
        self._linear = 0.0
        self._angular = 0.0

    @property
    def linear(self):
        return self._linear

    @property
    def angular(self):
        return self._angular

    def forward(self):
        self._linear = min(self._linear + .1, .5)

    def backward(self):
        self._linear = max(self._linear - .1, -.5)

    def left(self):
        self._angular = min(self._angular + .1, .5)

    def right(self):
        self._angular = max(self._angular - .1, -.5)

    def stop(self):
        self._linear = 0
        self._angular = 0

    def twist_msg(self):
        return new_twist(self.linear, self.angular)

    def json(self):
        return jsonify({'linear': self.linear, 'angular': self.angular})
