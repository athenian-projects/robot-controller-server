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

    def setLinear(self, val):
        if (val < -0.5):
            self._linear = -0.5
        elif (val > 0.5):
            self._linear = 0.5
        else:
            self._linear = val

    def setAngular(self, val):
        if (val < -0.5):
            self._angular = -0.5
        elif (val > 0.5):
            self._angular = 0.5
        else:
            self._angular = val

    def forward(self):
        self._linear = min(self._linear + 0.1, 0.5)

    def backward(self):
        self._linear = max(self._linear - 0.1, -0.5)

    def left(self):
        self._angular = min(self._angular + 0.1, 0.5)

    def right(self):
        self._angular = max(self._angular - 0.1, -0.5)

    def stop(self):
        self._linear = 0.0
        self._angular = 0.0

    def twist_msg(self):
        return new_twist(self.linear, self.angular)

    def json(self):
        return jsonify({'linear': round(self.linear, 1), 'angular': round(self.angular, 1)})
