from flask import jsonify
from utils import new_twist

MAX_LINEAR = 1.0
MAX_ANGULAR = 1.0


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
        print("Linear: " + str(val))
        if (val < -MAX_LINEAR):
            self._linear = -MAX_LINEAR
        elif (val > MAX_LINEAR):
            self._linear = MAX_LINEAR
        else:
            self._linear = val

    def setAngular(self, val):
        print("Angular: " + str(val))
        if (val < -MAX_ANGULAR):
            self._angular = -MAX_ANGULAR
        elif (val > MAX_ANGULAR):
            self._angular = MAX_ANGULAR
        else:
            self._angular = val

    def forward(self):
        self._linear = min(self._linear + 0.1, MAX_LINEAR)

    def backward(self):
        self._linear = max(self._linear - 0.1, -MAX_LINEAR)

    def left(self):
        self._angular = min(self._angular + 0.1, MAX_ANGULAR)

    def right(self):
        self._angular = max(self._angular - 0.1, -MAX_ANGULAR)

    def stop(self):
        self._linear = 0.0
        self._angular = 0.0

    def twist_msg(self):
        return new_twist(self.linear, self.angular)

    def json(self):
        return jsonify({'linear': round(self.linear, 1), 'angular': round(self.angular, 1)})
