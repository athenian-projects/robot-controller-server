# Robot Controller Server

```bash
rosrun robot-controller-server controller_server.py
```

Install the required python packages with:
```bash
$ pip2 install -r requirements.txt
```

Assign GOOGLE_APPLICATION_CREDENTIALS
```bash
GOOGLE_APPLICATION_CREDENTIALS=AthenianDemo-b41bb569b941.json
```

```bash
# On TurtleBot3
roscore

roslaunch turtlebot3_bringup turtlebot3_robot.launch

rosrun robot-controller-server controller_server.py
```