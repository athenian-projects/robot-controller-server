# Robot Controller Server

## Setup
Install the required python packages with:
```bash
$ pip2 install -r requirements.txt
```

## Magni Robot
```bash
rosrun robot-controller-server controller_server.py
```


## TurtleBot3
```bash
roscore
roslaunch turtlebot3_bringup turtlebot3_robot.launch
rosrun robot-controller-server controller_server.py
```

## Voice Controller Setup
Assign GOOGLE_APPLICATION_CREDENTIALS
```bash
GOOGLE_APPLICATION_CREDENTIALS=AthenianDemo-b41bb569b941.json
```
