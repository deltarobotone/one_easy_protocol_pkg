# one_easy_protocol_pkg

ROS package to control Delta-Robot One (one-easy-protocol bridge) ![CI](https://github.com/deltarobotone/one_easy_protocol_pkg/workflows/CI/badge.svg?branch=master) [![Codacy Badge](https://app.codacy.com/project/badge/Grade/4401410029564e428891267ce456f202)](https://www.codacy.com/gh/deltarobotone/one_easy_protocol_pkg?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=deltarobotone/one_easy_protocol_pkg&amp;utm_campaign=Badge_Grade) [![Total alerts](https://img.shields.io/lgtm/alerts/g/deltarobotone/one_easy_protocol_pkg.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/deltarobotone/one_easy_protocol_pkg/alerts/) [![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/deltarobotone/one_easy_protocol_pkg.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/deltarobotone/one_easy_protocol_pkg/context:python)

## Nodes

### one_ctrl_node.py

ROS node provides services to control Delta-Robot One using [one-easy-protocol](https://github.com/deltarobotone/one-easy-protocol)

#### Advertised Services

##### ctrl_robot_connect (one_easy_protocol_pkg/RobotConnect)

Connect robot. This service uses one-easy-protocol find_robot() function to scan all ports for a connected robot. If a robot is available it will be connected.

##### ctrl_robot_disconnect (one_easy_protocol_pkg/RobotDisconnect)

Disconnect robot.

##### ctrl_robot_extmotor (one_easy_protocol_pkg/RobotExtMotor)

Enable or disable robot external motor control via enable and choose velocity via speed (0-255).

##### ctrl_robot_gripper (one_easy_protocol_pkg/RobotGripper)

Enable or disable robot gripper via enable.

##### ctrl_robot_light (one_easy_protocol_pkg/RobotLight)

Enable robot leds with given colours RED, GREEN, BLUE, YELLOW, MAGENTA, CYAN, WHITE for light and choose intensity (0-255). Disable via light -> OFF.

##### ctrl_robot_move (one_easy_protocol_pkg/RobotMove)

Move robot to position x,y,z (mm) with a velocity v (0-100%).

