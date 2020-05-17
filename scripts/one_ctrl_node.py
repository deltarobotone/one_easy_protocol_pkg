#!/usr/bin/env python3

from one_easy_protocol_pkg.srv import *
from easyprotocol import *
import rospy

class OneCtrlNode:

    def __init__(self):
        self.__move_srv = rospy.Service('ctrl_robot_move', RobotMove, self.__moveCB)
        self.__light_srv = rospy.Service('ctrl_robot_light', RobotLight, self.__lightCB)
        self.__extmotor_srv = rospy.Service('ctrl_robot_extmotor', RobotExtMotor, self.__extmotorCB)
        self.__gripper_srv = rospy.Service('ctrl_robot_gripper', RobotGripper, self.__gripperCB)
        self.__connect_srv = rospy.Service('ctrl_robot_connect', RobotConnect, self.__connectCB)
        self.__disconnect_srv = rospy.Service('ctrl_robot_disconnect', RobotDisconnect, self.__disconnectCB)
        self.__connected = False
        self.__motorstate = False
        self.__robot = EasyProtocol()
        self.__colour = Colour()
        return None
    
    #Connect
    def __connectCB(self,req):
        self.__robot.findRobot()
        self.__robot.start()
        self.__connected = True
        return RobotConnectResponse("connected")

    #Disonnect
    def __disconnectCB(self,req):
        self.__robot.stop()
        self.__connected = False
        return RobotDisconnectResponse("disconnected")

    #Move
    def __moveCB(self,req):
        if self.__connected == False:
            return RobotMoveResponse("failed")

        self.__robot.move.ptp(req.x,req.y,req.z,req.v)

        return RobotMoveResponse("ready")

    #Light
    def __lightCB(self,req):
        if self.__connected == False:
            return RobotLightResponse("failed")

        if req.light == req.RED: self.__robot.light.setColour(self.__colour.red,req.intensity)

        elif req.light == req.BLUE: self.__robot.light.setColour(self.__colour.blue,req.intensity)

        elif req.light == req.GREEN: self.__robot.light.setColour(self.__colour.green,req.intensity)

        elif req.light == req.YELLOW: self.__robot.light.setColour(self.__colour.yellow,req.intensity)

        elif req.light == req.MAGENTA: self.__robot.light.setColour(self.__colour.magenta,req.intensity)

        elif req.light == req.CYAN: self.__robot.light.setColour(self.__colour.cyan,req.intensity)

        elif req.light == req.WHITE: self.__robot.light.setColour(self.__colour.white,req.intensity)

        elif req.light == req.OFF: self.__robot.light.off()

        return RobotLightResponse("ready")

    #ExtMotor
    def __extmotorCB(self,req):
        if self.__connected == False:
            return RobotExtMotorResponse("failed")

        if req.enable == True:
            if self.__motorstate == False:
                self.__robot.extmotor.start(req.speed)
                self.__motorstate = True
            else: self.__robot.extmotor.setSpeed(req.speed)
        else:
            self.__robot.extmotor.stop()
            self.__motorstate = False

        return RobotExtMotorResponse("ready")

    # Gripper
    def __gripperCB(self,req):
        if self.__connected == False:
            return RobotGripperResponse("failed")

        if req.enable == True:
            self.__robot.gripper.close()
        else:
            self.__robot.gripper.open()

        return RobotGripperResponse("ready")

    def run(self):
        rospy.init_node("one_ctrl_node")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    oneCtrlNode = OneCtrlNode()
    oneCtrlNode.run()
