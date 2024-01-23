#!/usr/bin/python3
import rospy
from math import acos, sqrt
import numpy as np
from std_msgs.msg import Float64

class CloseLoopController:
    # Geometry params in mm
    BASE_DRIVE = 150.0 #base lenth from drive motor to another
    DRI_LINK_LEN = 150.0 #drive link lenth point to point
    MOT_LINK_DRI_LEN = 273.724 #motor link lenth from drive point to wheel point
    MOT_LINK_WHE_LEN = 46.089 #motor link lenth from passive point to wheel point
    MOT_LINK_REAL_LEN = 280.0 #motor link lenth from drive point to passive point
    MOT_DRI_ANG = 0.16564047266052187
    PAS_LINK_LEN = 230.0  # virtual passive link lenth
    legControlTarget = {'leftHeight':180.0,'leftPosition':0.0,'rightHeight':150.0,'rightPosition':0.0}


    def __init__(self):
        rospy.init_node("close_loop_simulation_controller", anonymous=True)
        self.rate = rospy.Rate(50)

        self.leftHeightSub = rospy.Subscriber("/Wheel_Legged_Robot/Left_Height", Float64, self.LeftHeightCallback)
        self.leftPositionSub = rospy.Subscriber("/Wheel_Legged_Robot/Left_Position", Float64, self.LeftPositionCallback)

        self.rightHeightSub = rospy.Subscriber("/Wheel_Legged_Robot/Right_Height", Float64, self.RightHeightCallback)
        self.rightPositionSub = rospy.Subscriber("/Wheel_Legged_Robot/Right_Position", Float64, self.RightPositionCallback)

        self.leftFrontDrivePub = rospy.Publisher("/Wheel_Legged_Robot/Left_Front_Drive_position_controller/command",Float64,latch=True,queue_size=10)
        self.leftRearDrivePub = rospy.Publisher("/Wheel_Legged_Robot/Left_Rear_Drive_position_controller/command",Float64,latch=True,queue_size=10)
        self.leftMotLinkDrivePub = rospy.Publisher("/Wheel_Legged_Robot/Left_1_position_controller/command",Float64,latch=True,queue_size=10)

        self.rightFrontDrivePub = rospy.Publisher("/Wheel_Legged_Robot/Right_Front_Drive_position_controller/command",Float64,latch=True,queue_size=10)
        self.rightRearDrivePub = rospy.Publisher("/Wheel_Legged_Robot/Right_Rear_Drive_position_controller/command",Float64,latch=True,queue_size=10)
        self.rightMotLinkDrivePub = rospy.Publisher("/Wheel_Legged_Robot/Right_1_position_controller/command",Float64,latch=True,queue_size=10)
        self.PhysicUpdate()


    def LeftHeightCallback(self, msg):
        self.legControlTarget['leftHeight'] = msg.data
        print(type(self.legControlTarget['leftHeight']))


    def LeftPositionCallback(self, msg):
        self.legControlTarget['leftPosition'] = msg.data


    def RightHeightCallback(self, msg):
        self.legControlTarget['rightHeight'] = msg.data


    def RightPositionCallback(self, msg):
        self.legControlTarget['rightPosition'] = msg.data


    def GetCommandAngle(self,Target_Position, Target_Height):
        _point_A = np.array([self.BASE_DRIVE / 2, 0])
        _point_B = np.array([-self.BASE_DRIVE / 2, 0])
        _point_P = np.array([Target_Position, -Target_Height])
        _vec_AB = _point_B - _point_A
        _vec_AP = _point_P - _point_A
        _ang_PAB = self.GetAngle(_vec_AP,_vec_AB)
        _ang_PAA_ = self.GetUpperAngle(adjacent1=np.linalg.norm(_vec_AP), adjacent2=self.DRI_LINK_LEN, opposite=self.MOT_LINK_DRI_LEN)
        _ang_PA_A = self.GetUpperAngle(adjacent1=self.MOT_LINK_DRI_LEN, adjacent2=self.DRI_LINK_LEN, opposite=np.linalg.norm(_vec_AP))
        if (_ang_PAA_ + _ang_PAB > np.pi):
            _point_A_ = np.array([(self.BASE_DRIVE / 2) + self.DRI_LINK_LEN * np.cos(_ang_PAA_ + _ang_PAB - np.pi) ,
                                  self.DRI_LINK_LEN * np.sin(_ang_PAA_ + _ang_PAB - np.pi)])
        else:
            _point_A_ = np.array([(self.BASE_DRIVE / 2) + self.DRI_LINK_LEN * np.cos(np.pi - _ang_PAA_ - _ang_PAB),
                                  -self.DRI_LINK_LEN * np.sin(np.pi - _ang_PAA_ - _ang_PAB)])
        _point_K = _point_A_ + np.array([self.MOT_LINK_REAL_LEN * np.cos(np.pi- _ang_PA_A + self.MOT_DRI_ANG),
                                         self.MOT_LINK_REAL_LEN * np.sin(np.pi- _ang_PA_A + self.MOT_DRI_ANG)])
        _vec_BK = _point_K - _point_B
        _ang_KBA = self.GetAngle(_vec_BK,-_vec_AB)
        _ang_KBB_ = self.GetUpperAngle(adjacent1=np.linalg.norm(_vec_BK), adjacent2=self.DRI_LINK_LEN, opposite=self.PAS_LINK_LEN)
        _command_ang_A = _ang_PAA_+ _ang_PAB - np.pi
        _command_ang_B = _ang_KBA + _ang_KBB_ - np.pi
        return _command_ang_A,_command_ang_B ,_ang_PA_A


    def GetAngle(self, Vec1, Vec2):
        _l1 = np.sqrt(Vec1.dot(Vec1))
        _l2 = np.sqrt(Vec2.dot(Vec2))
        _times = Vec1.dot(Vec2)
        return np.arccos(_times/(_l2 * _l1))


    def GetUpperAngle(self, adjacent1,adjacent2, opposite):
        return np.arccos((adjacent1**2 + adjacent2**2 - opposite**2)/(2 * adjacent1 * adjacent2))


    def PhysicUpdate(self):
        while not rospy.is_shutdown():
            #left
            #print(self.legControlTarget)
            _command_ang_A,_command_ang_B ,_command_ang_A_ = self.GetCommandAngle(Target_Height=self.legControlTarget['leftHeight'],
                                                                                  Target_Position=self.legControlTarget['leftPosition'])
            self.leftFrontDrivePub.publish(-_command_ang_A)
            self.leftRearDrivePub.publish(_command_ang_B)
            self.leftMotLinkDrivePub.publish(np.deg2rad(34.62)-_command_ang_A_)
            # print(np.rad2deg(_command_ang_A),np.rad2deg(_command_ang_B) ,np.rad2deg(_command_ang_A_))

            #right
            _command_ang_A,_command_ang_B ,_command_ang_A_ = self.GetCommandAngle(Target_Height=self.legControlTarget['rightHeight'],
                                                                                  Target_Position=-self.legControlTarget['rightPosition'])
            self.rightFrontDrivePub.publish(_command_ang_B)
            self.rightRearDrivePub.publish(-_command_ang_A)
            self.rightMotLinkDrivePub.publish(_command_ang_A_ - np.deg2rad(34.62))
            # print(np.rad2deg(_command_ang_A),np.rad2deg(_command_ang_B) ,np.rad2deg(_command_ang_A_))



if __name__=="__main__":
    try:
        _controller = CloseLoopController()
    except KeyboardInterrupt:
        exit()
