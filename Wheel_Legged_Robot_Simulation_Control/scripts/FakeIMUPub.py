#!/usr/bin/python3
from math import pi, sqrt
import rospy
from geometry_msgs.msg import Pose,Twist
from sensor_msgs.msg import Imu
from gazebo_msgs.srv import GetModelState,GetModelStateRequest
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64
import time

class StateEstimationPublisher():
    RATE = 100 #hz
    MODEL = "Wheel_Legged_Robot"
    imu_msg = Imu()
    def __init__(self) -> None:
        rospy.init_node("state_estimator", anonymous=True)
        self.rate = rospy.Rate(self.RATE)
        self.imuPub = rospy.Publisher("/"+ self.MODEL + "/state_estimation/imu",Imu,latch=True,queue_size=10)
        # self.posePub = rospy.Publisher("/" + self.MODEL + "/state_estimation/pose",Pose,latch=True,queue_size=10)
        # self.twistPub = rospy.Publisher("/" + self.MODEL + "/state_estimation/twist",Twist,latch=True,queue_size=10)
        # self._fakePub = rospy.Publisher("/" + self.MODEL + "/state_estimation/pitch",Float64,latch=True,queue_size=10)
        self.run()

            
    def get_gazebo_models(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_coordinates = model_coordinates(self.MODEL, "")
            self.imu_msg.header = resp_coordinates.header
            self.imu_msg.header.frame_id = "imu"
            self.imu_msg.orientation = resp_coordinates.pose.orientation
            self.imu_msg.angular_velocity = resp_coordinates.twist.angular
            self.imu_msg.linear_acceleration = resp_coordinates.twist.linear
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))
    
    # def homotransform_to_pose_msg(homotransform):
    #     trans = tf.transformations.translation_from_matrix(homotransform)
    #     quat = tf.transformations.quaternion_from_matrix(homotransform)
    #     return Pose(
    #         position=Point(x=trans[0], y=trans[1], z=trans[2]),
    #         orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])) 


    def run(self):
        while not rospy.is_shutdown():
            #lm lf lr, rm, rf rr
            self.get_gazebo_models()
            self.imuPub.publish(self.imu_msg)
            self.rate.sleep
        
if __name__=="__main__":
    time.sleep(10)
    try:
        StateEstimationPublisher()
    except KeyboardInterrupt:
        exit()