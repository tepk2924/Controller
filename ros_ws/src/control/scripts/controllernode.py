import rospy
import os
import yaml
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped

def init():
    '''
    여기다가 waypoint들 (csv 파일 형식) 불러오고 그 waypoint들을 최적화하는 코드를 넣을 것.
    '''
    global waypoints
    waypoints = ...

def callback(data:Float32MultiArray):
    '''
    여기에 컨트롤러 메인 부분과 publish하는 코드를 넣을 거 같음
    '''
    x, y, theta, vel, steer = data.data
    #TODO: 카나야마 혹은 MPC 여기다가 구현

    msg:Vector3Stamped = Vector3Stamped()
    
    #TODO: 여기다가 Vector3Stamped 관련 헤더파일과 데이터 넣는 코드 구현

    pub = rospy.Publisher("/mobile_system_control/control_msg", Vector3Stamped)
    pub.publish(msg)
    return 

def main():
    '''
    여기다가 sub 하는 메인 코드
    '''
    rospy.init_node("controller", anonymous=True)
    rospy.Subscriber("/mobile_system_control/ego_vehicle", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    init()
    main()