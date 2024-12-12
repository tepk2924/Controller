import rospy
import os
import math
import yaml
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped

def distXY(pt1, pt2):
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

def CCW(pt1, pt2, pt3):
    vec21 = (pt2[0] - pt1[0], pt2[1] - pt1[1])
    vec32 = (pt3[0] - pt2[0], pt3[1] - pt2[1])
    cross = vec21[0]*vec32[1] - vec21[1]*vec32[0]
    return 1 if cross > 0 else -1 if cross < 0 else 0

def signed_curvature(pt1, pt2, pt3):
    '''
    세 점을 지나는 곡선의 회전곡률 계산; pt1, pt2, pt3 순으로 가는 경로; CCW는 양수, CW는 음수.
    '''
    a = distXY(pt1, pt2)
    b = distXY(pt2, pt3)
    c = distXY(pt3, pt1)
    s = (a + b + c)/2
    area = math.sqrt(s*(s-a)*(s-b)*(s-c))
    return 4*area/(a*b*c)*CCW(pt1, pt2, pt3)

def angle_confine(ang):
    ang %= 2*math.pi
    if ang > math.pi:
        ang -= 2*math.pi
    return ang

# class Integrator:
#     def __init__(self, Ts, initial_value:float=0, is_angle:bool=False):
#         self.Ts = Ts
#         self.integral = initial_value
#         self.is_angle = is_angle

#     def calc(self, val):
#         self.integral += val*self.Ts
#         if self.is_angle:
#             self.integral = angle_confine(self.integral)
#         return self.integral

#     def get_curr(self):
#         return self.integral

#TODO: 일단 지금은 레퍼런스보다 클때 브레이크 1.0, 작을 때 스로틀 1.0을 주는 간단한 로직을 사용하지만, 주행 중 문제 발생시 PID를 사용하는 전략을 사용할 것
# class PIDController:
#     def __init__(self, Kp, Ki, Kd, Ts, initial_e_prev):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.Ts = Ts
#         self.e_prev = initial_e_prev
#         self.e_integral = Integrator(Ts)

#     def __call__(self, e):
#         P = self.Kp*e
#         I = self.Ki*self.e_integral.calc(e)
#         D = self.Kd*(e - self.e_prev)/self.Ts
#         self.e_prev = e
#         return P + I + D

class KanayamaController:
    def __init__(self, Kx, Ky, Kw, max_vel, min_vel, max_steer, min_steer):
        self.Kx = Kx
        self.Ky = Ky
        self.Kw = Kw
        self.max_vel = max_vel
        self.min_vel = min_vel
        self.max_steer = max_steer
        self.min_steer = min_steer

    def __call__(self, x_err, y_err, theta_err, v_ref, steer_ref):
        v = v_ref*math.cos(theta_err) + self.Kx*x_err
        steer = steer_ref + v_ref*(self.Ky*y_err + self.Kw*math.sin(theta_err))
        v = min(v, self.max_vel)
        v = max(v, self.min_vel)
        steer = min(steer, self.max_steer)
        steer = max(steer, self.min_steer)
        return v, steer

def init():
    '''
    여기다가 waypoint들 (csv 파일 형식) 불러오는 코드를 넣을 것.
    '''
    repo_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))
    path_name = rospy.get_param("path_name")
    with open(os.path.join(repo_dir, "path", f"{path_name}.csv"), "r") as f:
        lines = f.readlines()

    #컨피그 yaml 파일 로드
    global configs
    with open(os.path.join(repo_dir, "config.yaml")) as f:
        configs = yaml.safe_load(f)

    #Contains X, Y, Z coordinate, then the sector value, and isdanger flag.
    waypoints_raw = [list(map(float, line.replace(","," ").split())) for line in lines]

    #만약 받은 파일의 시작 지점과 끝 지점이 같은 경우, 시작 지점을 삭제함.
    if distXY(waypoints_raw[0], waypoints_raw[-1]) <= 0.01:
        waypoints_raw = waypoints_raw[1:]
        if configs["debug_console_output"]:
            print("waypoint 파일의 첫 지점과 끝지점이 같아서 전처리 실시함.")

    global waypoints_num
    waypoints_num = len(waypoints_raw)
    
    #Contain X, Y, Z, theta, V, and steer.
    global waypoints
    waypoints = []
    for idx in range(waypoints_num):
        curr_waypoint_raw = waypoints_raw[idx]
        prev_waypoint_raw = waypoints_raw[(idx - 1) % waypoints_num]
        next_waypoint_raw = waypoints_raw[(idx + 1) % waypoints_num]
        #TODO: 일단 현재 기준 waypoint의 Z좌표가 0인 것으로 간주했음. 나중에 XYZ 좌표가 모두 있는 waypoint가 들어온다면 반영할 것.
        #TODO: 레퍼런스 속력도 현재는 최대 속력 값을 넣으나, 추후 수정의 여지가 있을 거임. 예를 들어 waypoint가 도로의 중앙에서 멀 수록 속력을 줄인다던가, 곡률에 따라 속력을 줄인다던가.
        waypoints.append((curr_waypoint_raw[0],
                          curr_waypoint_raw[1],
                          0,
                          math.atan2(next_waypoint_raw[1] - prev_waypoint_raw[1], next_waypoint_raw[0] - prev_waypoint_raw[0]),
                          configs["max_vel"] if curr_waypoint_raw[4] < 0.5 else configs["danger_vel"],
                          #math.atan(signed_curvature(prev_waypoint_raw, curr_waypoint_raw, next_waypoint_raw)*configs["L"])
                          0))

    global curr_waypoint_idx
    curr_waypoint_idx = 0

    global kanayama_controller
    kanayama_controller = KanayamaController(configs["Kx"], configs["Ky"], configs["Kw"], configs["max_vel"], configs["min_vel"], configs["max_steer"], configs["min_steer"])

def callback(data:Float32MultiArray):
    '''
    여기에 컨트롤러 메인 부분과 publish하는 코드를 넣을 거 같음
    '''
    global curr_waypoint_idx
    global configs
    global waypoints
    global waypoints_num
    global kanayama_controller
    x, y, theta, vel, steer = data.data
    #만약 그 현재 waypoint를 지나지 않았으면 while loop 통과, 그렇지 않다면 지나지 않은 waypoint를 찾을 때까지 whule loop를 돈다.
    while True:
        x_ref, y_ref, z_ref, theta_ref, v_ref, steer_ref = waypoints[curr_waypoint_idx]
        x_car_by_w = math.cos(theta_ref)*(x - x_ref) + math.sin(theta_ref)*(y - y_ref)
        # y_car_by_w = math.sin(-theta_ref)*(x - x_ref) + math.cos(theta_ref)*(y - y_ref)
        if x_car_by_w < -0.1: #레퍼런스 기준 차의 좌표가 레퍼런스 방향 기준 어느 정도 이상 뒤쳐지는 레퍼런스 지점의 번호를 찾는 것이 break 조건.
            break
        curr_waypoint_idx = (curr_waypoint_idx + 1) % waypoints_num
    x_sub = x_ref - x
    y_sub = y_ref - y
    x_err = math.cos(theta)*x_sub + math.sin(theta)*y_sub
    y_err = -math.sin(theta)*x_sub + math.cos(theta)*y_sub
    theta_err = angle_confine(theta_ref - theta)
    v_control, steer_control = kanayama_controller(x_err, y_err, theta_err, v_ref, steer_ref)
    if configs["debug_console_output"]:
        print(f"1. Data Received: {x = }, {y = }, {theta = }, {vel = }, {steer = }")
        print(f"2. Reference Point Info: {curr_waypoint_idx = }, {x_ref = }, {y_ref = }, {theta_ref = }, {v_ref = }, {steer_ref = }")
        print(f"3. Error Term Calculated: {x_err = }, {y_err = }, {theta_err = }")
        print(f"5. Kanayama Result: {v_control = }, {steer_control = }")
    
    #조향각을 -1.0에서 1.0 범위로 변환
    steer_normalized = -steer_control/configs["max_steer"]
    if steer_normalized > 1.0:
        steer_normalized = 1.0
    elif steer_normalized < -1.0:
        steer_normalized = -1.0

    if vel < v_control:
        throttle, brake = 1.0, 0.0
    elif vel > v_control:
        throttle, brake = 0.0, 1.0
    else:
        throttle, brake = 0.0, 0.0

    #Vector3Stamped 관련 헤더와 데이터 넣는 코드 구현
    msg:Vector3Stamped = Vector3Stamped()
    msg.header.frame_id = "3"
    msg.vector.x = throttle
    msg.vector.y = steer_normalized
    msg.vector.z = brake

    #Publish
    pub = rospy.Publisher("/mobile_system_control/control_msg", Vector3Stamped, queue_size=10)
    pub.publish(msg)

    # #debug
    # pub2 = rospy.Publisher("/debug", Vector3Stamped, queue_size=10)
    # msg2 = Vector3Stamped()
    # msg2.header.frame_id = "3"
    # msg2.vector.x = v_ref
    # msg2.vector.y = steer_ref
    # msg2.vector.z = 0
    # pub2.publish(msg2)
    # return


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