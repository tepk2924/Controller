import rospy
import math
from std_msgs.msg import Float32MultiArray

def main():
    pub = rospy.Publisher("/mobile_system_control/ego_vehicle", Float32MultiArray, queue_size=10)
    rospy.init_node("testtalk", anonymous=True)
    msg = Float32MultiArray()
    msg.data = [81.48997000882729, -13.339552285333534, math.pi/2, 0.0, 0.0]
    pub.publish(msg)

if __name__ == "__main__":
    main()