import rospy
from std_msgs.msg import Float32MultiArray

def main():
    pub = rospy.Publisher("/mobile_system_control/ego_vehicle", Float32MultiArray, queue_size=10)
    rospy.init_node("testtalk", anonymous=True)
    msg = Float32MultiArray()
    msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
    pub.publish(msg)

if __name__ == "__main__":
    main()