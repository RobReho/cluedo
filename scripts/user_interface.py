import rospy
from std_msgs.msg import String

def callback(msg):
    print(msg.data)
    
    
    
def main():
    rospy.init_node('user_interface')
    
    print('-_-_CLUEDO_-_-')
    print('welcome')
    print('___________________________________________________')
    rospy.Subscriber("ui_output", String, callback)
    
    rospy.spin()
    
    
if __name__ == '__main__':
    main()