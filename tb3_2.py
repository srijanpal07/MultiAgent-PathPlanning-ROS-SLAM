#!/usr/bin/env python
import rospy
from std_msgs.msg import String
i = 0
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard: %s", data.data)
    hello_str = "I'm turtebot 2"
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    


if __name__ == '__main__':
   
    rospy.init_node('tb3_2', anonymous=True)
    rate = rospy.Rate(0.1)
    try:
        while not rospy.is_shutdown():
         
            pub = rospy.Publisher('Location', String, queue_size=10)
            rospy.Subscriber("Location", String, callback)
            rospy.spin()
    except rospy.ROSInterruptException:
        pass

