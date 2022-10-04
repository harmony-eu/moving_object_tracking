#!/usr/bin/env python
import rospy
from obstacle_detector.msg import Obstacles, CircleObstacle, SegmentObstacle

class ObstacleSubscriber:

    def __init__(self):
        print("ObstacleSubscriber initiated")
        self.sub = rospy.Subscriber("obstacles", Obstacles, self.obstacle_callback)


    def obstacle_callback(self, obs_msg):
        self.circles = obs_msg.circles
        self.segments = obs_msg.segments
        self.nr_circles = len(self.circles)
        if self.nr_circles > 0:
            print(self.nr_circles, " circles detected")
            for i in range(self.nr_circles):
                print(i, self.circles[i].center)
        else: 
            print("no circles detected")


    
def main():

    rospy.init_node('obstacle_subscriber', anonymous=True)
    obstacle_subscriber = ObstacleSubscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
