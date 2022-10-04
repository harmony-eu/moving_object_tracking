#!/usr/bin/env python
import rospy
import numpy as np
from obstacle_detector.msg import Obstacles, CircleObstacle, SegmentObstacle

class ObstacleSubscriber:

    def __init__(self):
        print("ObstacleSubscriber initiated")
        self.sub = rospy.Subscriber("obstacles", Obstacles, self.obstacle_callback)
        self.pub = rospy.Publisher('obstacles_moving', Obstacles, queue_size=10)
        self.vel_thresh = 0.4 #min velocity to publish in moving objects message (m/s)


    def obstacle_callback(self, obs_msg):
        self.circles = obs_msg.circles
        self.segments = obs_msg.segments
        self.nr_circles = len(self.circles)
        self.moving_circles = []

        # filter out the moving obstacles
        if self.nr_circles > 0:
            print(self.nr_circles, " circles detected")
            for i in range(self.nr_circles):
                # print(i, self.circles[i].velocity)
                if np.linalg.norm([self.circles[i].velocity.x, self.circles[i].velocity.y]) > self.vel_thresh:
                    self.moving_circles.append(self.circles[i])
            
            # publish only the moving circles
            print(len(self.moving_circles), " circles are moving")
            moving_obs_msg = Obstacles()
            moving_obs_msg.circles = self.moving_circles
            moving_obs_msg.header = obs_msg.header
            self.pub.publish(moving_obs_msg)

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
