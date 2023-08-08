#!/usr/bin/env python3

from robot import Robot
import rospy
import message_filters
from std_msgs.msg import Float64

class Mover:
   
   def __init__(self):
      
      self.robot = Robot()
      
      self.x_correction_sub = message_filters.Subscriber("control_effort_x", Float64, queue_size=1)
      self.y_correction_sub = message_filters.Subscriber("control_effort_y",Float64, queue_size=1)
      self.ts = message_filters.ApproximateTimeSynchronizer([self.x_correction_sub, self.y_correction_sub], queue_size=1, slop=0.1, allow_headerless=True).registerCallback(self.callback)

      
    
   def callback(self, x, y):
      
      x_move = -x.data
      y_move = -y.data

      print("Current: " + str(self.robot.getPositions()[0]))
      print(x_move)

      # self.robot.incrementPositions([x_move, y_move, -x_move, y_move])
      self.robot.incrementPositions([x_move, y_move, -x_move, y_move])





def main():
  rospy.init_node('eye_mover', anonymous=True)
  mover = Mover()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()
