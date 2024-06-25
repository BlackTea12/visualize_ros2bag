import math
from geometry_msgs.msg import PointStamped

def pointstamped_dist(p1:PointStamped, p2:PointStamped): 
  return math.dist([p1.point.x, p1.point.y],[p2.point.x, p2.point.y])