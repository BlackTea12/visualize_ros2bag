from math import dist
from nav_msgs.msg import Path

def find_robot_closest_index_in_path(x, y, path:Path)-> int: 
  '''
  @x: robot point x
  @y: robot point y
  '''
  idx = 0
  min_dist = 1000
  for i in range(len(path.poses)):
    d = dist([x, y],[path.poses[i].pose.position.x, path.poses[i].pose.position.y])
    if d < min_dist:
      min_dist = d
      idx = i
  return idx

def find_closest_point_path_result(robot:list, paths:list)-> list:
  '''
  @brief based on recorded multiple path, this function will merge\
   path point closest to robot. Also time will be compared to apply path change
  @robot: [[timstamp0, (x, y, deg)], ...]
  @paths: [[timstamp0, Path0], ...]
  @return list of (x,y)
  '''
  if len(paths) == 1:
    return paths[0][1]
  
  idx = 0
  path_timestamp = paths[idx+1][0]
  current_path = paths[idx][1]
  result_path=[]
  
  for timestamp, pose in robot:
    if timestamp > path_timestamp:
      idx += 1
      if idx == len(paths):
        break
      elif idx == len(paths)-1:
        path_timestamp = robot[-1][0]+1
      current_path = paths[idx][1]

    closest_idx = find_robot_closest_index_in_path(pose[0], pose[1], current_path)
    result_path.append((current_path.poses[closest_idx].pose.position.x, current_path.poses[closest_idx].pose.position.y))
  
  return result_path