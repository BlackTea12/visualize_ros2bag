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

def find_traveled_robot_trajectory_in_time(robot:list, start_timestamp, limit_timestamp)-> list:
  '''
  @brief this function will select robot trajectory that actually traveled\
   based on timestamp
  @robot: [[timstamp0, (x, y, deg)], ...]
  @start_timestamp: published time of path
  @limit_timestamp: exceeding this value will break search in robot
  @return list of (x,y) robot trajectory
  '''
  result_path=[]
  
  for timestamp, pose in robot:
    if timestamp < start_timestamp:
      continue
    if timestamp > limit_timestamp:
      break
    result_path.append((pose[0], pose[1]))
  return result_path

def find_traveled_robot_trajectory_in_single_path(robot:list, path:Path, start_timestamp)-> list:
  '''
  @brief this function will select robot trajectory that actually traveled\
   based on given path
  @robot: [[timstamp0, (x, y, deg)], ...]
  @paths: nav_msgs.msg.Path
  @start_timestamp: published time of path
  @limit_timestamp: exceeding this value will break search in robot
  @return list of (x,y) robot trajectory
  '''
  result_path, ts = [], []
  idx = 0 
  for timestamp, pose in robot:
    if timestamp < start_timestamp:
      continue
    if idx > len(path.poses)*1.5:
      break
    idx += 1
    result_path.append((pose[0], pose[1]))
    ts.append(timestamp)
    
  return result_path, ts

def filter_only_length_diff_path(paths:list)-> list:
  '''
  @paths: [[timstamp0, Path0], ...]
  '''
  result = []
  result.append(paths[0])
  for path in paths:
    if len(result[-1][1].poses) != len(path[1].poses):
      result.append(path)
    
  return result