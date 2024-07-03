from ros2_bag_extractor.util import directory as dir
from ros2_bag_extractor.bag_parser.parser import Bag2FileParser
from ros2_bag_extractor.bag_parser.object import ObjectType
from visualize_ros2bag.utils.path_process import find_robot_closest_index_in_path
from visualize_ros2bag.utils.calculator import point_dist
import matplotlib.pyplot as plt
import numpy as np

def main():  
  paths_db = dir.get_rosbag_file('rosbag', 'rosbag2_2024_07_03-14_10_09')
  db_Path = []
  if paths_db:
    parsed_data = Bag2FileParser(paths_db)
    data_list = ObjectType(parsed_data.get_messages('/plan'))
    db_Path = data_list.get_data('Path')  # collects path that is different during recording, timestamp, Path
    # print(db_Path[0][0])
    # print(db_Path[0][1])
    print(f"plan fetched, total path count is {len(db_Path)}")
  else:
    print('wrong access!')
    return

  amcl_pose_db = dir.get_rosbag_file('rosbag', 'rosbag2_2024_07_03-14_10_09')
  db_amcl_pose = []
  if amcl_pose_db:
    parsed_data = Bag2FileParser(amcl_pose_db)
    data_list = ObjectType(parsed_data.get_messages('/amcl_pose'))
    db_amcl_pose = data_list.get_data('PoseWithCovarianceStamped')  # timestamp, (x,y,yaw)
    print(f"amcl pose fetched, total pose count is {len(db_amcl_pose)}")
  else:
    print('wrong access!')
    return
  
  # --- slicing poses process through time --- #
  sliced_poses = []
  for i in range(1,len(db_Path)): # skipping for first trash path
    before_timestamp = db_Path[i-1][0]
    next_timestamp = db_Path[i][0]
    poses = []
    for t, pose in db_amcl_pose:
      if t < before_timestamp:
        continue
      elif t < next_timestamp:
        poses.append([t, pose])
      else:
        break
    # print(f'saving poses as {len(poses)}')
    sliced_poses.append(poses)
  # print(f'sliced poses saved as {len(sliced_poses)}')
  db_Path = db_Path[1:] # apply slicing for match of poses

  # --- collecting data --- #
  total_time, total_err = [], []
  print(f'collecting data for path {len(db_Path)} and poses {len(sliced_poses)}')
  for i in range(len(sliced_poses)):
    time_arr, err_arr = calculate_lateral_error(sliced_poses[i], db_Path[i])
    total_time = total_time + time_arr
    total_err = total_err + err_arr

  # --- plot data --- #
  plot_lateral_error(total_time, total_err)

def calculate_lateral_error(robot:list, path:list):
  '''
  @robot (time1, (x,y,yaw)), ...
  @path [time, Path] 
  @return time array and error array
  '''
  t_arr, err = [], []
  for t, tup in robot:
    idx = find_robot_closest_index_in_path(tup[0], tup[1], path[1])
    val = point_dist(tup[0], tup[1], path[1].poses[idx].pose.position.x, path[1].poses[idx].pose.position.y)
    # if val > 0.7:
    #   continue
    t_arr.append(t)
    err.append(val)
  return t_arr, err

def plot_lateral_error(time_arr:list, err_arr:list):
  start = time_arr[0]
  timestamps = [(t-start)*1e-9 for t in time_arr]

  # post process
  # t1 = t1[10:-1]
  # t2 = t2[10:-1]
  # err_x1 = err_x1[10:-1]
  # err_x2 = err_x2[10:-1]

  mean_X, variance_X = check_numerical_evaluation(err_arr)
  mean_X = round(mean_X,2)
  variance_X = round(variance_X, 4)
  plt.figure(figsize=(8, 6))
  plt.title("Lateral Error", weight='bold', fontsize=12)
  plt.plot(timestamps, [mean_X]*len(timestamps), lw=16, alpha=0.3, color='#9467bd')
  plt.plot(timestamps, err_arr, '.', color='#9467bd')
  plt.xlabel('time[sec]', weight='bold')
  plt.ylabel('[m]', weight='bold')
  plt.grid(True)
  # plt.legend(loc='best')
  
  plt.text(120, 0.17, f'avg: {mean_X}\ncov: {variance_X}', fontsize=11, bbox=dict(boxstyle='square', color='white'))
  # plt.text(1,0.25, 'avg: 0.079\ncov: 0.0092', fontsize=11, bbox=dict(boxstyle='square', color='white'))
  
  # annotation1
  a1 = plt.annotate('', xy=(50, mean_X-0.01), xytext=(120,0.17),
    fontsize=10, ha='center',
    arrowprops=dict(facecolor='#9467bd', width=0.5, shrink=0.1, headwidth=10))
  a1.draggable()
  plt.show()

def check_numerical_evaluation(error:list):
  # Calculate sample mean (average)
  mean_X = np.mean(error)
  print("Sample mean:", mean_X)

  # Calculate sample variance (covariance with itself)
  variance_X = np.var(error, ddof=1)  # ddof=1 for sample variance (unbiased estimator)
  print("Sample variance (covariance with itself):", variance_X)
  return mean_X, variance_X

if __name__ == '__main__':
  main()