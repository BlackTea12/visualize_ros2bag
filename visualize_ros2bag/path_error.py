from ros2_bag_extractor.util import directory as dir
from ros2_bag_extractor.bag_parser.parser import Bag2FileParser
from ros2_bag_extractor.bag_parser.object import ObjectType
from visualize_ros2bag.utils.path_process import find_closest_point_path_result
from visualize_ros2bag.utils.calculator import point_dist
import matplotlib.pyplot as plt
import numpy as np

def main():  
  # 1 #
  paths_db = dir.get_rosbag_file('rosbag', 'rosbag2_2024_06_24-15_12_06')
  db_Path = []
  if paths_db:
    parsed_data = Bag2FileParser(paths_db)
    data_list = ObjectType(parsed_data.get_messages('/plan'))
    db_Path = data_list.get_data('Path')  # collects path that is different during recording
    # print(db_Path[0][0])
    # print(db_Path[0][1])
    print(f"plan fetched, total path count is {len(db_Path)}")
  else:
    print('wrong access!')
    return

  amcl_pose_db = dir.get_rosbag_file('rosbag', 'rosbag2_2024_06_24-15_12_06')
  db_amcl_pose = []
  if amcl_pose_db:
    parsed_data = Bag2FileParser(amcl_pose_db)
    data_list = ObjectType(parsed_data.get_messages('/amcl_pose'))
    db_amcl_pose = data_list.get_data('PoseWithCovarianceStamped')
    print("amcl pose fetched")
  else:
    print('wrong access!')
    return
  
  # --- cleaned path --- #
  merged_path1 = find_closest_point_path_result(db_amcl_pose, db_Path)
  if len(merged_path1)==0:
    print('no path made')
    return
  print(f'path will be visualized with length of {len(merged_path1)}')
  robot1 = [p for t, p in db_amcl_pose]
  robot_time1 = [t for t, p in db_amcl_pose]

  # 2 #
  base_link_pose_db = dir.get_rosbag_file('rosbag', 'rosbag2_2024_06_21-14_59_09')
  db_pose = []
  if base_link_pose_db:
    parsed_data = Bag2FileParser(base_link_pose_db)
    data_list = ObjectType(parsed_data.get_messages('/base_link_pose'))
    db_pose = data_list.get_data('PoseStamped')
    # print(db_pose[0][0])
    # print(db_pose[0][1])
    print("base_link_pose fetched")
  else:
    print('wrong access!')
    return
  
  paths_db = dir.get_rosbag_file('rosbag', 'rosbag2_2024_06_21-09_53_33')
  db_Path = []
  if paths_db:
    parsed_data = Bag2FileParser(paths_db)
    data_list = ObjectType(parsed_data.get_messages('/plan'))
    db_Path = data_list.get_data('Path')  # collects path that is different during recording
    # print(db_Path[0][0])
    # print(db_Path[0][1])
    print(f"plan fetched, total path count is {len(db_Path)}")
  else:
    print('wrong access!')
    return
  # --- cleaned path --- #
  merged_path2 = find_closest_point_path_result(db_pose, db_Path)
  if len(merged_path2)==0:
    print('no path made')
    return
  print(f'path will be visualized with length of {len(merged_path2)}')

  robot2 = [p for t, p in db_pose]
  robot_time2 = [t for t, p in db_pose]

  # --- plot result --- #
  plot_lateral_error(robot1, merged_path1, robot_time1, robot2, merged_path2, robot_time2)

def calculate_lateral_error(robot:list, path:list, time:list):
  t, err = [], []
  
  for i in range(len(robot)):
    val = point_dist(robot[i][0], robot[i][1], path[i][0], path[i][1])
    if val > 0.7:
      continue
    t.append(time[i])
    err.append(point_dist(robot[i][0], robot[i][1], path[i][0], path[i][1]))
  return t, err

def plot_lateral_error(robot1:list, path1:list, robot_time1:list, robot2:list, path2:list, robot_time2:list):
  t1, err_x1 = calculate_lateral_error(robot1, path1, robot_time1)
  t2, err_x2 = calculate_lateral_error(robot2, path2, robot_time2)
  start_rb1 = t1[10]
  t1 = [(t-start_rb1)*1e-9 for t in t1]
  start_rb2 = t2[10]
  t2 = [(t-start_rb2)*1e-9 for t in t2]

  # post process
  t1 = t1[10:-1]
  t2 = t2[10:-1]
  err_x1 = err_x1[10:-1]
  err_x2 = err_x2[10:-1]

  m1, c1 = check_numerical_evaluation(err_x1)
  m2, c2 = check_numerical_evaluation(err_x2)

  plt.figure(figsize=(8, 6))
  plt.title("Lateral Error", weight='bold', fontsize=12)
  plt.plot(t1, [m1]*len(t1), lw=8, alpha=0.4, color='#9467bd')
  plt.plot(t2, [m2]*len(t2), lw=8, alpha=0.4, color='#ff6969')
  plt.plot(t1, err_x1, '*', lw=2, color='#9467bd', label='AMCL')
  plt.plot(t2, err_x2, lw=3, color='#ff6969', label='Camera SLAM')
  plt.xlabel('time[sec]', weight='bold')
  plt.ylabel('[m]', weight='bold')
  plt.grid(True)
  plt.legend(loc='best')
  
  plt.text(45, 0.25, 'avg: 0.068\ncov: 0.0112', fontsize=11, bbox=dict(boxstyle='square', color='white'))
  plt.text(1,0.25, 'avg: 0.079\ncov: 0.0092', fontsize=11, bbox=dict(boxstyle='square', color='white'))
  
  # annotation1
  a1 = plt.annotate('', xy=(50, 0.068), xytext=(50,0.25),
    fontsize=10, ha='center',
    arrowprops=dict(facecolor='#9467bd', width=0.5, shrink=0.1, headwidth=10))
  a1.draggable()
  # annotation2
  a2 = plt.annotate('', xy=(5, 0.079), xytext=(5,0.25),
    fontsize=10, ha='center',
    arrowprops=dict(facecolor='#ff6969', width=0.5, shrink=0.1, headwidth=10))
  a2.draggable()
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