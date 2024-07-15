from ros2_bag_extractor.util import directory as dir
from ros2_bag_extractor.bag_parser.parser import Bag2FileParser
from ros2_bag_extractor.bag_parser.object import ObjectType
from visualize_ros2bag.utils.path_process import find_robot_closest_index_in_path, find_traveled_robot_trajectory_in_time, find_traveled_robot_trajectory_in_single_path
from visualize_ros2bag.matplots.user import UserSingleMatplot
from visualize_ros2bag.utils.calculator import check_numerical_evaluation, point_dist

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from functools import partial
import time

def main():
  paths_db = dir.get_rosbag_file('rosbag/scenes', 'rosbag2_2024_07_11-15_17_54')
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

  amcl_pose_db = dir.get_rosbag_file('rosbag/scenes', 'rosbag2_2024_07_11-15_17_54')
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
  single_path_checker = UserSingleMatplot(db_Path)
  # single_path_checker.check_paths()
  
  # --- plot x-y coordinate result --- #
  # plot_xy_plane(db_amcl_pose, db_Path, single_path_checker.get_selected_idx())
  plot_xy_plane_with_path_error(db_amcl_pose, db_Path, [0,2])

def plot_xy_plane(robot_db:list, path_db:list, idxes:list):
  fig, axis = plt.subplots(1,len(idxes))
  fig.suptitle('Observing Trajectory Change')

  for i in range(len(idxes)):
    if i == 0:
      robot = find_traveled_robot_trajectory_in_time(robot_db, path_db[idxes[i]][0], path_db[idxes[i]+1][0])
    else:
      robot  = find_traveled_robot_trajectory_in_single_path(robot_db, path_db[idxes[i]][1], path_db[idxes[i]][0])
    robot_x = [x for x, y in robot]
    robot_y = [y for x, y in robot]
    path_x = [p.pose.position.x for p in path_db[idxes[i]][1].poses]
    path_y = [p.pose.position.y for p in path_db[idxes[i]][1].poses]

    path_number = str(idxes[i])+"th follow path"
    axis[i].set_xlim((3,10))
    axis[i].set_ylim((10,15))
    axis[i].plot(robot_x, robot_y, '*', color='#9467bd', label='robot trajectory')
    axis[i].plot(path_x, path_y, color='#ff6969', label=path_number)
    axis[i].set_title(f't+{i+1}', weight='bold')
    axis[i].grid(True)
    axis[i].legend(loc='best')

  plt.tight_layout()
  plt.show()

def plot_xy_plane_with_path_error(robot_db:list, path_db:list, idxes:list):
  fig= plt.figure(figsize=(10,8))
  gs = fig.add_gridspec(2,len(idxes))
  fig.suptitle('Observing Trajectory Change')
  error = [], []

  for i in range(len(idxes)):
    if i == 0:
      robot = find_traveled_robot_trajectory_in_time(robot_db, path_db[idxes[i]][0], path_db[idxes[i]+1][0])
    else:
      robot, ts = find_traveled_robot_trajectory_in_single_path(robot_db, path_db[idxes[i]][1], path_db[idxes[i]][0])
      # path = [(p.pose.position.x,p.pose.position.y) for p in path_db[idxes[i]][1].poses]
      error = calculate_lateral_error(robot, path_db[idxes[i]][1])
    robot_x = [x for x, y in robot]
    robot_y = [y for x, y in robot]
    path_x = [p.pose.position.x for p in path_db[idxes[i]][1].poses]
    path_y = [p.pose.position.y for p in path_db[idxes[i]][1].poses]

    path_number = str(idxes[i])+"th follow path"
    axis = fig.add_subplot(gs[0,i])
    axis.set_xlim((3,10))
    axis.set_ylim((10,15))
    axis.plot(robot_x, robot_y, '*', color='#9467bd', label='robot trajectory')
    axis.plot(path_x, path_y, color='#ff6969', label=path_number)
    axis.set_title(f't+{i+1}', weight='bold')
    axis.grid(True)
    axis.legend(loc='best')
  
  # lateral error
  start_ts = ts[0]
  ts_filtered = [(t-start_ts)*1e-9 for t in ts]
  mean, variance = check_numerical_evaluation(error)
  mean = round(mean,2)
  variance = round(variance, 4)
  lateral_error_axis = fig.add_subplot(gs[1,:])
  lateral_error_axis.plot(ts_filtered, [mean]*len(ts_filtered), lw=16, alpha=0.3, color='#9467bd')
  lateral_error_axis.plot(ts_filtered, error, '.', color='#9467bd')
  lateral_error_axis.set_xlabel('time[sec]', weight='bold')
  lateral_error_axis.set_ylabel('[m]', weight='bold')
  lateral_error_axis.grid(True)
  lateral_error_axis.set_title('lateral error', weight='bold')
  lateral_error_axis.text(12, 0.11, f'avg: {mean}\ncov: {variance}', fontsize=11, bbox=dict(boxstyle='square', color='white'))
  a1 = lateral_error_axis.annotate('', xy=(10, mean-0.01), xytext=(12,0.11),
    fontsize=10, ha='center',
    arrowprops=dict(facecolor='#9467bd', width=0.5, shrink=0.1, headwidth=10))
  a1.draggable()

  plt.tight_layout()
  plt.show()

def calculate_lateral_error(robot:list, path):
  '''
  @robot: [(x1,y1,deg1), (x2,y2,deg2), ...]
  @path: nav_msgs.msg.Path
  @return err in each list format
  '''
  err = []
  
  for i in range(len(robot)):
    idx = find_robot_closest_index_in_path(robot[i][0], robot[i][1], path)
    # if val > 0.7:
    #   continue
    err.append(point_dist(robot[i][0], robot[i][1], path.poses[idx].pose.position.x, path.poses[idx].pose.position.y))
  return err

if __name__ == '__main__':
  main()