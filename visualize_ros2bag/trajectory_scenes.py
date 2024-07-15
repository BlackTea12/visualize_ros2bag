from ros2_bag_extractor.util import directory as dir
from ros2_bag_extractor.bag_parser.parser import Bag2FileParser
from ros2_bag_extractor.bag_parser.object import ObjectType
from visualize_ros2bag.utils.path_process import find_traveled_robot_trajectory_in_time, find_traveled_robot_trajectory_in_single_path
from visualize_ros2bag.matplots.user import UserSingleMatplot

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from functools import partial
import time

def main():
  # base_link_pose_db = dir.get_rosbag_file('rosbag', 'rosbag2_2024_06_21-14_59_09')
  # db_pose = []
  # if base_link_pose_db:
  #   parsed_data = Bag2FileParser(base_link_pose_db)
  #   data_list = ObjectType(parsed_data.get_messages('/base_link_pose'))
  #   db_pose = data_list.get_data('PoseStamped')
  #   # print(db_pose[0][0])
  #   # print(db_pose[0][1])
  #   print("base_link_pose fetched")
  # else:
  #   print('wrong access!')
  #   return
  
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
  single_path_checker.check_paths()
  
  # --- plot x-y coordinate result --- #
  plot_xy_plane(db_amcl_pose, db_Path, single_path_checker.get_selected_idx())

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

if __name__ == '__main__':
  main()