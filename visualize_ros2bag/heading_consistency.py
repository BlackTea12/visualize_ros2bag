from ros2_bag_extractor.util import directory as dir
from ros2_bag_extractor.bag_parser.parser import Bag2FileParser
from ros2_bag_extractor.bag_parser.object import ObjectType
from visualize_ros2bag.utils.path_process import find_closest_point_path_result
from visualize_ros2bag.utils.calculator import point_dist
import matplotlib.pyplot as plt
import numpy as np

def main():  
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
  robot1 = [p for t, p in db_amcl_pose]
  robot_time1 = [t for t, p in db_amcl_pose]

  # 2 #
  base_link_pose_db = dir.get_rosbag_file('rosbag', 'rosbag2_2024_06_21-14_59_09')
  db_pose = []
  if base_link_pose_db:
    parsed_data = Bag2FileParser(base_link_pose_db)
    data_list = ObjectType(parsed_data.get_messages('/base_link_pose'))
    db_pose = data_list.get_data('PoseStamped')
    print("base_link_pose fetched")
  else:
    print('wrong access!')
    return
  
  robot2 = [p for t, p in db_pose]
  robot_time2 = [t for t, p in db_pose]

  # --- plot result --- #
  plot_heading(robot1, robot_time1, robot2, robot_time2)

def plot_heading(robot1:list, robot_time1:list, robot2:list, robot_time2:list):
  start_rb1 = robot_time1[10]
  robot_time1 = [(t-start_rb1)*1e-9 for t in robot_time1]
  start_rb2 = robot_time2[10]
  robot_time2 = [(t-start_rb2)*1e-9 for t in robot_time2]

  head1 = [h*57.32 for x,y,h in robot1]
  head2 = [h*57.32 for x,y,h in robot2]

  # post process
  robot_time1 = robot_time1[10:-1]
  robot_time2 = robot_time2[10:-1]
  head1 = head1[10:-1]
  head2 = head2[10:-1]

  fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))
  plt.suptitle("Localized Heading", weight='bold', fontsize=18)
  ax1.set_title("AMCL", weight='bold', fontsize=14)
  ax2.set_title("Camera SLAM", weight='bold', fontsize=14)
  ax1.plot(robot_time1, head1, '--', lw=1.5, color='#9467bd', label='AMCL')
  ax2.plot(robot_time2, head2, lw=1.5, color='#ff6969', label='Camera SLAM')
  ax1.set_xlabel('time[sec]', weight='bold')
  ax1.set_ylabel('[deg]', weight='bold')
  ax2.set_xlabel('time[sec]', weight='bold')
  ax2.set_ylabel('[deg]', weight='bold')
  ax1.set_xlim(0,55)
  ax2.set_xlim(0,55)
  ax1.set_ylim(-180,180)
  ax2.set_ylim(-180,180)
  ax2.grid(True)
  ax1.grid(True)
  fig.tight_layout()
  plt.show()

if __name__ == '__main__':
  main()