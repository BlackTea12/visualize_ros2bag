from ros2_bag_extractor.util import directory as dir
from ros2_bag_extractor.bag_parser.parser import Bag2FileParser
from ros2_bag_extractor.bag_parser.object import ObjectType
from visualize_ros2bag.utils.path_process import find_closest_point_path_result
import matplotlib.pyplot as plt

def main():
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
  merged_path = find_closest_point_path_result(db_pose, db_Path)
  if len(merged_path)==0:
    print('no path made')
    return
  print(f'path will be visualized with length of {len(merged_path)}')

  # --- plot x-y coordinate result --- #
  robot = [p for t, p in db_pose]
  plot_xy_plane(robot, merged_path)

def plot_xy_plane(robot:list, path:list):
  rb_x = [x for x,y,deg in robot]
  rb_y = [y for x,y,deg in robot]
  path_x = [x for x,y in path]
  path_y = [y for x,y in path]
  plt.title("Trajectory Result", weight='bold', fontsize=12)
  plt.plot(rb_x, rb_y, '*', color='#9467bd', label='robot trajectory')
  plt.plot(path_x, path_y, color='#ff6969', label='follow path')
  plt.xlabel('X [m]', weight='bold')
  plt.ylabel('Y [m]', weight='bold')
  plt.grid(True)
  plt.legend(loc='best')
  plt.show()

if __name__ == '__main__':
  main()