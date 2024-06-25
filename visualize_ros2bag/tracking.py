from ros2_bag_extractor.util import directory as dir
from ros2_bag_extractor.bag_parser.parser import Bag2FileParser
from ros2_bag_extractor.bag_parser.object import ObjectType
from visualize_ros2bag.utils.path_process import find_closest_point_path_result
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from functools import partial
import time

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

  robot = [p for t, p in db_pose]

  # --- plot x-y coordinate result --- #
  # plot_xy_plane(robot, merged_path)

  # --- video animation ---#
  video_animation(robot, merged_path)

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

def video_plot_init(line1, line2):
  line1.set_data([], [])
  line2.set_data([], [])
  return line1, line2

def video_plot(frame, x1, y1, line1, x2, y2, line2):
  line1.set_data(x1[:frame], y1[:frame])
  line2.set_data(x2[:frame], y2[:frame])
  return line1, line2

def video_animation(robot:list, path:list):
  fig, ax = plt.subplots(figsize=(6, 5))
  ax.set_title("Trajectory Result", weight='bold', fontsize=12)
  ax.set_xlabel('X [m]', weight='bold')
  ax.set_ylabel('Y [m]', weight='bold')
  ax.grid(True)
  ax.set_xlim(-1,5)
  ax.set_ylim(-7,-2.5)
  robot_traj, = ax.plot([], [], '*', lw=2, color='#9467bd', label='robot trajectory')
  path_traj, = ax.plot([], [], lw=3, color='#ff6969', alpha=0.7, label='follow path')
  ax.legend()
  rb_x = [x for x,y,deg in robot]
  rb_y = [y for x,y,deg in robot]
  path_x = [x for x,y in path]
  path_y = [y for x,y in path]

  rb_x = rb_x[100:-1]
  rb_y = rb_y[100:-1]
  path_x = path_x[100:-1]
  path_y = path_y[100:-1]

  # Create the animation
  ani = animation.FuncAnimation(
    fig, partial(video_plot, x1=rb_x, y1=rb_y, line1=robot_traj, x2=path_x, y2=path_y, line2=path_traj)
    , frames=len(rb_x), init_func=partial(video_plot_init, line1=robot_traj, line2=path_traj)
    , blit=True
  )

  # Save the animation as an mp4 file
  # ani.save('/home/hd/main_ws/camera_slam_xyplane.mp4', writer='ffmpeg', fps=1/0.05)
  ani.save('/home/hd/main_ws/lidar_slam_xyplane.mp4', writer='ffmpeg', fps=1/0.05)
  
  print("file saved!")
if __name__ == '__main__':
  main()