from ros2_bag_extractor.util import directory as dir
from ros2_bag_extractor.bag_parser.parser import Bag2FileParser
from ros2_bag_extractor.bag_parser.object import ObjectType

import matplotlib.pyplot as plt

def main():
  # our data
  db_file_name = dir.get_rosbag_file('main_ws/ros2bag/ours_narrow', 'rosbag2_2024_11_14-11_17_14')
  our = []
  if db_file_name:
    parsed_data = Bag2FileParser(db_file_name)
    msgs = parsed_data.get_messages('/cmd_vel')
    data_list = ObjectType(msgs)
    our = data_list.get_data('Twist')
    print("\x1b[38;20msuccessfully finished!\x1b[0m")
  else:
    print('\x1b[31;20mwrong access!\x1b[0m')
    return

  # comparitve data
  db_file_name = dir.get_rosbag_file('main_ws/ros2bag/other_narrow', 'rosbag2_2024_11_14-11_02_31')
  other = []
  if db_file_name:
    parsed_data = Bag2FileParser(db_file_name)
    msgs = parsed_data.get_messages('/cmd_vel')
    data_list = ObjectType(msgs)
    other = data_list.get_data('Twist')
    print("\x1b[38;20msuccessfully finished!\x1b[0m")
  else:
    print('\x1b[31;20mwrong access!\x1b[0m')
    return

  # plot design  
  our_start_time = our[0].sec
  our_speed, our_rot, our_sec = [],[],[]
  for data in our:
    tick = data.sec-our_start_time
    if tick >= 16.385:
      our_speed.append(data.speed)
      our_rot.append(data.w)
      our_sec.append(tick-16.385)

  other_start_time = other[0].sec
  other_speed, other_rot, other_sec = [],[],[]
  for data in other:
    tick = data.sec-other_start_time
    if tick >= 5.9:
      other_speed.append(data.speed)
      other_rot.append(data.w)
      other_sec.append(tick-5.9)

  fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))
  plt.suptitle("Speed Command Comparison", weight='bold', fontsize=18)
  
  ax1.set_title("linear speed", weight='bold', fontsize=14, color='#363636')
  ax1.plot(our_sec, our_speed, '--', lw=1.5, color='#9467bd', label='ours')
  ax1.plot(other_sec, other_speed, lw=1.5, color='#ff6969', label='other')
  ax1.plot(our_sec[-1], our_speed[-1], '*', markerfacecolor='#9467bd', markeredgecolor='#cf4dff', markersize=20)
  ax1.plot(other_sec[-1], other_speed[-1], '*', markerfacecolor='#ff6969', markeredgecolor='#f50505', markersize=20)
  ax1.set_xlabel('time[sec]', weight='bold')
  ax1.set_ylabel('[m/s]', weight='bold')

  ax2.set_title("angular speed", weight='bold', fontsize=14, color='#363636')
  ax2.plot(our_sec, our_rot, '--', lw=1.5, color='#9467bd', label='ours')
  ax2.plot(other_sec, other_rot, lw=1.5, color='#ff6969', label='other')
  ax2.plot(our_sec[-1], our_rot[-1], '*', markerfacecolor='#9467bd', markeredgecolor='#cf4dff', markersize=20)
  ax2.plot(other_sec[-1], other_rot[-1], '*', markerfacecolor='#ff6969', markeredgecolor='#f50505', markersize=20)
  ax2.set_xlabel('time[sec]', weight='bold')
  ax2.set_ylabel('[rad/sec]', weight='bold')

  # ax1.set_xlim(0,55)
  # ax2.set_xlim(0,55)
  # ax1.set_ylim(-180,180)
  # ax2.set_ylim(-180,180)
  ax1.grid(True)
  ax1.legend(loc='best')
  ax2.grid(True)
  ax2.legend(loc='best')
  fig.tight_layout()
  plt.show()

if __name__ == '__main__':
  main()