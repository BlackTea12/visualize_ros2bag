import matplotlib.pyplot as plt
import matplotlib.animation as animation
from functools import partial

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
  ax.set_xlim(3, 8) #(-1,5)
  ax.set_ylim(10,18) #(-7,-2.5)
  robot_traj, = ax.plot([], [], '*', lw=2, color='#9467bd', label='robot trajectory')
  path_traj, = ax.plot([], [], lw=3, color='#ff6969', alpha=0.7, label='follow path')
  ax.legend()
  rb_x = [x for x,y,deg in robot]
  rb_y = [y for x,y,deg in robot]
  path_x = [x for x,y in path]
  path_y = [y for x,y in path]

  # post process (optional)
  rb_x = rb_x[:1000]
  rb_y = rb_y[:1000]
  path_x = path_x[:1000]
  path_y = path_y[:1000]

  # Create the animation
  ani = animation.FuncAnimation(
    fig, partial(video_plot, x1=rb_x, y1=rb_y, line1=robot_traj, x2=path_x, y2=path_y, line2=path_traj)
    , frames=len(rb_x), init_func=partial(video_plot_init, line1=robot_traj, line2=path_traj)
    , blit=True
  )

  # Save the animation as an mp4 file
  # ani.save('/home/hd/main_ws/camera_slam_xyplane.mp4', writer='ffmpeg', fps=1/0.05)
  ani.save('/home/hd/mid_scenario_video.mp4', writer='ffmpeg', fps=1/0.05)

  print("file saved!")

