
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

class UserSingleMatplot:
  def __init__(self, path) -> None:
    '''
    @path [[timestamp0, Path0], ...]
    '''
    # data save
    self.paths = [p for t,p in path]
    self.idx = -1
    self._saved_idxes = []
    return

  def _on_next_bt(self, event):
    self.idx += 1
    if self.idx == len(self.paths):
      self.idx = 0
    self._draw_path(self.idx)

  def _on_save_bt(self, event):
    self._save_index()

  def _draw_path(self, idx):
    x = [p.pose.position.x for p in self.paths[idx].poses]
    y = [p.pose.position.y for p in self.paths[idx].poses]
    self.ax.lines.clear()
    self.ax.plot(x, y)
    self.ax.set_title(str(idx)+'th path in DB')
    # self.ax.set_xlabel('X[m]')
    # self.ax.set_ylabel('Y[m]')
    # self.ax.set_xlim((3,10))
    # self.ax.set_ylim((10,15))
    plt.draw()

  def _save_index(self):
    self._saved_idxes.append(self.idx)
    if len(self._saved_idxes) == 2:
      plt.close()

  def check_paths(self):
    # single plot
    self.fig, self.ax = plt.subplots()
    plt.subplots_adjust(bottom=0.2)

    # button setting
    button_ax = plt.axes([0.7, 0.05, 0.1, 0.075])
    button_next = Button(button_ax, 'Next')
    button_next.on_clicked(self._on_next_bt)
    button_ax = plt.axes([0.2, 0.05, 0.1, 0.075])
    button_save = Button(button_ax, 'Save')
    button_save.on_clicked(self._on_save_bt)
    self.ax.set_xlabel('X[m]')
    self.ax.set_ylabel('Y[m]')
    # self.ax.set_xlim((3,8))
    # self.ax.set_ylim((11,15))
    plt.show()

  def get_selected_idx(self):
    return self._saved_idxes