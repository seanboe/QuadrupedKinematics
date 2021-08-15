import matplotlib.pyplot as plt

# A graphing script. Just input the individual points into input.txt and run.
# Don't forget to navigate to this directory when running.

class coordinate:
  def __init__(self):
    self.x = []
    self.y = []
  
  def add_data(self, new_x, new_y):
    self.x.append(new_x)
    self.y.append(new_y)


def run(input_file, coordinate):
  lines = input_file.readlines()
  current_x = 0
  for line in lines:
    line = float(line.strip("\n"))
    coordinate.add_data(current_x, line)
    current_x += 1
  return

if __name__ == "__main__":
  with open('input.txt', 'r') as file:
    graph_coordinates = coordinate()
    run(file, graph_coordinates)
  plt.plot(graph_coordinates.x, graph_coordinates.y)
  plt.show()