import matplotlib.pyplot as plt
import math
import random
from matplotlib.animation import FuncAnimation
from numpy import arange
from matplotlib import animation, rc
from IPython.display import HTML

# obstacle type flag
static_only = 0

# Calculate the distance between two points
def distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Generate the neighbors of a point
def neighbors(point):
    x, y = point
    return [(x - 0.5, y - 0.5), (x - 0.5, y), (x - 0.5, y + 0.5),
            (x, y - 0.5), (x, y + 0.5),
            (x + 0.5, y - 0.5), (x + 0.5, y), (x + 0.5, y + 0.5)]

# Check if a point is inside a polygon
def point_in_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        x_inters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= x_inters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

# Check if obstacle is present in between new node and nearby node
def obstacle_in_btw_points(new_node, nearby_node, obstacle):
    obstacle_in_btw = False
    dist = distance(new_node, nearby_node)
    dx = new_node[0] - nearby_node[0]
    dy = new_node[1] - nearby_node[1]
    theta = math.atan2(dy, dx)
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    for pt in arange(0, dist, 0.05):
        if point_in_polygon((new_node[0] + pt*cos_theta, new_node[1] + pt*sin_theta), obstacle):
            obstacle_in_btw = True
        if point_in_polygon((new_node[0] - pt*cos_theta, new_node[1] - pt*sin_theta), obstacle):
            obstacle_in_btw = True
    return obstacle_in_btw

def slope(point1, point2):
  dx = point1[0] - point2[0]
  dy = point1[1] - point2[1]
  slope = math.atan2(dy, dx)
  return slope

from matplotlib.colors import LinearSegmentedColormap

# Generate a path from start to goal avoiding static and dynamic obstacles
def generate_path(start=(2,2), goal=(8,6), static_obstacles=None, dynamic_obstacles=None, frame=0):
  max_dist = 1
  node = start
  #print(f'Start point: {start}')

  # Check if the goal is reached
  if node == goal:
    return goal
  # Check if the last node is close to the goal
  elif distance(goal, node) < max_dist:
    return goal 
  else:
    #print(f'Goal not reached - new node = start = {node}')

    # Get the neighbouring points of the node
    neighboring_points = neighbors(node)
    #print(f'Neighboring point of new node:{neighboring_points}')
    
    neigh_count = 0
    for neighbor in neighboring_points:
      # Check if the neighbouring point is within static obstacle
      if static_obstacles != None:
        stat_obs = 0
        for obstacle in static_obstacles:
          if point_in_polygon(neighbor, obstacle):
            #print(f'Neighboring point {neigh_count} of New node is within static obstacle {stat_obs}')
            point_inside_obstacle = True
            break # breaking out of the obstacle for loop
          else:
            #print(f'Neighboring point {neigh_count} of New node is ouside static obstacle {stat_obs}')
            point_inside_obstacle = False
          stat_obs = stat_obs+1
      
      if point_inside_obstacle == True:
        break # breaking out of the neighboring points for loop
      elif point_inside_obstacle == False:
        # Check if the neighbouring point is within dynamic obstacle
        if dynamic_obstacles != None:
          dyn_obs=0
          for obstacle in dynamic_obstacles:
            x, y = get_dynamic_obstacle_location(obstacle, frame+1)
            if distance(neighbor, (x[0], y[0])) < 1.5:
              point_inside_obstacle = True
              #print(f'Neighboring point {neigh_count} of New node is within dynamic obstacle {dyn_obs} for frame+1={frame+1}')
              break
            else:
              #print(f'Neighboring point {neigh_count} of New node is ouside dynamic obstacle {dyn_obs} for frame+1={frame+1}')
              dyn_obs = dyn_obs + 1

      neigh_count = neigh_count + 1
      if point_inside_obstacle == True:
        break # breaking out of the neighboring points for loop
    
    if point_inside_obstacle == False:
      #print(f'None of the neighboring points are within any of the obstacles')
      # Find the neigboring point which is at minmum distance to the goal
      min_dist = float('inf')
      neigh_count = 0
      for neighbor_min_dist in neighboring_points:
        if distance(neighbor_min_dist, goal) < min_dist:
          min_dist = distance(neighbor_min_dist, goal)
          new_min_dist_node = neighbor_min_dist
        #print(f'Neighboring point {neigh_count} : {neighbor_min_dist} is at distance {distance(neighbor_min_dist, goal)}')
        neigh_count = neigh_count + 1
      #print(f'Neighboring point : {neighbor_min_dist} is at min distance {min_dist}')
      #print(f'New Node: {new_min_dist_node}')
      return new_min_dist_node

    elif point_inside_obstacle == True:
      neighbors_not_in_obs = []
      # Check which neighboring points are not within static obstacle
      neighbor_stat_count = 0
      for neighbor_stat in neighboring_points:
        if static_obstacles != None:
          for obstacle in static_obstacles:
                if not point_in_polygon(neighbor_stat, obstacle):
                  neigbour_point_in_statobs = False
                  #print(f'Neighboring point {neighbor_stat} is outside static obstacle {neighbor_stat_count} . Appended!')
                else:
                  neigbour_point_in_statobs = True
                  break
          if neigbour_point_in_statobs == False:
            neighbors_not_in_obs.append(neighbor_stat)
        neighbor_stat_count = neighbor_stat_count + 1
      #print(f'Number of neighboring point outside static obstacle: {len(neighbors_not_in_obs)}')

      # Check which neighboring points are not within dynamic obstacle
      neighbor_dyn_count = 0
      for neighbor_dyn in neighbors_not_in_obs:
        if dynamic_obstacles != None:
          for obstacle in dynamic_obstacles:
            x, y = get_dynamic_obstacle_location(obstacle, frame+1)
            if distance(neighbor_dyn, (x[0], y[0])) < 1.5:
              #print(f'Neighboring point {neighbor_dyn} is inside dynamic obstacle {neighbor_dyn_count} .Removed!')
              neighbors_not_in_obs.remove(neighbor_dyn)
        neighbor_dyn_count = neighbor_dyn_count +1 
      #print(f'Number of neighboring point outside both static and dynamic obstacle: {len(neighbors_not_in_obs)}')

      # Imposing some conditions to move away from obstacle
      slope_get_pos = {}
      slope_get_neg = {}
      for neighbor_nn_obs in neighbors_not_in_obs:
        slope_get = slope(neighbor_nn_obs, node)
        if slope_get >= 0:
          slope_get_pos[neighbor_nn_obs] = slope_get
        else:
          slope_get_neg[neighbor_nn_obs] = slope_get
      #print(f'Slope dictionary of neighboring points : {slope_get_pos}  ........... {slope_get_neg}')

      if len(slope_get_pos) > 1 :
        return min(slope_get_pos, key = slope_get_pos.get)
      else:
        return min(slope_get_neg, key = slope_get_neg.get)

# Define the start and goal points
start = (12.5, 5)
goal = (-2.5, 5)
path = [start]
flag = 1

# Define the static obstacles as a list of polygons
static_obstacles = [
    [(2, 2), (2, 8), (3, 8), (3, 3), (8, 3), (8, 2)],
    [(6, 6), (7, 6), (7, 7), (6, 7)]
]


# Define the dynamic obstacles as a list of points
dynamic_obstacles = [
    {'initial_position': [
        (10, 1)], "velocity": [random.uniform(-1, 1), random.uniform(-1, 1)]},
    {'initial_position': [
        (2.5, 10)], "velocity": [random.uniform(-1, 1)*.5, random.uniform(-1, 1)*.5]},
    {'initial_position': [
        (5, 5)], "velocity": [random.uniform(-1, 1)*.2, random.uniform(-1, 1)*.2]},
    {'initial_position': [
        (0, 2.5)], "velocity": [random.uniform(-1, 1)*.1, random.uniform(-1, 1)*.1]}
]

# Define functions to plot obstacles
def plot_polygon(polygon, color):
    x, y = zip(*polygon)
    axes.fill(x, y, color=color)


def get_dynamic_obstacle_location(obstacle, frame):
    point = obstacle['initial_position']
    velocity = obstacle['velocity']
    vx, vy = velocity[0], velocity[1]
    x = [i[0] + frame*vx for i in point]
    y = [i[1] + frame*vy for i in point]
    return x, y


fig = plt.figure(figsize=(5, 5))
axes = fig.add_subplot(111)
plt.xlim(-5, 15)
plt.ylim(-5, 15)
plt.xlabel('X')
plt.ylabel('Y')

dynamic_obstacles_location = []

for i, obstacle in enumerate(dynamic_obstacles):
    point, = axes.plot([], [], 'ok', ms=20)
    dynamic_obstacles_location.append(point)


for i, obstacle in enumerate(static_obstacles):
    plot_polygon(obstacle, 'darkgray')


def update_animation(frame):
    # update dynamic obstacles
    #print(f'Frame:{frame}')
    for i, obstacle in enumerate(dynamic_obstacles):
        x, y = get_dynamic_obstacle_location(obstacle, frame+1)
        dynamic_obstacles_location[i].set_data(x, y)

    # TODO: you may compute the path here!
    global path
    global flag
    if frame != 0 :
      next_node = generate_path(path[-1], goal, static_obstacles, dynamic_obstacles, frame)
      if next_node == goal:
        if flag == 1:
          print(f'Goal Reached after {frame} frames (timesteps)!')
          flag = 0
      else :    
        path.append(next_node)
      #print(f'path : {path}')
    
      

    # Plot the path as a red line up to the current frame
    x = [i[0] for i in path[:frame+1]]
    y = [i[1] for i in path[:frame+1]]
    #print(f'Path x: {x}')
    #print(f'Path y: {y}')
    axes.plot(x, y, color='red')

    # Plot the start and goal points as green and blue circles, respectively
    axes.scatter(start[0], start[1], color='green', s=100)
    axes.scatter(goal[0], goal[1], color='blue', s=100)
    return []

# Create the animation using FuncAnimation
animation = FuncAnimation(fig, update_animation, frames=70, interval=250, blit=True)

'''
# Show the plot
plt.tight_layout()
plt.show()
animation
'''

# To run the animation on Google Colab
from matplotlib import rc
rc('animation', html='html5')

animation