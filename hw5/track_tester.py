import math
import matplotlib.pyplot as plt
import numpy as np

def track2midpoint(box1_loc, box2_loc, gem_startloc, num_points=4):
    goal = ((box1_loc[0]+box2_loc[0])/2, (box1_loc[1]+box2_loc[1])/2) #midpoint of the two boxes

    track_points_x = np.linspace(gem_startloc[0], goal[0], num_points)
    track_points_y = np.linspace(gem_startloc[1], goal[1], num_points)

    theta = math.atan(((goal[1]-gem_startloc[1]) / (goal[0]-gem_startloc[0])))
    print((goal[0]-gem_startloc[0]))
    print((goal[1]-gem_startloc[1]))
    print(float((goal[0]-gem_startloc[0]) / (goal[1]-gem_startloc[1])))
    print(np.degrees(theta))
    track_points_heading = [np.degrees(theta)+90 for i in range(len(track_points_x))]
    if gem_startloc[0] > goal[0]:
      track_points_heading = [x +180 for x in track_points_heading] #accounting for wrapping that happens at 180
    return track_points_x, track_points_y, track_points_heading

def circlepoints(circle_center, gem_startloc, perimeter_point, num_points=20):
    r = math.sqrt((perimeter_point[0]-circle_center[0])**2 + (perimeter_point[1]-circle_center[1])**2)
    starting_t = np.degrees(math.acos((perimeter_point[0] - circle_center[0])/r))
    if perimeter_point[1] < circle_center[1]:
      starting_t *= -1
    
    if gem_startloc[0] < circle_center[0]:
      if perimeter_point[1] > circle_center[1]:
        direction = 0
      else:
        direction = 1
    else:
      if perimeter_point[1] > circle_center[1]:
        direction = 1
      else:
        direction = 0
    
    if direction:
      angles = np.linspace(starting_t, starting_t+360, num_points)
    else:
      angles = np.linspace(starting_t, starting_t-360, num_points)
    
    circle_points_x = []
    circle_points_y = []
    circle_points_heading = []
    for i in range(num_points):
        circle_points_x = np.append(circle_points_x, r*np.cos(np.radians(angles[i]))+circle_center[0])
        circle_points_y = np.append(circle_points_y, r*np.sin(np.radians(angles[i]))+circle_center[1])

        # +90 for forward x direction being 90, plus another 90 to calculated the tangent
        # Counterclockwise, direction = 1, clockwise, direction = 0
        circle_points_heading = np.append(circle_points_heading, angles[i]+(direction*180)) 
    
    return circle_points_x, circle_points_y, circle_points_heading
    
    
if __name__ == '__main__':
  box1 = (12,2)
  box2 = (10,3)
  robot = (18,2)
  line_points_x, line_points_y, line_points_head = track2midpoint(box1, box2, robot)
  circle_x, circle_y, circle_head = circlepoints(box1, robot, (line_points_x[-1], line_points_y[-1]))
  ax = plt.axes()
  plt.plot(box1[0], box1[1], 'o')
  plt.plot(box2[0], box2[1], 'o')
  plt.plot(robot[0], robot[1], 'o')
  plt.plot(circle_x[0], circle_y[0], 'o')
  plt.plot(line_points_x, line_points_y)
  plt.plot(circle_x, circle_y)
  for i in range(len(line_points_x)):
    ax.arrow(line_points_x[i], line_points_y[i], math.cos(np.radians(line_points_head[i]-90)), math.sin(np.radians(line_points_head[i]-90)), head_width=0.1, head_length=0.2)
  for i in range(len(circle_x)):
    ax.arrow(circle_x[i], circle_y[i], math.cos(np.radians(circle_head[i]-90)), math.sin(np.radians(circle_head[i]-90)), head_width=0.1, head_length=0.2)
  plt.xlim(-1, 20)
  plt.ylim(-1, 20)
  plt.gca().set_aspect('equal')
  plt.show()