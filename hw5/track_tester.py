import math
import matplotlib.pyplot as plt
import numpy as np

def track2midpoint(box1_loc, box2_loc, gem_startloc, num_points=4):
    goal = ((box1_loc[0]+box2_loc[0])/2, (box1_loc[1]+box2_loc[1])/2) #midpoint of the two boxes
    print(gem_startloc[0])
    print(goal)
    print( int(goal[0]-gem_startloc[0])//num_points)
    track_points_x = np.linspace(gem_startloc[0], goal[0], num_points)
    track_points_y = np.linspace(gem_startloc[1], goal[1], num_points)
    # track_points_x = np.linspace(gem_startloc[0], goal[0], (goal[0]-gem_startloc[0])/num_points)
    # track_points_y = np.linspace(gem_startloc[1], goal[1], (goal[1]-gem_startloc[1])/num_points)
    theta = math.atan(goal[0] / goal[1])
    track_points_heading = [theta+90 for i in range(len(track_points_x))]        
    return track_points_x, track_points_y, track_points_heading

def circlepoints(circle_center, gem_startloc, num_points=20):
    r = math.sqrt((gem_startloc[0]-circle_center[0])**2 + (gem_startloc[1]-circle_center[1])**2)
    print((circle_center[0]/r))
    print(np.degrees(math.acos(((gem_startloc[0]-circle_center[0])/r))))
    starting_t = np.degrees(math.acos((gem_startloc[0] - circle_center[0])/r))
    if gem_startloc[1] < circle_center[1]:
      starting_t *= -1
    angles = np.linspace(starting_t, starting_t+360, num_points)
    circle_points_x = []
    circle_points_y = []
    circle_points_heading = []
    
    for i in range(num_points):
        circle_points_x = np.append(circle_points_x, r*np.cos(np.radians(angles[i]))+circle_center[0])
        circle_points_y = np.append(circle_points_y, r*np.sin(np.radians(angles[i]))+circle_center[1])
        circle_points_heading = np.append(circle_points_heading, angles[i]+90)
    
    return circle_points_x, circle_points_y, circle_points_heading
    
    
if __name__ == '__main__':
  box1 = (7,6)
  box2 = (3,6)
  robot = (8,12)
  line_points_x, line_points_y, line_points_head = track2midpoint(box1, box2, robot)
  circle_x, circle_y, circle_head = circlepoints(box1, (line_points_x[-1], line_points_y[-1]))
  plt.plot(box1[0], box1[1], 'o')
  plt.plot(box2[0], box2[1], 'o')
  plt.plot(robot[0], robot[1], 'o')
  plt.plot(circle_x[0], circle_y[0], 'o')
  plt.plot(line_points_x, line_points_y)
  plt.plot(circle_x, circle_y)
  plt.xlim(-1, 20)
  plt.ylim(-1, 20)
  plt.gca().set_aspect('equal')
  plt.show()