import numpy as np

#circle parameters
center_axis = [0, 0]
radius = 30
max_depth = 25
min_depth = 3

#path parameters
n = 12
depth_n = 10
approach_speed = 1.0
traversal_speed = 0.9

#robot parameters
robotCoord = [32, 36, -20]
robotCoord2d = robotCoord[:2]

start_angle = np.arctan(robotCoord2d[1]/robotCoord2d[0])
if robotCoord2d[0] < 0:
    start_angle += np.pi

slices = np.linspace(start_angle, start_angle + 2 * np.pi, n * 2, endpoint=False)
vertical = np.linspace(min_depth, max_depth, depth_n)

def getPathCoord(angle):
    x = center_axis[0] + (radius * np.cos(angle))
    y = center_axis[1] + (radius * np.sin(angle))
    return [x, y]

pathCoords = np.array([getPathCoord(angle) for angle in slices])

""" def getDistance(coord1, coord2):
    return ((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)**0.5
    
pathDistances = [getDistance(robotCoord2d, pathCoord) for pathCoord in pathCoords]
minPointIndex = pathDistances.index(min(pathDistances))

slices = np.concatenate((slices[minPointIndex:], slices[:minPointIndex]))
pathCoords = np.concatenate((pathCoords[minPointIndex:], pathCoords[:minPointIndex])) """

waypoints = []
for i in range(n): #construct main path
    angle_up = slices[i * 2]
    
    x, y = pathCoords[i * 2]
    #x = center_axis[0] + (radius * np.cos(angle_up))
    #y = center_axis[1] + (radius * np.sin(angle_up))

    for v in vertical:
        #waypoints.append([x, y, -v, (angle_up - (np.pi / 2)) % (2 * np.pi)])
        waypoints.append([x, y, -v, 0])
    
    angle_down = slices[(i * 2) + 1]

    x, y = pathCoords[(i * 2) + 1]
    #x = center_axis[0] + (radius * np.cos(angle_down))
    #y = center_axis[1] + (radius * np.sin(angle_down))

    for v in vertical[::-1]:
        #waypoints.append([x, y, -v, (angle_down + (np.pi / 2)) % (2 * np.pi)])
        waypoints.append([x, y, -v, 0])
        
waypoints.append([pathCoords[0][0], pathCoords[0][1], -vertical[0], 0])
waypoints.append([robotCoord[0], robotCoord[1], robotCoord[2], 0])
        
#write waypoints to file
with open('generated_waypoints.yaml', 'w') as f:
    for i in range(len(waypoints)):
        x, y, z, h = waypoints[i]
        f.write(f'''- point: [{x:0.2f}, {y:0.2f}, {z:0.2f}]
  max_forward_speed: {approach_speed if (i == 0 or i == len(waypoints)-1) else traversal_speed}
  heading: {h if (i != 0 and i != len(waypoints)-1) else 0:0.3f}
  use_fixed_heading: {'false' if (i == 0 or i == len(waypoints)-1) else 'true'}
''')
