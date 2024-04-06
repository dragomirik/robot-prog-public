import re
import matplotlib.pyplot as plt
import numpy as np

def readValuesCart(filePath):
  x = []
  y = []

  with open(filePath, 'r') as file:
      data = file.read()
      pattern = r'\((-?\d+\.\d+),(-?\d+\.\d+)\)'
      matches = re.findall(pattern, data)

      for match in matches:
          x_value, y_value = match
          x.append(float(x_value))
          y.append(float(y_value))

  return x, y

def readValues(filePath):
  patternSplit = r"\(\d+,\d+\)"
  patternPoint = r"\d+"

  with open(filePath) as file:
      #next(file)
      data = file.read()
  data = re.findall(patternSplit, data)

  angles = []
  distances = []

  for point in data:
      dataPart = re.findall(patternPoint, point)
      angles.append(int(dataPart[0])/100)
      distances.append(int(dataPart[1])/10)
  return angles, distances

def circularPlot(filePath):
  angles, distances = readValues(filePath)
  #print("angles  :", angles)
  #print("distance:", distances)
  fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
  ax.set_theta_direction(-1)
  ax.set_theta_zero_location('N')
  anglesRad = np.radians(angles)
  ax.scatter(anglesRad, distances, s=0.1)
  ax.grid(True)
  plt.show()

def normalPlot(filePath):
  angles, distances = readValues(filePath)
  plt.scatter(angles, distances, s=0.1)
  plt.xlabel("angles")
  plt.ylabel("distances")
  plt.show()

def anglePlot(filePath):
  angles, distances = readValues(filePath)
  plt.scatter(list(range(len(angles))), angles, s=0.1)
  plt.show()

def cartPlot(filePath):
  x, y = readValuesCart(filePath)
  #print("x  :", x)
  #print("y:", y)
  fig, ax = plt.subplots()
  ax.scatter(x, y, s=0.1)
  ax.set_xlabel("X")
  ax.set_ylabel("Y")
  ax.grid(True)
  ax.set_aspect('equal')

  rho_values = [418.46,-939.82,-785.47,356.72]
  theta_values = [0.80,0.89,2.43,0.54]

  draw_lines_from_rho_theta(rho_values, theta_values, ax)

  x_points = [840.71,158.72,-6.16,2225.79]
  y_points = [-230.13,428.45,-1204.34,-3011.74]
  ax.scatter(x=x_points, y=y_points, color='green')

  x_points = [447.81]
  y_points = [-228.96]
  ax.scatter(x=x_points, y=y_points, color='blue')
  
  plt.show()

def draw_lines_from_rho_theta(rho_values, theta_values, ax):
  max_x = 200 

  for i, (rho, theta) in enumerate(zip(rho_values, theta_values)):
      x1 = -max_x if theta < np.pi/2 or theta > 3*np.pi/2 else rho * np.cos(theta) - max_x
      y1 = (rho - x1 * np.cos(theta)) / np.sin(theta)
      x2 = max_x if theta < np.pi/2 or theta > 3*np.pi/2 else rho * np.cos(theta) + max_x
      y2 = (rho - x2 * np.cos(theta)) / np.sin(theta)

      ax.plot([x1, x2], [y1, y2], 'r-', linewidth=1)
      # print(f"x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}, cos: {np.cos(theta)}, sin: {np.sin}")
      
      # Calculer la position du milieu de la ligne
      mid_x = (x1 + x2) / 2
      mid_y = (y1 + y2) / 2
      
      # Ajouter une annotation au milieu de la ligne
      ax.annotate(f'Wall {i+1} (rho={rho}, theta={theta})', (mid_x, mid_y), textcoords="offset points", xytext=(10,10), ha='center')


def parse_file(file_path):
    lidar_points = []
    cartesian_points = []
    walls_rho = []
    walls_theta = []
    corners_x = []
    corners_y = []
    centroid = []

    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            
            if line.startswith("** LIDAR points: "):
                lidar_points_str = line.split(":")[1].strip()
                points_str = lidar_points_str.split(';')
                lidar_points = [(float(coord.strip('()').split(',')[0]), float(coord.strip('()').split(',')[1])) for coord in points_str if coord]
            
            elif line.startswith("** CARTESIAN points: "):
                cartesian_points_str = line.split(":")[1].strip()
                points_str = cartesian_points_str.split(';')
                cartesian_points = [(float(coord.strip('()').split(',')[0]), float(coord.strip('()').split(',')[1])) for coord in points_str if coord]
            
            elif line.startswith("** Walls rho: "):
                walls_rho = list(map(float, line.split(":")[1].strip().split(',')))
            
            elif line.startswith("** Walls theta: "):
                walls_theta = list(map(float, line.split(":")[1].strip().split(',')))
            
            elif line.startswith("** corners x: "):
                corners_x = list(map(float, line.split(":")[1].strip().split(',')))
            
            elif line.startswith("** corners y: "):
                corners_y = list(map(float, line.split(":")[1].strip().split(',')))
            
            elif line.startswith("** centroid: "):
                centroid = list(map(float, line.split(":")[1].strip().split(',')))

    return lidar_points, cartesian_points, walls_rho, walls_theta, corners_x, corners_y, centroid

def lidarDataPlot(cartesian_points, walls_rho, walls_theta, corners_x, corners_y, centroid):
  x = [point[0] for point in cartesian_points]
  y = [point[1] for point in cartesian_points]
  fig, ax = plt.subplots()
  ax.scatter(x, y, s=0.1)
  ax.set_xlabel("X")
  ax.set_ylabel("Y")
  ax.grid(True)
  ax.set_aspect('equal')

  draw_lines_from_rho_theta(walls_rho, walls_theta, ax)

  ax.scatter(x=corners_x, y=corners_y, color='green')
  for i, (x, y) in enumerate(zip(corners_x, corners_y)):
    ax.annotate(f'Corner {i+1}', (x, y), textcoords="offset points", xytext=(10,10), ha='center')

  if centroid:
    ax.scatter(x=centroid[0], y=centroid[1], color='blue')
  
  

  plt.show()

if __name__ == "__main__":
  #anglePlot(r"C:\Users\thoma\Downloads\testProg.txt")
  #circularPlot(r"C:\Users\thoma\Downloads\lidar_data_240318_2224.txt")
  #circularPlot(r"C:\Users\remye\Documents\robot-prog\lidar.log")
  #cartPlot(r"C:\Users\remye\Documents\robot-prog\log_cartesien.txt")
  
  lidar_points, cartesian_points, walls_rho, walls_theta, corners_x, corners_y, centroid = parse_file(r"C:\Users\remye\Documents\robot-prog\lidar_data.txt")
  lidarDataPlot(cartesian_points, walls_rho, walls_theta, corners_x, corners_y, centroid)