import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml
import os
from shapely.geometry import Polygon, Point



MAP_DIR = os.path.expanduser('~/git/ROS-Causal_HRISim/utilities_ws/src/trajectory_plot/maps/')
MAP_NAME = 'inb3235_small'
TRAJ_CSV = os.path.expanduser('utilities_ws/src/bag_postprocess_bringup/data/A1_traj_interp.csv')
SEL_ID = '1000_' # NOTE: set '' if not needed


with open(MAP_DIR + MAP_NAME + '/map.yaml', 'r') as yaml_file:
    map_info = yaml.safe_load(yaml_file)

# Step 1: Load the PNG Image
map_image = mpimg.imread(MAP_DIR + MAP_NAME + '/map.pgm')

# Step 2: Plot the Map
# plt.imshow(map_image, extent=[-map_image.shape[0]/2, map_image.shape[0]/2, -map_image.shape[1]/2, map_image.shape[1]/2])
# Get resolution and origin from YAML
resolution = map_info['resolution']
origin_x, origin_y = map_info['origin'][:2]

# Plot the map image
plt.imshow(map_image, extent=(origin_x, origin_x + len(map_image[0]) * resolution, 
                               origin_y, origin_y + len(map_image) * resolution),
           cmap='gray')

# Step 3: Load Trajectory Data
trajectory_data = pd.read_csv(TRAJ_CSV)

# MAP_BOUNDARIES = [(5.45, -4.66), (0.75, -0.56), (0.47, -0.73), (-0.73, 0.28), 
#                   (0.01, 1.05), (-0.37, 1.37), (0.69, 2.62), (1.29, 2.14),
#                   (2.01, 2.82), (2.95, 1.82), (3.52, 1.4), (4.16, 1.17), (7.86, -1.85)]
# MAP = Polygon(MAP_BOUNDARIES) 
# x, y = MAP.exterior.xy
# plt.plot(x, y)

# Step 4: Plot Trajectories
# plt.plot(trajectory_data['r_x'].values, trajectory_data['r_y'].values, label=f'Robot')
# plt.plot(trajectory_data['h_' + SEL_ID + 'x'].values, trajectory_data['h_' + SEL_ID + 'y'].values, label=f'Human')

# Step 5: Plot goals
GOAL_LIST = [(3.5, -2.5), (-0.295, 0.386), (1.878, 2.371), (7.069, -1.907)]
x_values = [point[0] for point in GOAL_LIST]
y_values = [point[1] for point in GOAL_LIST]
plt.scatter(x_values, y_values, s=500, color='green', alpha=0.5, zorder=2)
for i, g in enumerate(GOAL_LIST):
    plt.text(g[0], g[1], 'G' + str(i+1), fontsize=12, ha='center', va='center')

# Add labels and legend
plt.xlabel('X')
plt.ylabel('Y')
plt.title(r'$A_1$ participant and robot trajectories')
plt.legend()
# Set axis limits
plt.xlim(-1, 8.5)  # Set x-axis limits from 0 to 6
plt.ylim(-6, 4)  # Set y-axis limits from 0 to 12

# Show plot
plt.show()



