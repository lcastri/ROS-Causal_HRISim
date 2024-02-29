import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml
import os

MAP_DIR = os.path.expanduser('~/git/ROS-Causal_HRISim/utilities_ws/src/trajectory_plot/maps/')
MAP_NAME = 'inb3235_small'
TRAJ_CSV = os.path.expanduser('~/git/ROS-Causal_HRISim/utilities_ws/src/causal_discovery_offline/data/data_20240229_174258.csv')
SEL_ID = '1000_' # NOTE: set '' if not needed
from shapely.geometry import Polygon, Point


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

# MAP_BOUNDARIES = [(5.04, -5.28), (-1.156, -0.189), (1.92, 3.03), (7.88, -1.76)]
# MAP = Polygon(MAP_BOUNDARIES) 
# x, y = MAP.exterior.xy
# plt.plot(x, y)

# Step 4: Plot Trajectories
plt.plot(trajectory_data['r_x'].values, trajectory_data['r_y'].values, label=f'Robot')
plt.plot(trajectory_data['h_' + SEL_ID + 'x'].values, trajectory_data['h_' + SEL_ID + 'y'].values, label=f'Human')

# Step 5: Plot goals
GOAL_LIST = [(3.5, -2.5), (-0.295, 0.386), (1.878, 2.371), (7.069, -1.907)]
x_values = [point[0] for point in GOAL_LIST]
y_values = [point[1] for point in GOAL_LIST]
plt.scatter(x_values, y_values, s=100, color='green', zorder=2)

# Add labels and legend
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trajectories over Map')
plt.legend()
# Set axis limits
plt.xlim(-1, 8.5)  # Set x-axis limits from 0 to 6
plt.ylim(-6, 4)  # Set y-axis limits from 0 to 12

# Show plot
plt.show()
