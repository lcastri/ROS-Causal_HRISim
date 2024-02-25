import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml

with open('/home/lucacastri/git/ROS-Causal_HRISim/HRISim/hrisim_gazebo/tiago_maps/maze/map.yaml', 'r') as yaml_file:
    map_info = yaml.safe_load(yaml_file)

# Step 1: Load the PNG Image
map_image = mpimg.imread('/home/lucacastri/git/ROS-Causal_HRISim/HRISim/hrisim_gazebo/tiago_maps/maze/map.pgm')

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
trajectory_data = pd.read_csv('/home/lucacastri/git/ROS-Causal_HRISim/utilities_ws/src/causal_discovery_offline/data/raw20240128_160917.csv')

# Step 4: Plot Trajectories
plt.plot(trajectory_data['r_x'].values, trajectory_data['r_y'].values, label=f'Robot')
plt.plot(trajectory_data['h_x'].values, trajectory_data['h_y'].values, label=f'Human')

# Add labels and legend
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trajectories over Map')
plt.legend()
# Set axis limits
plt.xlim(-12, 12)  # Set x-axis limits from 0 to 6
plt.ylim(-12, 12)  # Set y-axis limits from 0 to 12

# Show plot
plt.show()
