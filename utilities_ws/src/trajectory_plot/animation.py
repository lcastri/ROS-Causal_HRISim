import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg
import yaml
import os

# Load map information
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
fig, ax = plt.subplots()
ax.imshow(map_image, extent=(origin_x, origin_x + len(map_image[0]) * resolution, 
                               origin_y, origin_y + len(map_image) * resolution),
           cmap='gray')

# Load trajectory data
trajectory_data = pd.read_csv(TRAJ_CSV)

# Define goals
GOAL_LIST = [(3.5, -2.5), (-0.295, 0.386), (1.878, 2.371), (7.069, -1.907)]

# Initialize the line objects for robot and human trajectories
robot_line, = ax.plot([], [], 'o-', color='orange', label='TIAGo')
human_line, = ax.plot([], [], 'o-', color='blue', label='Human')

# Initialize the scatter plot for goals
goals_scatter = ax.scatter([], [], s=100, color='green', zorder=2)

# Set the threshold for disappearing points (in number of frames)
threshold = 15  # Adjust as needed

# Function to update the plot for animation
def update_plot(frame):
    start_index = max(0, frame - threshold)  # Starting index for the trajectory

    
    robot_x = trajectory_data['r_x'].iloc[start_index:frame].values
    robot_y = trajectory_data['r_y'].iloc[start_index:frame].values
    
    human_x = trajectory_data['h_' + SEL_ID + 'x'].iloc[start_index:frame].values
    human_y = trajectory_data['h_' + SEL_ID + 'y'].iloc[start_index:frame].values

    
    # Determine colors for each point based on the 'Interp' values
    interp_values = trajectory_data['Interp'].iloc[start_index:frame].values
    human_colors = ['blue' if interp == 0 else 'gray' for interp in interp_values]
    
    # Update the color of each point in the human trajectory
    for i in range(len(human_x)):
        human_line.set_color(human_colors[i])
    
    robot_line.set_data(robot_x, robot_y)
    human_line.set_data(human_x, human_y)
    goals_scatter.set_offsets(GOAL_LIST)
    
    return robot_line, human_line, goals_scatter

# Set labels and legend
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trajectories over Map')
plt.legend()

# Set axis limits
plt.xlim(-1, 8.5)
plt.ylim(-6, 4)

# Create the animation
ani = animation.FuncAnimation(fig, update_plot, frames=len(trajectory_data), interval=100, blit=True)

# Show the animation
plt.show()
