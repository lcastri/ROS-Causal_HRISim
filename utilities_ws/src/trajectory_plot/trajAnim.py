import json
from matplotlib.collections import LineCollection
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg
import yaml
import os
from PIL import Image

# Load map information
ANIM_DIR = os.path.expanduser('~/git/ROS-Causal_HRISim/utilities_ws/src/trajectory_plot/')
GOALS_JSON = os.path.expanduser('~/git/ROS-Causal_HRISim/utilities_ws/src/hrisim_postprocessing/info/goals.json')
MAP_DIR = os.path.expanduser('~/git/ROS-Causal_HRISim/utilities_ws/src/trajectory_plot/maps/')
MAP_NAME = 'inb3235_small'
TRAJ_PATH= os.path.expanduser('utilities_ws/src/bag_postprocess_bringup/data/')
AGENT= 'A15'
SEL_ID = '1000_' # NOTE: set '' if not needed
ROBOT_COLOR = 'tab:orange'
HUMAN_COLOR = 'tab:blue'

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
trajectory_data = pd.read_csv(TRAJ_PATH + AGENT + '_traj_interp.csv')

# Define goals
with open(GOALS_JSON) as json_file:
    GOALS = json.load(json_file)
GOALS_LIST = [(g['x'], g['y']) for _, g in GOALS.items()]

# Initialize the line objects for robot and human trajectories
robot_line, = ax.plot([], [], linestyle='-', color=ROBOT_COLOR, label='TIAGo')
robothead_point = ax.scatter([], [], s=50, color=ROBOT_COLOR, zorder=3)
human_line, = ax.plot([], [], linestyle='-', color=HUMAN_COLOR, label=AGENT)
humanhead_point = ax.scatter([], [], s=50, color=HUMAN_COLOR, zorder=3)

# Initialize the scatter plot for goals
goals_scatter = ax.scatter([], [], s=500, color='green', alpha=0.5, zorder=1)

time_text = ax.text(0.02, 0.02, '', transform=ax.transAxes, fontsize=12, verticalalignment='bottom')
start_time = pd.to_datetime(trajectory_data['time'].iloc[0], unit='s') 
end_time = pd.to_datetime(trajectory_data['time'].iloc[-1], unit='s') 
endt = (end_time - start_time).total_seconds()

# Set the threshold for disappearing points (in number of frames)
SECS_TODISPLAY = 1 # [s]
DT = 0.1 # [s]
threshold = int(SECS_TODISPLAY / DT)

# Function to update the plot for animation
def update_plot(frame):
    # Calculate the time difference from the start time
    current_time = pd.to_datetime(trajectory_data['time'].iloc[frame], unit='s')
    dt = (current_time - start_time).total_seconds()

    # Update the time label text
    time_text.set_text(f'Time: {dt:.2f} | {endt:.2f} s')
    print(f'Time: {dt:.2f} | {endt:.2f} s')

    # Create a new scatter plot with just the specific goal position
    colors = list()
    for gid, g in GOALS.items():
        if g['x'] == trajectory_data['h_' + SEL_ID + '{gx}'].iloc[frame] and g['y'] == trajectory_data['h_' + SEL_ID + '{gy}'].iloc[frame]:
            colors.append("red")
        else:
            colors.append("green")
        ax.text(g['x'], g['y'], gid, ha='center', va='center')  # Create text annotation
    goals_scatter.set_offsets(GOALS_LIST)  # Update the position
    goals_scatter.set_color(colors)  # Change the color

    start_index = max(0, frame - threshold)  # Starting index for the trajectory
    
    # Robot line
    robot_x = trajectory_data['r_x'].iloc[start_index:frame].values
    robot_y = trajectory_data['r_y'].iloc[start_index:frame].values
    robot_line.set_data(robot_x, robot_y)
    
    # Human line
    human_x = trajectory_data['h_' + SEL_ID + 'x'].iloc[start_index:frame].values
    human_y = trajectory_data['h_' + SEL_ID + 'y'].iloc[start_index:frame].values
    human_line.set_data(human_x, human_y)
    
    if frame != 0:
        # Robot head
        robot_line_segments = np.array([robot_x, robot_y]).T.reshape(-1, 1, 2)
        robot_line_collection = LineCollection(robot_line_segments, color=ROBOT_COLOR, linewidth=2)
        ax.add_collection(robot_line_collection)
        dot_x, dot_y = robot_x[-1], robot_y[-1]
        robothead_point.set_offsets([[dot_x, dot_y]])
        
        # Human head
        human_line_segments = np.array([human_x, human_y]).T.reshape(-1, 1, 2)
        human_line_collection = LineCollection(human_line_segments, color=HUMAN_COLOR, linewidth=2)
        ax.add_collection(human_line_collection)
        dot_x, dot_y = human_x[-1], human_y[-1]
        humanhead_point.set_offsets([[dot_x, dot_y]])

    return robot_line, human_line, robothead_point, humanhead_point, goals_scatter, time_text

# Set labels and legend
plt.xlabel('X')
plt.ylabel('Y')
plt.title('HRSI: TIAGo - ' + AGENT)
plt.legend()

# Set axis limits
plt.xlim(-1, 8.5)
plt.ylim(-6, 4)

# Create the animation
ani = animation.FuncAnimation(fig, update_plot, frames=len(trajectory_data), interval=10, blit=True, repeat=False)

# Create a directory to save frames
if not os.path.exists('frames'):
    os.makedirs('frames')

# Save frames as images
for i in range(0, len(trajectory_data), 5):
    update_plot(i)
    plt.savefig(f'frames/frame_{i:04d}.png')

# Convert frames to GIF using Pillow
images = []
for i in range(0, len(trajectory_data), 5):
    images.append(Image.open(f'frames/frame_{i:04d}.png'))
images[0].save(ANIM_DIR + 'gif/' + AGENT + '.gif', save_all=True, append_images=images[1:], duration=100, loop=0)


# # Show the animation
# plt.show()