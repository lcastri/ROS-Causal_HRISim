import os
import pandas as pd
import json

GOALS_JSON = os.path.expanduser('~/git/ROS-Causal_HRISim/utilities_ws/src/hrisim_postprocessing/info/goals.json')

# Load the CSV file
csv_path = "/home/lucacastri/git/ROS-Causal_HRISim/utilities_ws/src/bag_postprocess_bringup/data/"

for i in range(1, 16):
    input_csv_path = "A" + str(i) + "_traj_interp.csv"  # Replace with your input CSV file path
    output_csv_path = "A" + str(i) + "_traj_interp.csv"  # Replace with your desired output CSV file path

    # Read the CSV file
    df = pd.read_csv(csv_path + input_csv_path)

    # Load the goal information from the JSON file
    with open(GOALS_JSON) as f:
        goals = json.load(f)

    # Define a function to determine the goal ID based on coordinates
    def get_goal_id(x, y):
        for goal_id, goal_info in goals.items():
            if x == goal_info['x'] and y == goal_info['y']: return goal_id
        return None

    # Apply the function to create the 'goalID' column
    df['goalID'] = df.apply(lambda row: get_goal_id(row['h_1000_{gx}'], row['h_1000_{gy}']), axis=1)

    # Save the modified DataFrame back to the CSV file
    df = df.loc[:, ~df.columns.str.contains('^Unnamed')]
    df.to_csv(csv_path + input_csv_path, index=False)