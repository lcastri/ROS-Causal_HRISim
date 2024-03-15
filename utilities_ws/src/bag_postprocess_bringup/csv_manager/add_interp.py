import pandas as pd

# Load the CSV file
csv_path = "/home/lucacastri/git/ROS-Causal_HRISim/utilities_ws/src/bag_postprocess_bringup/data/"
input_csv_path = "A13_traj_interp.csv"  # Replace with your input CSV file path
output_csv_path = "A13_traj_interp.csv"  # Replace with your desired output CSV file path

# Load CSV file into a DataFrame
df = pd.read_csv(csv_path+ input_csv_path)

# Check for NaN values in each row and create the "Interp" column
# df['Interp'] = df.isnull().any(axis=1).astype(int)

df.interpolate(method='linear', axis=0, inplace=True)
df.bfill(axis=0, inplace=True)

# Write the modified DataFrame with the "Interp" column to a new CSV file
df.to_csv(csv_path + output_csv_path, index=False)