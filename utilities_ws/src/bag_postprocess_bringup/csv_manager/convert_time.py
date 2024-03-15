import pandas as pd

# Load the CSV file
csv_path = "/home/lucacastri/git/ROS-Causal_HRISim/utilities_ws/src/bag_postprocess_bringup/data/"

for i in range(5, 16):

    input_csv_path = "A" + str(i) + "_traj_interp.csv"  # Replace with your input CSV file path
    output_csv_path = "A" + str(i) + "_traj_interp.csv"  # Replace with your desired output CSV file path

    # Load CSV file into a DataFrame
    df = pd.read_csv(csv_path + input_csv_path)

    # Convert the 'time' column from epoch format to datetime
    tmp_time = pd.to_datetime(df['time'], unit='s')

    # Calculate the time difference in seconds
    df['time_seconds'] = (tmp_time - tmp_time.iloc[0]).dt.total_seconds()

    # Save the modified DataFrame back to the CSV file
    df.to_csv(csv_path + output_csv_path, index=False)