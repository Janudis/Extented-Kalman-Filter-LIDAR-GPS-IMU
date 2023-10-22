import pandas as pd
import numpy as np
# Read the two CSV files
# df_points = pd.read_csv('median_points.csv')
# df_points = pd.read_csv('clustered_points.csv')
# df_points = pd.read_csv('clustered_points2.csv')
# df_timestamps = pd.read_csv('lidar_timestamps.csv')
df_points = pd.read_csv('clustered_points_new.csv')
# df_points = pd.read_csv('clustered_points_new2.csv')
df_timestamps = pd.read_csv('lidar_timestamps_new.csv')

# Merge the two DataFrames on pcd_id and id columns
merged_df = pd.merge(df_points, df_timestamps, left_on='pcd_id', right_on='id')

# Reorder columns to include timestamp, x, y, z, id
# final_df = merged_df[['timestamp', 'x', 'y', 'z', 'distance', 'id']]
final_df = merged_df[['timestamp', 'x', 'y', 'z', 'id']]

# Save the final DataFrame to a new CSV file
# final_df.to_csv('merged_data.csv', index=False)
# final_df.to_csv('merged_data2.csv', index=False)
final_df.to_csv('merged_data_new.csv', index=False)
# final_df.to_csv('merged_data_new2.csv', index=False)

# Read the merged CSV file
# merged_df = pd.read_csv('merged_data.csv')
# merged_df = pd.read_csv('merged_data2.csv')
merged_df = pd.read_csv('merged_data_new.csv')
# merged_df = pd.read_csv('merged_data_new2.csv')

# # Group by the timestamp and keep the row with the smallest distance
# final_df = merged_df.loc[merged_df.groupby('timestamp')['distance'].idxmin()]
# Sort the DataFrame by timestamp in ascending order
final_df.sort_values(by='timestamp', inplace=True)

# # Drop the 'distance' column as it's no longer needed
# final_df = final_df.drop(columns=['x'])
# # Drop the 'distance' column as it's no longer needed
# final_df = final_df.drop(columns=['y'])
# Drop the 'distance' column as it's no longer needed
# final_df = final_df.drop(columns=['z'])
# Drop the 'distance' column as it's no longer needed
final_df = final_df.drop(columns=['id'])

# Save the final DataFrame to a new CSV file
# final_df.to_csv('distance_data.csv', index=False)
# final_df.to_csv('distance_data2.csv', index=False)
final_df.to_csv('distance_data_new.csv', index=False)
# final_df.to_csv('distance_data_new2.csv', index=False)

