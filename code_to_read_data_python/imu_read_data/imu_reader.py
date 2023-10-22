import numpy as np
import pandas as pd

def load_gps_data(i):
#    file = 'data/000035.bin'
    # file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/imu/'+str(i)+'.txt'
    file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/map_10km_post_processed/imu/'+str(i)+'.txt'
    imu_string = np.loadtxt(file, dtype=str)

    imu_message = imu_string.astype(np.float)
    #imu_message = imu_message.reshape(-1, 1)

    ## Format is:
    ## 0. Timestamp in us

    # Rotation measurements
    ## 1. Roll (in degrees)
    ## 2. Pitch (in degrees)
    ## 3. Yaw (in degrees)

    ## 4. Quaternion X
    ## 5. Quaternion Y
    ## 6. Quaternion Z
    ## 7. Quaternion W

    # Gyro measurements
    ## 8. Angular speed around X-axis (in rad/sec)
    ## 9. Angular speed around Y-axis (in rad/sec)
    ## 10. Angular speed around Z-axis (in rad/sec)

    # Heading
    ## 11. Heading Type. 0: True,    1: Magnetic,     2: Unknown
    ## 12. Heading. Heading of the IMU measured in respect to the ENU system [degrees], i.e., compass.

    # Accelerometer measurements
    ## 13. Acceleration in X-axis in m/s2
    ## 14. Acceleration in Y-axis in m/s2
    ## 15. Acceleration in Z-axis in m/s2

    # Magnetometer measurements
    ## 16. Acceleration in X-axis in m/s2
    ## 17. Acceleration in Y-axis in m/s2
    ## 18. Acceleration in Z-axis in m/s2

    ## For the above format, if a value of "-999" is found, then the specific measurement is invalid

    # print ("imu_message is: ", imu_message)
    # print ("imu_message.shape is: ", imu_message.shape)
    # print ("\n")

    imu_message = imu_message.reshape(-1, 19) # reshaping the array to the assumed shape

    # Extract required measurements
    imu_df = pd.DataFrame(imu_message, columns=['timestamp','roll', 'pitch', 'yaw', 'quaternion_x', 'quaternion_y', 'quaternion_z', 'quaternion_w', 
                                               'roll_rate', 'pitch_rate', 'yaw_rate', 'heading_type', 'heading',
                                               'ax', 'ay', 'az', 'mx', 'my', 'mz'])
    
    # Select required columns
    selected_imu_df = imu_df[['timestamp', 'roll', 'pitch', 'yaw', 'roll_rate', 'pitch_rate', 'yaw_rate', 'ax', 'ay', 'az']]
    
    # Rename columns
    selected_imu_df.columns = ['timestamp', 'imu_roll', 'imu_pitch', 'imu_yaw', 'imu_roll_rate', 'imu_pitch_rate', 'imu_yaw_rate', 'imu_ax', 'imu_ay', 'imu_az']
    
    # Write to CSV
    selected_imu_df.to_csv('imu_data_new.csv', index=False)

    return selected_imu_df

# # Define an empty DataFrame to concatenate all the data
# all_imu_data = pd.DataFrame()
# # Iterate through the IMU files
# # for imu_id in range(69072):
# for imu_id in range(193198):
#     imu_data = load_gps_data(imu_id)
#     # Concatenate this IMU data with the overall DataFrame
#     all_imu_data = pd.concat([all_imu_data, imu_data])
# # Save the final concatenated DataFrame to CSV
# all_imu_data.to_csv('all_imu_data_new.csv', index=False)

# Set the chunk size
chunk_size = 69072
# Calculate the number of chunks
num_chunks = (193198 + chunk_size - 1) // chunk_size
# Process each chunk
for chunk in range(num_chunks):
    # Calculate the start and end index for each chunk
    start = chunk * chunk_size
    end = min((chunk + 1) * chunk_size, 193198)

    # Define an empty DataFrame to concatenate all the data in the current chunk
    all_imu_data = pd.DataFrame()

    # Iterate through the IMU files in the current chunk
    for imu_id in range(start, end):
        imu_data = load_gps_data(imu_id)

        # Concatenate this IMU data with the overall DataFrame for the current chunk
        all_imu_data = pd.concat([all_imu_data, imu_data])

    # Save the concatenated DataFrame for the current chunk to CSV
    all_imu_data.to_csv(f'all_imu_data_new{chunk}.csv', index=False)


