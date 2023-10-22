import numpy as np
import csv
import pandas as pd

def load_gps_data(i):
    # file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/gps/'+str(i)+'.txt'
    # file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/HAV_Lidar_test/Recordings_with_cones/rec3/post_processed_data_capture_cones_rev/gps/'+str(i)+'.txt'
    file = 'D:/Python_Projects/Extented-Kalman-Filter/fusion_camera_gps_imu_without_Eigen/map_10km_post_processed/gps/'+str(i)+'.txt'

    ## Format is:
    ## 0. Timestamp in us
    ## 1. Latitude in WGS84
    ## 2. Longtitude in WGS84
    ## 3. Altitude in meters
    ## 4. Course to magnetic North, in degrees
    ## 5. Speed (horizontal speed in m/s)
    ## 6. Climb (vertical speed in m/s)
    ## 7. HDOP (horizontal dilution of precision)
    ## 8. VDOP (vertical dilution of precision)
    ## 9. HACC (Horizontal accuracy, in meters)
    ## 10. VACC (Vertical accuracy, in meters)
    ## For the above format, if a value of "-999" is found, then the specific measurement is invalid

    # print ("gps_message is: ", gps_message)
    # print ("gps_message.shape is: ", gps_message.shape)
    # print ("\n")
    
    # Read the file line by line, converting each line to float
    with open(file, 'r') as f:
        gps_data = [float(line.strip()) for line in f.readlines()]

    # Reshape the data into a 2D array with 11 columns
    gps_data = np.array(gps_data).reshape(-1, 11)

    # Extract the required columns
    gps_rows = [[row[0], row[1], row[2], row[3], row[5]] for row in gps_data]

    # Append the rows to the existing CSV file, creating the file if it doesn't exist
    with open('gps_data_new.csv', 'a', newline='') as f:
    # with open('gps_cones_data.csv', 'a', newline='') as f:
        writer = csv.writer(f)
        if f.tell() == 0:
            writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude', 'speed']) # Write header if file is empty
        writer.writerows(gps_rows)

# for i in range(0, 2291): # Adjust the range according to your files
for i in range(1, 7728): # Adjust the range according to your files
    load_gps_data(i)


