# Extented-Kalman-Filter-LIDAR_GPS_IMU
Sensor Fusion of LIDAR, GPS and IMU with Extended Kalman Filter for Localization in Autonomous Driving

1) lidar_preprocessing : read and filter lidar pointcloud
2) DBSCAN_clustering : do DBSCAN clustering and return the closest's cluster centroid
3) gps_geodetic_utm : GPS data from geodetic to UTM coordinate system
4) lidar_gt_geodetic_utm : lidar groundtruth landmarks from geodetic to UTM coordinate system
5) data_preprocessing : association of observed landmarks with the ground-truth landmarks (used in main)
   
