# ITRI-visualization

## Prepare Data
[ 20201007 DCV (City Day and Night)](https://trello.com/c/oPI4CjUc/71-20201007-dcv-urban-day-and-night)

* Save radar and LiDAR msgs in `.pkl` format:
```
python -B rosbag_save_rav4_single_sensor.py
```

*  Convert radar `.pkl` to `.png`:
```
python -B radar2img.py
```
For further details, please refer to [Oxford robotcar-dataset-sdk](https://github.com/ori-mrg/robotcar-dataset-sdk/blob/master/python/radar.py).

* Save camera:
```
python -B rosbag_camera_timestamp.py
```
```
python -B rosbag_subscribe_camera.py
```
Please run `decoder.sh` first.



## Visualize all
```
rosrun rviz rviz -d sdc-2023/rviz_config.rviz
```
```
python -B sdc-2023/pub_sensors.py
```


## Video

https://youtu.be/tlw3QJ4NaRs  
https://youtu.be/89CzDpNCrdY
