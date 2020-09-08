# events_repacking_helper

This package provides an example of preparing a bag file for ESVO. A bag file recorded by a stereo event camera (e.g., a pair of DAVIS sensors) typically consists of the following topics:

	-- /davis/left/events
	-- /davis/right/events

	-- /davis/left/imu
	-- /davis/right/imu

	-- /davis/left/camera_info
	-- /davis/right/camera_info

	-- /davis/left/image_raw (optional for visualization)
	-- /davis/right/image_raw (optional for visualization)

## Preparation

### 1. Extract event messages from the original bag, and change the streaming rate to 1000 Hz.
	
Set the input and output paths as arguments in the file `repacking.launch`, and then run   

   `$ roslaunch events_repacking_helper repacking.launch`

This command will return a bag file (e.g., output.bag.events) which only contains the re-packed stereo event messages.

### 2. Extract other needed topics from the original bag using [srv_tools](https://github.com/srv/srv_tools).

   `$ cd path_to_/srv_tools/bag_tools/scripts`

   `$ python extract_topics.py source_bag_name output_bag_name [topic names]`

This command will return a bag file (e.g., output.bag.extracted) which contains other necessary topics except for events.

### 3. Merge above output bags.

   `$ python merge.py output.bag.events output.bag.extracted --output output_bag_name`

### 4. Remove hot pixels using [DVS_Hot_Pixel_Filter](https://github.com/cedric-scheerlinck/dvs_tools/tree/master/dvs_hot_pixel_filter) (optional).

   `$ rosrun dvs_hot_pixel_filter hot_pixel_filter path_to_input.bag`

This command will generate a bag file named xxx.bag.filtered, which is good to feed to ESVO.
