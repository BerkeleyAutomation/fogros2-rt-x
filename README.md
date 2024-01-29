
# fogros2-rt-x

This repository contains the code for the fogros2-rt-x project. It is designed to work with ROS and TensorFlow for data collection. Note that this is a very early prototype that hacked within a week. Please submit any issues/bugs through github issues. Also checkout https://github.com/KeplerC/fogros2-rt-x/issues/1 for known issues and upcoming features. 

### Current Workflow 
Currently, it works as following:
1. (User) provides a series of rosbags (preferably in ROS2). We assume one rosbag contains one episode / trajactory. 
2. FogROS2-RT-X stores the data with a private relational database locally. By default, we use sqlite, but this can be easily adjusted. 
3. User may visualize, adjust, edit the data with the relational database (with FogROS, SQL or web interface)
4. To export the dataset to researchers worldwide, user specifies desired observation / action topic names, and desired way of orchestrate the 
 observation-action pair; FogROS2-RT-X library provides helper functions and utilities to streamline the process. 
5. FogROS2-RT-X exports the dataset in standard RT-X data format and shareable with researchers worldwide. 

* Note #1: this doesn't reflect the ultimate architecture. We welcome all the archtectual suggestions to move foward. 
* Note #2: especially on step 3, we are interested in what our beta tester's needs. A few ideas we can think of: adding a column; fill in the values programmically without knowing how to use sql; truncating the episode; 

## Installation 
### Steps
0. Setting environment variables
```
export FOG_WS=~/fog_ws # desired location for FogROS2 ROS2 workspace
export ROS_DISTRO=humble
```
1. Install ROS2 following the instructions on the official ROS2 website.

2. Install python dependencies.
```
apt-get install libgmp-dev sqlite3 ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-ament-cmake-nose ros-$ROS_DISTRO-rosbag2
pip install imageio tensorflow envlogger[tfds] numpy transforms3d
```
It's not recommended to use conda environment. It [does not work well](https://docs.ros.org/en/foxy/How-To-Guides/Using-Python-Packages.html) with ROS/ROS2.

3. clone the repo
```
mkdir -p $FOG_WS
cd $FOG_WS
git clone https://github.com/KeplerC/fogros2-rt-x.git
git clone https://github.com/Box-Robotics/ros2_numpy
```

4. Compile the repo
```
cd $FOG_WS
colcon build
source install/setup.bash
```

#### Google Cloud Setup (Optional)

1. Install google cloud command line tools with the following link: [Google Cloud SDK](https://cloud.google.com/sdk/docs/install#deb)
```
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo gpg --dearmor -o /usr/share/keyrings/cloud.google.gpg
echo "deb [signed-by=/usr/share/keyrings/cloud.google.gpg] https://packages.cloud.google.com/apt cloud-sdk main" | sudo tee -a /etc/apt/sources.list.d/google-cloud-sdk.list
sudo apt-get update && sudo apt-get install google-cloud-cli
```
2. Login to your Google Cloud account using the command line:
    ```
    gcloud auth login
    ```


### Using FogROS2-RT-X 

#### Data Collection 

1. Collect data with standard [rosbag](https://wiki.ros.org/rosbag)/[ros2bag](https://github.com/ros2/rosbag2). For example, if you want to record all topics, simply run 
```
ros2 bag record -a
```
with your standard ROS2 applications. We assume one rosbag-per-episode.

2. Load the collected rosbags with FogROS2 with 
```
ros2 fgr load --dataset_dir=./datasets --dataset_name="berkeley_fanuc_manipulation"
```
Here `dataset_dir` is the directory that holds the rosbags, ideally you want a directory that holds all rosbags. `dataset_name` is how you want to name the collected dataset. 

#### Data Management 
1. You may manage the collected dataset with 
```
sqlite_web ./metadata.db
```
(TODO: streamline this)

#### Data Sharing 
1. To share the data,  first specify the desired observation / action topic names, and desired way of orchestrate the topics by editing the export.py. Currently, FogROS2-RT-X creates a `step` based on given period of time through `PerPeriodTopicOrchestrator`. 
Implementation can be found in [orchestrator_base.py](./fogros2-rt-x/fogros2_rt_x/plugins/orchestrator_base.py). 
You can implement your own policy of orchestrating different topics by inheriting the base class. 


2. Run 
```
ros2 fgr export --dataset_name="berkeley_fanuc_manipulation"
```

#### Replaying with existing datasets in Open-X-Embodiment
You can replay existing datasets in ROS2 with 
```
source install/setup.bash
ros2 run fogros2_rt_x replayer --ros-args -r dataset_name:=$DATASET_NAME 
# replace with yours, e.g. berkeley_fanuc_manipulation
```

## Contributing

Feel free to raise any concerns and bugs through the [issue](https://github.com/KeplerC/fogros2-rt-x/issues) portal, and submit PR with your own edits. 

## License

Apache 2.0
