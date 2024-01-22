
# fogros2-rt-x

This repository contains the code for the fogros2-rt-x project. It is designed to work with ROS and TensorFlow for data collection. Note that this is a very early prototype that hacked within a week. Please submit any issues/bugs through github issues. Also checkout https://github.com/KeplerC/fogros2-rt-x/issues/1 for known issues and upcoming features. 

### Current Workflow 
Currently, it works as following:
1. (User) configures what topics should be subscribed and how the topics are orchstrated 
as observation-action pair; FogROS2-RT-X library provides helper functions and utilities to streamline the process. 
2. FogROS2-RT-X stores the data with a private relational database locally, or on the edge, or on the cloud. By default, we use sqlite, but this can be easily adjusted. 
3. (User) manages all the collected data through the private relational database(e.g. chooses what dataset shall be exported to public). 
4. FogROS2-RT-X exports the dataset in standard RT-X data format. 

Note that this doesn't reflect the ultimate architecture. We welcome all the archtectual suggestions to move foward. 

## Installation 
### Steps
0. Setting environment variables
```
export FOG_WS=~/fog_ws # desired location for FogROS2 ROS2 workspace
export DATASET_NAME=fogros_rt_x_example # name of the dataset
```
1. Install ROS2 following the instructions on the official ROS2 website.
2. Install python dependencies.
```
apt-get install libgmp-dev sqlite3 ros-humble-tf-transformations ros-humble-ament-cmake-nose
pip install google-cloud-bigquery tensorflow envlogger[tfds] numpy transforms3d
```
It's not recommended to use conda environment. It [does not work well](https://docs.ros.org/en/foxy/How-To-Guides/Using-Python-Packages.html) with ROS/ROS2.
3. clone the repo
```
mkdir -p $FOG_WS
cd $FOG_WS
git clone https://github.com/KeplerC/fogros2-rt-x.git
```
4. Copy and edit the [configuration File](./fogros2-rt-x/fogros2_rt_x/plugins/$DATASET_NAME.py) for your own dataset.
For example,
```
cp $FOG_WS/fogros2-rt-x/fogros2_rt_x/plugins/template.py $FOG_WS/fogros2-rt-x/fogros2_rt_x/plugins/$DATASET_NAME.py 
```

7. Compile the repo
```
cd $FOG_WS
colcon build
source install/setup.bash
```

8. Generate ROS2 message files for the tensorflow dataset types and re-compile the repo
```
ros2 fgr config --dataset=$DATASET_NAME
colcon build
```
Note that `DATASET_NAME` should match the file name under `plugins` directory. 

9. Setup google cloud with Google Cloud Setup instructions (see below)

#### Google Cloud Setup

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
3. (To be Automated) Create a Google Storage bucket through the Google Cloud Console and update the bucket name to your configuration file.

## Usage 
#### Dataset Collector
Run dataset generator (stores ROS2 message as RLDS format) with the following instructions
```
source install/setup.bash
ros2 launch fogros2_rt_x data_collector.py
```
This saves all the collected data through a local sqlite file `fogros_rt_x.db`

#### Dataset Export
The following instruction converts all the episode data that `should_export=1` in the database. 
```
ros2 fgr export --dataset_name=$DATASET_NAME
```

#### (Optional) Customize Data Collection Policy
Currently, FogROS2-RT-X creates a `step` based on given period of time through `PerPeriodTopicOrchestrator`. 
Implementation can be found in [orchestrator_base.py](./fogros2-rt-x/fogros2_rt_x/plugins/orchestrator_base.py). 
You can implement your own policy of orchestrating different topics by inheriting the base class. 

#### Replaying with existing datasets in Open-X-Embodiment
You can replay existing datasets in ROS2 with 
```
source install/setup.bash
ros2 run fogros2_rt_x replayer --ros-args -r dataset_name:=$DATASET_NAME 
# replace with yours, e.g. berkeley_fanuc_manipulation
```

#### Data Visualization and Editing 
You may visualize the collected data in the database by querying `fogros_rt_x.db` with tools such as [sqlite-web](https://github.com/coleifer/sqlite-web). You can also replay the dataset in ROS2, and visualize with rviz or foxglove. 


## ROS1 Support 

For ROS1 support, use the [ros1bridge](https://github.com/ros2/ros1_bridge).


## Contributing

Feel free to raise any concerns and bugs through the [issue](https://github.com/KeplerC/fogros2-rt-x/issues) portal, and submit PR with your own edits. 

## License

Apache 2.0
