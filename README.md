
# fogros2-rt-x

This repository contains the code for the fogros2-rt-x project. It is designed to work with ROS and TensorFlow for data collection. Note that this is a very early prototype that hacked within a week. Please submit any issues/bugs through github issues. Also checkout https://github.com/KeplerC/fogros2-rt-x/issues/1 for known issues and upcoming features. 

## Installation 
### Steps

1. Install ROS2 following the instructions on the official ROS2 website.
2. Install python dependencies.
```
pip install google-cloud-bigquery tensorflow envlogger[tfds]
```
Don't use conda environment, which does not work well with ROS/ROS2.
3. clone the repo
```
mkdir -p ~/fog_ws/src
git clone https://github.com/KeplerC/fogros2-rt-x.git
```
4. Setup google cloud with Google Cloud Setup instructions 
5. Edit [Configuration File](./fogros2-rt-x/fogros2_rt_x/dataset_conf.py)
6. (To be automated) Edit [ROS2 Message Definition](./fogros2_rt_x_msgs/msg/) to align with the configuration file
7. compile the repo
```
cd ~/fog_ws
colcon build
```

#### Google Cloud Setup

1. Install google cloud command line tools with the following link: [Google Cloud SDK](https://cloud.google.com/sdk/docs/install#deb)
2. Login to your Google Cloud account using the command line:
    ```
    gcloud auth application-default login
    ```
3. (To be Automated) Create a Google Storage bucket through the Google Cloud Console.
4. (To be Automated) Create a Google BigQuery table XXX.DATASET_NAME.metadata

## Usage 
#### Dataset Generator
Run dataset generator (stores ROS2 message as RLDS format) with the following instructions
```
source install/setup.bash
ros2 run ros2 run fogros2_rt_x recorder
```

#### Replaying with existing datasets in Open-X-Embodiment
You can replay existing datasets in ROS2 with 
```
source install/setup.bash
ros2 run ros2 run fogros2_rt_x replayer
```
TODO: currently it plays the bridge dataset. We have not tested other datasets. 

#### Adapting your own data collection
Currently, it takes the [step](https://github.com/KeplerC/fogros2-rt-x/blob/main/fogros2_rt_x_msgs/msg/Step.msg) message as the input. We are working on automating the generation of step message. With the next iteration, it will be 
1. an Action message (type defined by the user instead of FogROS)
2. multiple Observation topics 
as input, and FogROS2 facilitates the orchestration of the topics.  

## ROS1 Support 

For ROS1 support, use the [ros1bridge](https://github.com/ros2/ros1_bridge).

## File Structure
```
.
├── fogros2_rt_x
    ├── recorder.py
    ├── dataset_utils.py
    ├── dataset_spec.py
    |__ dataset_conf.py # Update this file 
├── fogros2_rt_x_msgs
│   ├── msg
│   │   ├── Step.msg
│   │   ├── Observation.msg
│   │   └── Action.msg
```

## Contributing

Feel free to raise any concerns and bugs through the [issue](https://github.com/KeplerC/fogros2-rt-x/issues) portal, and submit PR with your own edits. 

## License

Apache 2.0
