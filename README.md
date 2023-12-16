
# fogros2-rt-x

This repository contains the code for the fogros2-rt-x project. It is designed to work with ROS and TensorFlow for data collection. Note that this is a very early prototype that hacked within a week. Please submit any issues/bugs through github issues. Also checkout https://github.com/KeplerC/fogros2-rt-x/issues/1 for known issues and upcoming features. 

## Installation 
### Steps

1. Install ROS2 following the instructions on the official ROS2 website.
2. Install python dependencies.
```
apt-get install libgmp3-dev
pip install google-cloud-bigquery tensorflow envlogger[tfds] numpy
```
Don't use conda environment. It [does not work well](https://docs.ros.org/en/foxy/How-To-Guides/Using-Python-Packages.html) with ROS/ROS2.
3. clone the repo
```
export FOG_WS=~/fog_ws
mkdir -p $FOG_WS
cd $FOG_WS
git clone https://github.com/KeplerC/fogros2-rt-x.git
```
4. Edit [Configuration File](./fogros2-rt-x/fogros2_rt_x/dataset_conf.py)
7. compile the repo
```
cd $FOG_WS
colcon build
source install/setup.bash
```
8. generate ROS2 message files for the tensorflow dataset types and re-compile the repo
```
ros2 fgr config
colcon build
```

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
    gcloud auth application-default login
    ```
3. (To be Automated) Create a Google Storage bucket through the Google Cloud Console.
4. (To be Automated) Create a Google BigQuery table XXX.DATASET_NAME.metadata

## Usage 
#### Dataset Collector
Run dataset generator (stores ROS2 message as RLDS format) with the following instructions
```
source install/setup.bash
ros2 launch fogros2_rt_x data_collector.launch.py
```

### Adapting your own application
Currently, there are two ways of adapting your own applications to FogROS2-RT-X. 

#### (Option 1) Write your own ROS2 publisher
It takes the [step](https://github.com/KeplerC/fogros2-rt-x/blob/main/fogros2_rt_x_msgs/msg/Step.msg) message type as the input; the step message is automatically generated for your dataset after you run `ros2 fgr config`. You can write your own publisher as following: 

```python
from fogros2_rt_x_msgs.msg import Step, Observation, Action

# initialize the data 
observation_msg = Observation()
action_msg = Action()
step_msg = Step()

# set your data
step_msg.observation = observation_msg
step_msg.action = action_msg
...

# publish it 
publisher.publish(step_msg)
```
The publisher can be created with [this example](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#id7)


#### (Option 2) Automatic Message Collection with FogROS2-RT-X (Beta, Unstable)
FogROS2-RT-X opens up one-to-one mapping of Observation/Action/Step field to ROS2 topic. After you run `data_collector.launch.py`, you may use `ros2 topic list` to view all the exposed topics. You may edit [Configuration File](./fogros2-rt-x/fogros2_rt_x/dataset_conf.py) to specify the desired topic to receive the specific type, or use [topic remapping](https://design.ros2.org/articles/static_remapping.html). 


#### Replaying with existing datasets in Open-X-Embodiment
You can replay existing datasets in ROS2 with 
```
source install/setup.bash
ros2 run fogros2_rt_x replayer --ros-args -r dataset_name:=bridge 
# replace with yours, e.g. berkeley_fanuc_manipulation
```
You may edit [replayer.launch.py](./fogros2-rt-x/launch/replayer.launch.py) for different datasets. 

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
