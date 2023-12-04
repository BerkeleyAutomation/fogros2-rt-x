
# fogros2-rt-x

This repository contains the code for the fogros2-rt-x project. It is designed to work with ROS and TensorFlow.

## Installation 
### Steps

1. Install ROS following the instructions on the official ROS website.
2. Install TensorFlow dependencies.
3. Compile the repo
```
colcon build
```

### Google Cloud Setup

1. Install google cloud command line tools with the following link: [Google Cloud SDK](https://cloud.google.com/sdk/docs/install#deb)
2. Login to your Google Cloud account using the command line:
    ```
    gcloud auth application-default login
    ```
3. Create a Google Storage bucket through the Google Cloud Console.

## ROS1 Support 

For ROS1 support, use the ros2 bridge.

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
## Usage

Provide instructions on how to use your project.

## Contributing

Details on how to contribute to this project.

## License

Information about the license.
