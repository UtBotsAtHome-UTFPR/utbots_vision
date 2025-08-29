# yolov8_ros
- ROS package that implements YOLOv8
- Synchronous detection is Enabled/Disabled with a ROS service (disabled by standard)
- Assynchronous detection is possible with actions. Optional target category and success result for finding specific category bounding boxes
- Assynchronous batch detection is possible with actions, returning the detected objects with presence in multiple frames.
- Tested in Ubuntu 22 with ROS Humble

## Installation

### Dependencies and Build

To avoid conflicts between package dependencies, we use virtual environments. Change the virtuelenv path in the `executable` field in `setup.cfg`. *Not the ideal solution, but the current one while we don't use Docker*.

If you haven't installed `virtualenv`:
```bash
pip3 install virtualenv
```

Create and activate env:
```bash
python -m virtualenv <env_path>
source <env_path>/bin/activate
```

Install requirements and build:
```bash
pip3 install -r requirements.txt
cd ../..
colcon build --packages-select yolov8_ros utbots_actions utbots_srvs utbots_msgs \
--allow-overriding utbots_msgs utbots_actions utbots_srvs \
&& source install/setup.bash
```

## Running
To run the Mediapipe pose estimation node:

```bash
ros2 run yolov8_ros yolo_node
```

With launchfiles you can specify the parameter values using any of the arguments in Command Line or other launchfiles (for instance, disabling *draw* could save processing usage):

```bash
'weights':
    Path to the YOLOv8 model weights file.
    (default: '')

'camera_topic':
    The input ROS topic for RGB images.
    (default: '')

'device':
    Device to run inference on. Options: 'cuda' or 'cpu'.
    Defaults to 'cuda' if available.
    (default: 'cuda')

'conf':
    Confidence threshold for filtering detections.
    (default: 0.25)

'draw':
    Whether to draw bounding boxes on the output image.
    (default: true)

'target_category':
    Target class name to filter detections.
    If empty, all classes are allowed.
    (default: '')
```

## Training, Validation and Auto Annotation with YOLO models
Scripts avaliable in `./utils` folder facilitate the procedures of training, validating and auto annotating with YOLO model in our custom models.

### Training
The `./utils/yolo_train.py` makes it easy to train YOLO with the followingfeatures:
- Download and setup custom datasets annotated and stored at [Roboflow](https://roboflow.com/)with YAML file inside `./config` folder
- Import custom model hyperparemeters defined in YAML files inside the `./config` folder
- Train with custom pretrained models saved in [Hugging Face](https://huggingface.co/) repositories

First, you need to access the profile of the owner of the dataset in RoboFlow and find the API key, then set it in your terminal with:

```bash
export ROBOFLOW_API_KEY=<your_api_key>
```

**OBS**: if you run in a different directory than ./yolov8_ros, it might not work

Then you can run:
```bash
python3 utils/yolo_train.py --train-config config/training_params.yaml --data-config config/datasets_config/robocup2025.yaml --pretrained-repo UTBotsAtHomeUTFPR/object_pretrained --pretrained--filename objects_pretrained.pt # Example on RoboCup2025 dataset with our pretrained model on all competition objects 
```

To train on standard YOLO weights (do it if our pretrained is not good, for example, but it will result in a longer training)>
```bash
python3 utils/yolo_train.py --train-config config/training_params.yaml --data-config config/datasets_config/robocup2025.yaml --use_yolo_weights # Example on RoboCup2025 dataset with standard pretrained model
```

## Validate

To validate with standard YOLO validation tools:
```bash
yolo val model=training_dir/runs/detect/train/weights/best.pt data=training_dir/data.yaml project=./training_dir/runs/detect # Example in any model trained with our training script 
```

## Auto Annotation

To infer in multiple images for auto annotation and/or visual inspection:
```bash
python utils/auto_annotate.py --det_model training_dir/runs/detect/train/weights/best.pt --draw training_dir/valid/images/ # Example in any model trained with our training script 
```