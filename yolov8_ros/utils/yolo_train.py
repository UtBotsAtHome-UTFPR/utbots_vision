# train_yolo_args.py

import argparse
import yaml
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
pkg_path = get_package_share_directory("yolov8_ros")

# Define the repository and the specific file you want to download
REPO_ID = "UTBotsAtHomeUTFPR/object_pretrained"
FILENAME = "objects_pretrained.pt"

def download_hf_model(repo_id, filename):
    """Downloads a model file from a Hugging Face repository if not cached."""
    from huggingface_hub import hf_hub_download
    from huggingface_hub.utils import HfHubHTTPError

    print(f"Attempting to download '{filename}' from Hugging Face repo '{repo_id}'...")
    try:
        model_path = hf_hub_download(repo_id=repo_id, filename=filename)
        print(f"Model successfully located at: {model_path}")
        return model_path
    except HfHubHTTPError as e:
        print(f"HTTP Error downloading model: {e}. Check repo/file names and your connection.")
        return None
    except Exception as e:
        print(f"An unexpected error occurred during model download: {e}")
        return None

def setup_roboflow_dataset(config, data_path):
    """
    Checks for a local dataset. If not found, downloads it from Roboflow.
    The `data_path` is the final directory where the dataset is expected to be.
    """
    if os.path.exists(f"{data_path}/train"):
        print(f"Dataset already exists at '{data_path}'. Skipping download.")
        return True

    print(f"Dataset not found at '{data_path}'. Attempting to download from Roboflow...")
    
    if 'roboflow' not in config:
        print("Error: 'roboflow' configuration not found in YAML file, and dataset is missing.")
        return False

    rf_config = config['roboflow']
    
    try:
        from roboflow import Roboflow
    except ImportError:
        print("Error: 'roboflow' package not installed. Please run 'pip install roboflow'")
        return False

    # Securely get API key from environment variable
    api_key = os.getenv("ROBOFLOW_API_KEY")
    if not api_key:
        print("Error: Roboflow API key not found.")
        print("Please set the 'ROBOFLOW_API_KEY' environment variable.")
        return False

    try:
        rf = Roboflow(api_key=api_key)
        project_name = rf_config['project']
        project = rf.workspace(rf_config['workspace']).project(project_name)
        version = project.version(rf_config['version'])
        
        # The download location is the parent directory of the final data_path
        download_location = os.path.join(pkg_path, "training_dir")
        os.makedirs(download_location, exist_ok=True)
        
        print(f"Downloading dataset from workspace {rf_config['workspace']}, project {project_name} and version {rf_config['version']} to '{download_location}'...")
        dataset = version.download("yolov11", location=download_location)
        
        if os.path.exists(download_location):
             print(f"Dataset downloaded and verified successfully at {download_location}.")
             return True
        else:
            print(f"Error: Download completed, but the expected data path '{download_location}' was not found.")
            print("Please ensure the 'path' in your YAML matches the Roboflow project name (e.g., './My-Dataset-1').")
            return False

    except Exception as e:
        print(f"An error occurred while downloading from Roboflow: {e}")
        return False

def main(args):
    """
    Main training function.
    """
    
    try:
        with open(args.data_config, 'r') as file:
            data_config = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Error: Configuration file not found at '{args.data_config}'")
        return

    dataset_path = os.path.dirname(args.data_config) + "/train"

    try:
        with open(args.train_config, 'r') as file:
            train_config = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Error: Configuration file not found at '{args.train_config}'")
        return
    
    if not setup_roboflow_dataset(data_config, dataset_path):
        print("Halting execution due to dataset setup failure.")
        return

    # --- Model Training ---
    # Load a pre-trained model (e.g., 'yolov8n.pt')
    if args.use_yolo_weights:
        model_path = 'yolov8n.pt'
    else:
        model_path = download_hf_model(REPO_ID, FILENAME)
    if not model_path:
        return
    
    model = YOLO(model_path)
    
    # Train the model using the loaded configuration
    print("Starting YOLOv8 model training...")

    train_args = train_config.copy()

    # Remove the keys not set by YAMP
    train_args.pop('model', None)  
    train_args.pop('data', None)

    # All parameters for YOLO training can and should be changed only in the configuration YAML
    results = model.train(
        data=args.data_config, 
        **train_args
    )
    print("Training finished!")
    print(f"Model and results are saved in the 'runs/detect/{results.save_dir}' directory.")

if __name__ == '__main__':
    # Create the argument parser
    parser = argparse.ArgumentParser(description="Train a YOLOv8 model with a custom dataset.")
    
    # Add arguments
    parser.add_argument(
        '--train-config', 
        type=str, 
        required=True, 
        help="Path to the training's YAML configuration file."
    )
    parser.add_argument(
        '--data-config', 
        type=str, 
        required=True, 
        help="Path to the dataset's YAML configuration file."
    )
    parser.add_argument(
        '--use_yolo_weights', 
        type=str, 
        default=None, 
        help="(Optional) Uses standard yolo weights as pretrained model for fine-tuning. "
             "Recommended only if our pretrained weights are giving bad results." \
             "Will result in longer training times."
    )
    
    # Parse the command-line arguments
    cli_args = parser.parse_args()
    
    # Run the main function
    main(cli_args)