# Don't trust this README, it is all garbage, don't use this file, we are moving to new face recognition so that there isn't some idiotic python 3.9 dependency

## Arrumar depois pra pegar as coisas do ROS1

Nessa ordem:

- Instalar versão correta do python

    sudo apt update
    sudo apt install software-properties-common
    sudo add-apt-repository ppa:deadsnakes/ppa
    sudo apt update
    sudo apt install python3.9
    sudo apt install python3.9-venv

- Criar e ativar venv
    cd ~/ros2_ws/src/utbots_face_recognition
    python3.9 -m venv .venv
    . .venv/bin/activate


- Instalar pacotes

    pip install pip install numpy==1.19.5
    pip install face_recognition
    pip install scikit-learn
    pip install opencv-python
    pip install git+https://github.com/ageitgey/face_recognition_models
