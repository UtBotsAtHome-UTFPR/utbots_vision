# apollo_vision
vision stack

## COMO RODAR O YOLOV3 (I) E O DARKNET_ROS (II E III)
OBS: se vc fizer os passos I e II, terá 2 instalações distintas do darknet; o passo 1 só é importante para entender o YOLO, mas se preferir siga direto para o darknet 

- I) Seguir os passos dos tópicos para rodar o YOLOv3:
    - 1) Detection Using A Pre-Trained Model
    - 2) Real-Time Detection on a Webcam
        - a) ver https://pjreddie.com/darknet/install/#cuda
        - b) para instalar o CUDA: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/  index.html
        - c) ajustar o cuda capability no Makefile do darknet:
            - "ARCH= -gencode arch=compute_XX,code=[sm_XX,compute_XX]" => substituir XX com a CUDA capability do seu processador gráfico

- II) Seguir os passos de Installation do darknet_ros (o darknet será reinstalado, cuidado para não conflitar, se preferir exclua o darknet anterior)
    - OBS: para rodar com processamento gráfico, instalar o CUDA de acordo com o tópico I)2)
    - Mais informações em:
        - https://roboticsknowledgebase.com/wiki/machine-learning/ros-yolo-gpu/
        - https://github.com/leggedrobotics/darknet_ros

- III) Para rodar com o pacote usb_cam (webcam):
    - ```roslaunch apollo_vision darknet_webcam.launch```

- IV) Para abrir o rviz:
    - ```roslaunch apollo_vision rviz.launch```
