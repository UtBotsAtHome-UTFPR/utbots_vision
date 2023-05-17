- **This stack contains vision related packages, such as:**
    - [darknet_ros](https://github.com/gustavo-fardo/darknet_ros.git)
    - [mediapipe_track](https://github.com/UtBotsAtHome-UTFPR/mediapipe_track.git)
    - vision_tools

- ### Getting started
    - ### Installation
        ```bash 
        cd catkin_ws/src
        git clone --recurse-submodules https://github.com/UtBotsAtHome-UTFPR/utbots_vision.git
        cd ../
        ```

    - #### Building
        ```bash
        catkin_make -DCMAKE_BUILD_TYPE=Release
        ```
