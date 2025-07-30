# utbots_vision

This stack contains vision related packages, such as:

- [mediapipe_track](https://github.com/UtBotsAtHome-UTFPR/mediapipe_track)
- utbots_face_recognition
- yolov8ros

And is dependant on:

- [utbots_dependencies](https://github.com/UtBotsAtHome-UTFPR/utbots_dependencies)

### Updating

To push changes to the submodule package [mediapipe_track](https://github.com/UtBotsAtHome-UTFPR/mediapipe_track) you should go to their repository path and perform a simple add, commit and push. After, you have to push the changes to the stack, going back to the stack repository path and doing the following command:

```bash
git submodule update --remote --merge
```

And then, perform a simple add, commit and push in the stack repository.

### Running

All packages listed above contain their own README files inside, some configuration is global but refer to them for specific documentation.

All systems for utbots_vision can be installed in a single venv and are generally compatible with the rest of utbots stack (except NLU). To gather how to setup the venv for a given package please refer to the README inside the package itself.

To call the venv sourcing is insufficient (ros2 is weird) and setup.cfg file needs to be altered to contain the path to the venv you've created. Shortcuts such as ~/ don't work so the path must be expanded (this file is in gitignore so changes to it are not pushed).