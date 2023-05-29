Header header

# skeletal key landmarks (points) naming
# Follows MediapipePose Landmarker Model
# https://developers.google.com/mediapipe/solutions/vision/pose_landmarker

uint8 NOSE             = 0 
uint8 LEFT_EYE_INNER   = 1 
uint8 LEFT_EYE         = 2 
uint8 LEFT_EYE_OUTER   = 3 
uint8 RIGHT_EYE_INNER  = 4 
uint8 RIGHT_EYE        = 5 
uint8 RIGHT_EYE_OUTER  = 6 
uint8 LEFT_EAR         = 7 
uint8 RIGHT_EAR        = 8 
uint8 MOUTH_LEFT       = 9 
uint8 MOUTH_RIGHT      = 10
uint8 LEFT_SHOULDER    = 11
uint8 RIGHT_SHOULDER   = 12
uint8 LEFT_ELBOW       = 13
uint8 RIGHT_ELBOW      = 14
uint8 LEFT_WRIST       = 15
uint8 RIGHT_WRIST      = 16
uint8 LEFT_PINKY       = 17
uint8 RIGHT_PINKY      = 18
uint8 LEFT_INDEX       = 19
uint8 RIGHT_INDEX      = 20
uint8 LEFT_THUMB       = 21
uint8 RIGHT_THUMB      = 22
uint8 LEFT_HIP         = 23
uint8 RIGHT_HIP        = 24
uint8 LEFT_KNEE        = 25
uint8 RIGHT_KNEE       = 26
uint8 LEFT_ANKLE       = 27
uint8 RIGHT_ANKLE      = 28
uint8 LEFT_HEEL        = 29
uint8 RIGHT_HEEL       = 30
uint8 LEFT_FOOT_INDEX  = 31
uint8 RIGHT_FOOT_INDEX = 32

# Detected skeleton points array
geometry_msgs/Point[] points
