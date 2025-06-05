import os
import os.path
from deepface import DeepFace
import cv2

# Add capability to search for a person by walking around the room, or at least looking around

class PictureTaker:
    def __init__(self):

        # Should create /faces
        self.train_dir = os.path.realpath(os.path.dirname(__file__)) + "/../faces"


    def picture_path_maker(self, name="Operator"):
        """
        Creates and returns the path to save images when given an Operator name. If name is aready used deletes current pictures

        ## Parameters
        - **name**: str = "Operator"

        ## Return
        - **path**: str
        """

        path = os.path.realpath(os.path.dirname(__file__)) + "/faces/" + name + "/"

        # Check whether the specified path exists or not
        if not os.path.exists(path):    
            os.makedirs(path)

        return path

    def crop_img(self, img: cv2.typing.MatLike, i=0):
        """
        Takes in image with single face, returns cropped image with just the face. If 0 or >1 face is detected returns False

        ## Parameters
        - **img**: MatLike

        ## Return
        - **cropped_img**  ||  **None**
        """

        backends = [
            'opencv', 'ssd', 'dlib', 'mtcnn', 'fastmtcnn',
            'retinaface', 'mediapipe', 'yolov8', 'yolov11s',
            'yolov11n', 'yolov11m', 'yunet', 'centerface',
        ]
        detector = backends[3] # Utilizar um desses no lugar de chamar o modelo de análise é melhor
        align = True # Melhora 6% o reconhecimento (aparentemente)

        face_objs = DeepFace.extract_faces(img_path = img, detector_backend = detector, align = align, enforce_detection=False)

        if len(face_objs) != 1:
            return None

        area = face_objs[0]["facial_area"]

        x = area["x"]
        y = area["y"]
        h = area["h"]
        w = area["w"]

        face_img = img[y:y+h, x:x+w]

        return face_img
    
    def save_img(self, path, img):

        cv2.imwrite(path + ".jpeg", img)

if __name__ == "__main__":
    program = PictureTaker()

    img = cv2.imread('person.jpeg')

    img = program.crop_img(img)
    #print(img)
    path = program.picture_path_maker("Operator")
    print(path)
    
    program.save_img(path + "Operator", img)