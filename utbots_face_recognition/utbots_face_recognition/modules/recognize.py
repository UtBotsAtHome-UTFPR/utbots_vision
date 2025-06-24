#!/usr/bin/venv_utbots_face_recognition/bin/python
import os
import pickle
import cv2
import copy
import base64
from deepface import DeepFace

class Recognize_Action():
    """
    A module that makes offers image recognition features when given a trained knn model.

    ## Functions

    - **recognize(img, model, expected_faces)**

    Performs recognition, if no model is given it searches on the default model path
    """
    
    def __init__(self):
        
        # OpenCV
        self.cv_img = None

        # Flags
        self.new_rgbImg = False

        # Algorithm variables
        self.face_encodings = []

        self.path = os.path.expanduser("~/.faces")



    def recognize(self, img:cv2.typing.MatLike, model: str=False, expected_faces: int=0):
        '''Performs recognition on an image.

        ## Parameters
        - **img**: MatLike

        Must be a cv2 type Image

        - **model**: str

        Model decoded into string (IDEALLY CHANGE IT SO IT'S ONLY THE MODEL NAME)

        - **expected_faces**: int

        Number of faces expected to be found, if 0 only checks for problems in the recognition

        ## Return
        - **tuple(MatLike, bbox:list) or None**

        If expected faces is correct or 0 returns the marked image + list of bounding boxes where each bounding box is a map
        
        '''

        backends = [
            'opencv', 'ssd', 'dlib', 'mtcnn', 'fastmtcnn',
            'retinaface', 'mediapipe', 'yolov8', 'yolov11s',
            'yolov11n', 'yolov11m', 'yunet', 'centerface',
        ]
        detector = backends[3] # Utilizar um desses no lugar de chamar o modelo de análise é melhor
        align = True # Melhora 6% o reconhecimento (aparentemente)

        face_objs = DeepFace.extract_faces(img_path = img, detector_backend = detector, align = align, enforce_detection=False)

        if len(face_objs) == 0:
            print("No faces were found in this image")
            return None
        
        model_name = "Facenet512"
        distance_metric = "cosine"

        people = []
        for i in face_objs:
            area = i["facial_area"]

            x = area["x"]
            y = area["y"]
            h = area["h"]
            w = area["w"]
            
            
            face_img = img[y:y+h, x:x+w]

            results_df = DeepFace.find(
                img_path=face_img,
                db_path=self.path,
                model_name=model_name,
                distance_metric=distance_metric,
                detector_backend="skip", # Estamos passando imagens já cropadas e alinhadas
                enforce_detection=False,
                align=False,
                silent=True
            )

            if isinstance(results_df, list) and len(results_df) > 0 and not results_df[0].empty:
                top_match = results_df[0].iloc[0]

                distance = top_match['distance']
                identity = top_match['identity']

                directory = os.path.dirname(identity)
                identity = os.path.basename(directory)

                # Also extract face coordinates
                face_objs = DeepFace.extract_faces(img_path=img, detector_backend="skip", align=True)

                people.append({
                    'identity': identity,
                    'distance': distance,
                    'facial_area': area
                })
            else:
                people.append({
                    'identity': "Unknown",
                    'distance': None,
                    'facial_area': area
                })
        return people

    # SÓ FUNCIONA QUANDO PEOPLE ESTÁ ORGANIZADO SEGUNDO UTBOTS/MSG/BOUNDINGBOX[]
    def draw_rec_on_faces(self, img, people):

        for person in people:
            top = person.ymin
            bottom = person.ymax
            left = person.xmin
            right = person.xmax
            name = person.id

            # Draw a box around the face
            cv2.rectangle(img, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(img, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(img, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        return img


if __name__ == '__main__':
    classifier = Recognize_Action()

    image_path = 'person.jpeg'
    image = cv2.imread(image_path)

    result = classifier.recognize(image)
    print(result)

    #for person in result:
    #    classifier.draw_rec_on_faces()
    
    #cv2.imwrite("recognized.jpeg", img)