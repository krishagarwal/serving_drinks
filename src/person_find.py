import face_recognition
import numpy as np
import time

def get_initial():
    print("in get_initial")
    time.sleep(3)
    image = face_recognition.load_image_file("/root/projects/catkin_ws/src/serving_drinks/src/harshal.jpg")
    encodings = face_recognition.face_encodings(image)
    return encodings[0]

def get_encoding(image):
    encodings = face_recognition.face_encodings(image)
    if (len(encodings) == 0):
        return np.zeros(shape=(0, 0))
    return encodings[0]

def find_person(image, original_encoding):
    loc = face_recognition.face_locations(image)
    enc = face_recognition.face_encodings(image, loc)
    print("Number of faces: ", len(loc))
    if(len(enc) > 0):
        for (top, right, bottom, left), face_encoding in zip(loc, enc):
            center = ((top + bottom) // 2, (left + right) // 2)
            results = face_recognition.compare_faces([original_encoding], face_encoding, tolerance=0.55)
            print("Results: ", results)
            if results[0]:
                return center
    return (-1, -1)