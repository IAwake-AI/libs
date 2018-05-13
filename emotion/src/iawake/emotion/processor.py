import os

import cv2
from keras.models import model_from_json
from keras.preprocessing import image
import numpy as np
import tensorflow as tf

from iawake.core.processor import (
    NoDataAvailable,
    Processor,
)


class EmotionProcessor(Processor):
    return_type = str

    def __init__(self):
        models_directory = os.path.dirname(os.path.realpath(__file__)) + '/models'
        cascade_path = os.path.join(
            models_directory,
            'haarcascade_frontalface_default.xml',
        )
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        model_path = os.path.join(
            models_directory,
            'facial_expression_model_structure.json'
        )
        self.model = model_from_json(open(model_path).read())
        weights_path = os.path.join(
            models_directory,
            'facial_expression_model_weights.h5',
        )
        self.model.load_weights(weights_path)
        self.graph = tf.get_default_graph()

    def process(self, data):
        img = data
        emotions = ('angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral')

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

        if len(faces) == 0:
            raise NoDataAvailable

        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)  # draw rectangle to main image

            detected_face = img[int(y):int(y + h), int(x):int(x + w)]  # crop detected face
            detected_face = cv2.cvtColor(detected_face, cv2.COLOR_BGR2GRAY)  # transform to gray scale
            detected_face = cv2.resize(detected_face, (48, 48))  # resize to 48x48

            img_pixels = image.img_to_array(detected_face)
            img_pixels = np.expand_dims(img_pixels, axis=0)

            img_pixels /= 255  # pixels are in scale of [0, 255]. normalize all pixels in scale of [0, 1]

            with self.graph.as_default():
                predictions = self.model.predict(img_pixels)  # store probabilities of 7 expressions

            # find max indexed array 0: angry, 1:disgust, 2:fear, 3:happy, 4:sad, 5:surprise, 6:neutral
            max_index = np.argmax(predictions[0])

            return emotions[max_index]
