import cv2

from iawake.people_detection.processor import PeopleToRectanglesProcessor


class DebugDisplayPeopleToRectanglesProcessor(PeopleToRectanglesProcessor):
    def get_detected_people(self, frame):
        detections = super(DebugDisplayPeopleToRectanglesProcessor, self) \
            .get_detected_people(frame)
        for detection in detections:
            x = detection.bounding_box.x
            y = detection.bounding_box.y
            w = detection.bounding_box.w
            h = detection.bounding_box.h
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.imshow('image', frame)
        cv2.waitKey()
        return detections
