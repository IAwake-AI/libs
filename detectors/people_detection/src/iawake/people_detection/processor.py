# ripped from https://bit.ly/2HI0VPp
import cv2
from imutils import object_detection

from iawake.core.processor import (
    NoDataAvailable,
    Processor,
)

from iawake.core.types import DetectedBox


def is_relevant_frame(previous_frame, frame_resized_grayscale, min_area):
    """
    This function returns 1 for the frames in which the area
    after subtraction with previous frame is greater than minimum area
    defined.
    Thus expensive computation of human detection face detection
    and face recognition is not done on all the frames.
    Only the frames undergoing significant amount of change (which is controlled min_area)
    are processed for detection and recognition.
    """
    frame_delta = cv2.absdiff(previous_frame, frame_resized_grayscale)
    thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)
    im2, contours, hierarchy = cv2.findContours(
        thresh.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    for c in contours:
        if cv2.contourArea(c) > min_area:
            return True
    return False


class PeopleToRectanglesProcessor(Processor):
    return_type = str

    def __init__(self):
        self._previous_frame = None
        self._min_area = None
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def get_detected_people(self, frame, resize_ratio):
        rectangles, weights = self._hog.detectMultiScale(
            frame,
            winStride=(8, 8),
            padding=(16, 16),
            scale=1.06
        )
        weights = weights.flatten()
        filtered = object_detection.non_max_suppression(
            rectangles,
            probs=weights.flatten(),
            overlapThresh=0.65,
        )
        detections = []
        for (x, y, w, h) in (filtered / resize_ratio):
            detections.append(DetectedBox({
                'x': int(x),
                'y': int(y),
                'w': int(w),
                'h': int(h),
                'confidence': 1,
            }))
        return detections

    def process(self, data):
        frame = data
        (h, w) = frame.shape[:2]
        desired_width = min(800, frame.shape[1])
        resize_ratio = desired_width / float(w)
        resized_dimensions = (desired_width, int(h * resize_ratio))

        frame_resized = cv2.resize(
            frame,
            resized_dimensions,
            interpolation=cv2.INTER_AREA,
        )
        frame_resized_grayscale = cv2.cvtColor(
            frame_resized,
            cv2.COLOR_BGR2GRAY,
        )
        if self._previous_frame is None:
            self._min_area = (3000 / 800) * frame_resized.shape[1]

        detections = None
        if self._previous_frame is None or is_relevant_frame(
                self._previous_frame,
                frame_resized_grayscale,
                self._min_area,
        ):
            detections = self.get_detected_people(frame_resized, resize_ratio)

        self._previous_frame = frame_resized_grayscale
        if detections is None:
            raise NoDataAvailable
        return detections
