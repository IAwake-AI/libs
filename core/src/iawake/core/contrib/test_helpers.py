import cv2


def generate_debug_bounding_box_display_processor(_cls):
    class DebugBoundingBoxDisplayProcessor(_cls):
        def process(self, frame):
            detections = super(DebugBoundingBoxDisplayProcessor, self) \
                .process(frame)
            for detection in detections:
                x = detection.x
                y = detection.y
                w = detection.w
                h = detection.h
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.imshow('image', frame)
            cv2.waitKey()
            return detections

    return DebugBoundingBoxDisplayProcessor
