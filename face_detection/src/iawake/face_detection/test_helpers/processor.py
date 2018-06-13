from iawake.core.contrib.test_helpers \
    import generate_debug_bounding_box_display_processor
from iawake.face_detection.processor import FaceDetectionProcessor

DebugDisplayFaceDetectionProcessor = \
    generate_debug_bounding_box_display_processor(
        FaceDetectionProcessor
    )
