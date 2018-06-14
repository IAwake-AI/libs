from iawake.core.processor import Processor
from iawake.core.types.animations import HeadTilt, AnimationWrapper


class FaceTrackingProcessor(Processor):
    return_type = AnimationWrapper

    def process(self, data):
        detected_face_boxes = data
        face_of_interest = detected_face_boxes[0]
        point_of_interest = face_of_interest.x + face_of_interest.w / 2.0
        angle = 1.57 * (1 - point_of_interest / 360.0)
        animation = HeadTilt({
            'angle': angle,
        })
        return AnimationWrapper.from_animation(animation)
