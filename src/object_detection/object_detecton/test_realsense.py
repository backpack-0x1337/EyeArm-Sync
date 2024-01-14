from realsense_camera import *
import cv2
from ultralytics import YOLO
from ObjectDetection.yolo_reader import YoloReader


def main():
    rs = RealsenseCamera()
    # model = YOLO("./models/yolov8s_playing_cards.pt")
    model = YOLO("./models/yolov8s_jasmine.pt")
    obj_reader = YoloReader(model)
    while True:
        ret, bgr_image, depth_image = rs.get_frame_stream()
        if ret is False:
            continue

        # Get Object Detection Data
        obj_reader.track(bgr_image, depth_image, conf=0.1)

        # Draw Box, class info & depth info on the Object
        for obj in obj_reader.object_list:
            cv2.rectangle(bgr_image, (obj.top_left_x, obj.top_left_y), (obj.bottle_right_x, obj.bottom_right_y),
                          (255, 0, 255), 1)
            cv2.circle(bgr_image, (obj.center_x, obj.center_y), 5, (0, 0, 255), 1)
            cv2.putText(bgr_image, model.names[obj.cls], (obj.top_left_x, obj.top_left_y), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (255, 0, 255), 1)
            cv2.putText(bgr_image, obj.info_str, (obj.bottle_right_x, obj.bottom_right_y), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (255, 0, 255), 1)
        obj_reader.clear_object_list()

        cv2.imshow("View", bgr_image)
        if cv2.waitKey(1) == ord('q'):
            break

    rs.release()
    obj_reader.clear_object_list()


def detect_object(results):
    for r in results:
        boxes = r.boxes

        for box in boxes:
            # bounding box
            top_left_x, top_left_y, bottle_right_x, bottom_right_y = box.xyxy[0]
            top_left_x, top_left_y, bottle_right_x, bottom_right_y = \
                int(top_left_x), int(top_left_y), int(bottle_right_x), int(bottom_right_y)

            center_x, center_y, width, height = box.xywh[0]
            center_x, center_y, width, height = int(center_x), int(center_y), int(width), int(height)
            cls = int(box.cls[0])
            if box.id is not None:
                object_id = int(box.id[0])
            else:
                object_id = "none"
            conf = round(float(box.conf[0]), 2)
            info_str = "Object ID: %s Conf: %s" % (str(object_id), str(conf))


if __name__ == "__main__":
    main()
