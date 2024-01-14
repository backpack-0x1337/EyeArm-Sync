from ultralytics import YOLO
import cv2 as cv


class Object:
    top_left_x, top_left_y, bottle_right_x, bottom_right_y = [None] * 4
    center_x, center_y, width, height = [None] * 4
    id = None
    cls = None
    conf = None
    info_str = None
    center_point_depth = None


class YoloReader:
    def __init__(self, model: YOLO):
        self.model = model
        self.object_list = []

    def track(self, bgr_img, depth_img, conf=0.5, retina_masks=True, show_boxes=True, show=False, persist=True):
        result = self.model.track(bgr_img, conf=conf,
                                  retina_masks=retina_masks,
                                  show_boxes=show_boxes,
                                  show=show,
                                  persist=persist,
                                  verbose=False)
        self.extract_data(result, depth_img)

    def predict(self, bgr_img, depth_img, conf=0.5):
        result = self.model(bgr_img,
                            conf=conf,
                            verbose=False)
        self.extract_data(result, depth_img)

    def extract_data(self, results: list, depth_img: list):
        for r in results:
            boxes = r.boxes

            for box in boxes:
                obj = Object()

                # Bounding Box X.Y Information
                obj.top_left_x, obj.top_left_y, obj.bottle_right_x, obj.bottom_right_y = box.xyxy[0]
                obj.top_left_x, obj.top_left_y, obj.bottle_right_x, obj.bottom_right_y = \
                    int(obj.top_left_x), int(obj.top_left_y), int(obj.bottle_right_x), int(obj.bottom_right_y)

                # Center X.Y Information
                obj.center_x, obj.center_y, obj.width, obj.height = box.xywh[0]
                obj.center_x, obj.center_y, obj.width, obj.height = int(obj.center_x), int(obj.center_y), int(
                    obj.width), int(obj.height)

                # Get object center depth
                obj.center_point_depth = depth_img[obj.center_y][obj.center_x]
                obj.cls = int(box.cls[0])
                if box.id is not None:
                    obj.id = int(box.id[0])
                else:
                    obj.id = "none"
                obj.conf = round(float(box.conf[0]), 2)
                obj.info_str = "Conf: %s Depth: %s" % (str(obj.conf), str(obj.center_point_depth))
                self.object_list.append(obj)

    def clear_object_list(self):
        self.object_list = []

    def print_object_list(self):
        for obj in self.object_list:
            print(obj.info_str)
