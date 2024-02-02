from ultralytics import YOLO
import cv2 as cv
from realsense_camera import RealsenseCamera



class Object:
    top_left_x, top_left_y, bottle_right_x, bottom_right_y = [None] * 4
    center_pixel_x, center_pixel_y, width, height = [None] * 4
    center_point_x, center_point_y = [None] * 2
    top_right_x, top_right_y = [None] * 2
    center_right_x, center_right_y = [None] * 2

    id = None
    cls = None
    conf = None
    name = None
    info_str = None
    center_point_depth = None


class YoloReader:
    def __init__(self, model: YOLO, rs: RealsenseCamera):
        self.model = model
        self.object_list = []
        self.rs = rs

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
                obj.center_pixel_x, obj.center_pixel_y, obj.width, obj.height = box.xywh[0]
                obj.center_pixel_x, obj.center_pixel_y, obj.width, obj.height = int(obj.center_pixel_x), int(obj.center_pixel_y), int(
                    obj.width), int(obj.height)
                
                obj.top_right_x, obj.top_right_y  = obj.top_left_x + obj.width, obj.top_left_y
                obj.center_right_x, obj.center_right_y  = obj.top_right_x, int(obj.top_right_y + obj.height / 2)



                # Get object center depth
                obj.center_point_depth = depth_img[obj.center_pixel_y][obj.center_pixel_x]

                # Convert 2D pixel to 3D point
                center_point_3d = self.rs.rs2_deproject_pixel_to_point(self.rs.intrinsics, [obj.center_pixel_x, obj.center_pixel_y], obj.center_point_depth)
                center_point_3d = [round(num, 1) for num in center_point_3d]
                obj.center_point_x, obj.center_point_y = center_point_3d[0], center_point_3d[1]
                
                obj.cls = int(box.cls[0])
                obj.name = self.model.names[obj.cls]
                if box.id is not None:
                    obj.id = int(box.id[0])
                else:
                    obj.id = "none"


                # Update Object Information
                obj.conf = round(float(box.conf[0]), 2)
                obj.info_str = [f"x:{center_point_3d[0]}",f"y:{center_point_3d[1]}", f"d: {center_point_3d[2]}"]
                self.object_list.append(obj)

    def clear_object_list(self):
        self.object_list = []

    def print_object_list(self):
        for obj in self.object_list:
            print(obj.info_str)
