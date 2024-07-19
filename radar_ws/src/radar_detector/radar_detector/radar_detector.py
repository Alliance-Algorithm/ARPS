from queue import Queue
import traceback

import ffmpeg
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import Float32MultiArray # type: ignore

# Don't delete the imports
import ctypes
import os
import shutil
import random
import sys
import threading
import time
import cv2
import numpy as np
import pycuda.autoinit
import pycuda.driver as cuda
import tensorrt as trt
import logging
import json

# Defines
VIDEO = 1
CAMREA = 0
RED = 0
BLUE = 1

# Config settings
with open('./resources/config.json') as f:
    config = json.load(f)

    CONF_THRESH = config['CONF_THRESH']
    IOU_THRESHOLD = config['IOU_THRESHOLD']
    LEN_ALL_RESULT = config['LEN_ALL_RESULT']
    LEN_ONE_RESULT = config['LEN_ONE_RESULT']
    VIDEO_STREAM_MODE = VIDEO if config['VIDEO_STREAM_MODE'] == "VIDEO" else CAMREA
    DEBUG = True if config['DETECT_DEBUG'] == "true" else False
    FRIEND_SIDE = RED if config['FRIEND_SIDE'] == "RED" else BLUE
    publish_topic = config['publish_topic']

    # Paths
    PLUGIN_LIBRARY = config['PLUGIN_LIBRARY']
    car_engine_file_path = config['car_engine_file_path']
    armor_engine_file_path = config['armor_engine_file_path']
    video_path = config['video_path']
    log_path = config['detect_log_path']


# This is the ordered results of neural network
categories = ["CantIdentfy",
              "B_Hero", "B_Engineer", "B_Solider_3", "B_Solider_4", "B_Solider_5", "B_Sentry",
              "R_Hero", "R_Engineer", "R_Solider_3", "R_Solider_4", "R_Solider_5", "R_Sentry"]
# robot ids to be sent
robot_id = [0, 101, 102, 103, 104, 105, 107, 1, 2, 3, 4, 5, 7]


class inference_result:
    def __init__(self):
        self.image = None
        self.box = None
        self.score = None
        self.classid = None


def initialize_logging():
    logger = logging.getLogger(__name__)
    logger.setLevel(level=logging.DEBUG)
    handler = logging.FileHandler(log_path)
    handler.setLevel(logging.INFO)
    formatter = logging.Formatter(
        '[%(asctime)s][%(levelname)s] %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    return logger
logger = initialize_logging()

def package_message(message, result):
    x, y = get_chassis_position(result.box)
    message.data.append(
        float(robot_id[int(result.classid)]))
    message.data.append(float(x))
    message.data.append(float(y))
    return message


def exclude_same_classid(results):
    temp_results = [inference_result()]*13
    filtered_results = []
    for result in results:
        classid = int(result.classid)
        if classid == 0:
            # filtered_results.append(result)
            continue
        if temp_results[classid].score == None:
            temp_results[classid] = result
        elif temp_results[classid].score < result.score:
            temp_results[classid] = result
        else:
            pass

    for temp_result in temp_results:
        if temp_result.classid != None:
            filtered_results.append(temp_result)
    return filtered_results


def get_chassis_position(box):
    """
    @description: Get robot chassis from one box
    @param:
        box:      a box likes [x1,y1,x2,y2]
    @return:
        x,y
    """
    p1, p2 = (float(box[0]), float(box[1])), (float(box[2]), float(box[3]))
    aver_x = int((p1[0] + p2[0]) / 2)

    max_y = p1[1] if (p1[1] > p2[1]) else p2[1]
    chassis_y = int(max_y - ((abs(p1[1] - p2[1])*0.4) * (max_y / 2160.0)))
    return aver_x, chassis_y


def plot_one_box(x, img, side=None, label=None, line_thickness=None):
    """
    @description: Plots one bounding box on image img,
                 this function comes from YoLov5 project.
    @param:
        x:      a box likes [x1,y1,x2,y2]
        img:    a opencv image object
        color:  color to draw rectangle, such as (0,255,0)
        label:  str
        line_thickness: int
    @return:
        no return

    """
    tl = (
        line_thickness or round(0.001 * (img.shape[0] + img.shape[1]) / 2) + 1
    )  # line/font thickness
    # color = color or [random.randint(0, 255) for _ in range(3)]
    if side == RED:
        color = (50, 50, 255)
    else:
        color = (255, 50, 50)
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)


    p1, p2 = (float(x[0]), float(x[1])), (float(x[2]), float(x[3]))
    aver_x = int((p1[0] + p2[0]) / 2)
    max_y = p1[1] if (p1[1] > p2[1]) else p2[1]
    chassis_y = int(max_y - ((abs(p1[1] - p2[1])*0.4) * (max_y / 2160.0)))

    cv2.circle(img,(aver_x,chassis_y),15,(20,255,20),3)
    cv2.circle(img,(aver_x,chassis_y),5,(20,255,20),-1)

    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )


class YOLOv5TRT(object):
    """
    @description: A YOLOv5 class that warps TensorRT ops, preprocess and postprocess ops.
    """

    def __init__(self, engine_file_path):
        # Create a Context on this device,
        self.ctx = cuda.Device(0).make_context()
        stream = cuda.Stream()
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        runtime = trt.Runtime(TRT_LOGGER)

        # Deserialize the engine from file
        with open(engine_file_path, "rb") as f:
            engine = runtime.deserialize_cuda_engine(f.read())
        context = engine.create_execution_context()

        host_inputs = []
        cuda_inputs = []
        host_outputs = []
        cuda_outputs = []
        bindings = []

        for binding in engine:
            print('bingding:', binding, engine.get_binding_shape(binding))
            size = trt.volume(engine.get_binding_shape(
                binding)) * engine.max_batch_size
            dtype = trt.nptype(engine.get_binding_dtype(binding))
            # Allocate host and device buffers
            host_mem = cuda.pagelocked_empty(size, dtype)
            cuda_mem = cuda.mem_alloc(host_mem.nbytes)
            # Append the device buffer to device bindings.
            bindings.append(int(cuda_mem))
            # Append to the appropriate list.
            if engine.binding_is_input(binding):
                self.input_w = engine.get_binding_shape(binding)[-1]
                self.input_h = engine.get_binding_shape(binding)[-2]
                host_inputs.append(host_mem)
                cuda_inputs.append(cuda_mem)
            else:
                host_outputs.append(host_mem)
                cuda_outputs.append(cuda_mem)

        # Store
        self.stream = stream
        self.context = context
        self.engine = engine
        self.host_inputs = host_inputs
        self.cuda_inputs = cuda_inputs
        self.host_outputs = host_outputs
        self.cuda_outputs = cuda_outputs
        self.bindings = bindings
        self.batch_size = engine.max_batch_size

    def car_infer(self, raw_image):
        """
        @description: Do car inference
        @param:
            raw_image:input image for car inference
        @return:
            car_inference_results
            batch_image_raw

        """
        threading.Thread.__init__(self)
        # Make self the active context, pushing it on top of the context stack.
        self.ctx.push()
        # Restore
        stream = self.stream
        context = self.context
        engine = self.engine
        host_inputs = self.host_inputs
        cuda_inputs = self.cuda_inputs
        host_outputs = self.host_outputs
        cuda_outputs = self.cuda_outputs
        bindings = self.bindings
        # Do image preprocess
        batch_image_raw = []
        batch_origin_h = []
        batch_origin_w = []
        batch_input_image = np.empty(
            shape=[self.batch_size, 3, self.input_h, self.input_w])
        # for i, image_raw in enumerate(raw_image_generator):
        input_image, image_raw, origin_h, origin_w = self.preprocess_image(
            raw_image)
        batch_image_raw.append(image_raw)
        batch_origin_h.append(origin_h)
        batch_origin_w.append(origin_w)
        np.copyto(batch_input_image[0], input_image)
        batch_input_image = np.ascontiguousarray(batch_input_image)

        # Copy input image to host buffer
        np.copyto(host_inputs[0], batch_input_image.ravel())

        # Transfer input data  to the GPU.
        cuda.memcpy_htod_async(cuda_inputs[0], host_inputs[0], stream)
        # Run inference.
        context.execute_async(batch_size=self.batch_size,
                              bindings=bindings, stream_handle=stream.handle)
        # Transfer predictions back from the GPU.
        cuda.memcpy_dtoh_async(host_outputs[0], cuda_outputs[0], stream)
        # Synchronize the stream
        stream.synchronize()

        # Remove any context from the top of the context stack, deactivating it.
        self.ctx.pop()
        # Here we use the first row of output in that batch_size = 1
        output = host_outputs[0]

        # save results
        car_images = []
        car_inference_results = []
        car_infer_result = inference_result()
        # Do postprocess
        for i in range(self.batch_size):
            result_boxes, result_scores, result_classid = self.post_process(
                output[i * LEN_ALL_RESULT: (i + 1) *
                       LEN_ALL_RESULT], batch_origin_h[i], batch_origin_w[i]
            )
            # Draw rectangles and labels on the original image
            for j in range(len(result_boxes)):
                car_infer_result = inference_result()
                box = result_boxes[j]
                car_image = batch_image_raw[i][int(box[1]):int(
                    box[3]), int(box[0]):int(box[2])]
                car_infer_result.image = car_image
                car_infer_result.box = box
                car_infer_result.score = result_scores[j]
                car_infer_result.classid = int(result_classid[j])

                # plot_one_box(
                #     box,
                #     batch_image_raw[i],
                #     label="{}:{:.2f}".format(
                #         categories[int(result_classid[j])
                #                    ], result_scores[j]
                #     ),
                # )
                car_inference_results.append(car_infer_result)
        return car_inference_results, batch_image_raw

    def armor_infer(self, car_result):
        """
        @description: Do armor inference)
        @param:

            car_result:input class data car_result for armor inference
        @return:
            car_results

        """
        threading.Thread.__init__(self)
        # Make self the active context, pushing it on top of the context stack.
        self.ctx.push()
        # Restore
        stream = self.stream
        context = self.context
        engine = self.engine
        host_inputs = self.host_inputs
        cuda_inputs = self.cuda_inputs
        host_outputs = self.host_outputs
        cuda_outputs = self.cuda_outputs
        bindings = self.bindings
        # Do image preprocess
        raw_image = car_result.image

        batch_image_raw = []
        batch_origin_h = []
        batch_origin_w = []
        batch_input_image = np.empty(
            shape=[self.batch_size, 3, self.input_h, self.input_w])
        input_image, image_raw, origin_h, origin_w = self.preprocess_image(
            raw_image)
        batch_image_raw.append(image_raw)
        batch_origin_h.append(origin_h)
        batch_origin_w.append(origin_w)
        np.copyto(batch_input_image[0], input_image)
        batch_input_image = np.ascontiguousarray(batch_input_image)

        # Copy input image to host buffer
        np.copyto(host_inputs[0], batch_input_image.ravel())

        # Transfer input data  to the GPU.
        cuda.memcpy_htod_async(cuda_inputs[0], host_inputs[0], stream)
        # Run inference.
        context.execute_async(batch_size=self.batch_size,
                              bindings=bindings, stream_handle=stream.handle)
        # Transfer predictions back from the GPU.
        cuda.memcpy_dtoh_async(host_outputs[0], cuda_outputs[0], stream)
        # Synchronize the stream
        stream.synchronize()

        # Remove any context from the top of the context stack, deactivating it.
        self.ctx.pop()
        # Here we use the first row of output in that batch_size = 1
        output = host_outputs[0]
        classid = 0
        for i in range(self.batch_size):
            result_boxes, result_scores, result_classid = self.post_process(
                output[i * LEN_ALL_RESULT: (i + 1) *
                       LEN_ALL_RESULT], batch_origin_h[i], batch_origin_w[i]
            )
            class_scores = [0] * 12
            if_identfy = False
            for j in range(len(result_boxes)):
                if_identfy = True
                class_scores[int(result_classid[j])] += result_scores[j]
                # box = result_boxes[j]
                # plot_one_box(
                #     box,
                #     batch_image_raw[i],
                #     label="{}:{:.2f}".format(
                #         categories[int(result_classid[j] + 1)
                #                    ], result_scores[j]
                #     ),
                # )
            classid = class_scores.index(max(class_scores)) + 1 if if_identfy else 0
        car_result.classid = classid
        return car_result

    def destroy(self):
        # Remove any context from the top of the context stack, deactivating it.
        self.ctx.pop()

    def preprocess_image(self, raw_bgr_image):
        """
        @description: Convert BGR image to RGB,
                     resize and pad it to target size, normalize to [0,1],
                     transform to NCHW format.
        @param:
            input_image_path: str, image path
        @return:
            image:  the processed image
            image_raw: the original image
            h: original height
            w: original width
        """
        image_raw = raw_bgr_image
        h, w, c = image_raw.shape
        image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2RGB)
        # Calculate widht and height and paddings
        r_w = self.input_w / w
        r_h = self.input_h / h
        if r_h > r_w:
            tw = self.input_w
            th = int(r_w * h)
            tx1 = tx2 = 0
            ty1 = int((self.input_h - th) / 2)
            ty2 = self.input_h - th - ty1
        else:
            tw = int(r_h * w)
            th = self.input_h
            tx1 = int((self.input_w - tw) / 2)
            tx2 = self.input_w - tw - tx1
            ty1 = ty2 = 0
        # Resize the image with long side while maintaining ratio
        image = cv2.resize(image, (tw, th))
        # Pad the short side with (128,128,128)
        image = cv2.copyMakeBorder(
            image, ty1, ty2, tx1, tx2, cv2.BORDER_CONSTANT, None, (
                128, 128, 128)
        )
        image = image.astype(np.float32)
        # Normalize to [0,1]
        image /= 255.0
        # HWC to CHW format:
        image = np.transpose(image, [2, 0, 1])
        # CHW to NCHW format
        image = np.expand_dims(image, axis=0)
        # Convert the image to row-major order, also known as "C order":
        image = np.ascontiguousarray(image)
        return image, image_raw, h, w

    def xywh2xyxy(self, origin_h, origin_w, x):
        """
        @description:    Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
        @param:
            origin_h:   height of original image
            origin_w:   width of original image
            x:          A boxes numpy, each row is a box [center_x, center_y, w, h]
        @return:
            y:          A boxes numpy, each row is a box [x1, y1, x2, y2]
        """
        y = np.zeros_like(x)
        r_w = self.input_w / origin_w
        r_h = self.input_h / origin_h
        if r_h > r_w:
            y[:, 0] = x[:, 0] - x[:, 2] / 2
            y[:, 2] = x[:, 0] + x[:, 2] / 2
            y[:, 1] = x[:, 1] - x[:, 3] / 2 - \
                (self.input_h - r_w * origin_h) / 2
            y[:, 3] = x[:, 1] + x[:, 3] / 2 - \
                (self.input_h - r_w * origin_h) / 2
            y /= r_w
        else:
            y[:, 0] = x[:, 0] - x[:, 2] / 2 - \
                (self.input_w - r_h * origin_w) / 2
            y[:, 2] = x[:, 0] + x[:, 2] / 2 - \
                (self.input_w - r_h * origin_w) / 2
            y[:, 1] = x[:, 1] - x[:, 3] / 2
            y[:, 3] = x[:, 1] + x[:, 3] / 2
            y /= r_h

        return y

    def post_process(self, output, origin_h, origin_w):
        """
        @description: postprocess the prediction
        @param:
            output:     A numpy likes [num_boxes,cx,cy,w,h,conf,cls_id, cx,cy,w,h,conf,cls_id, ...]
            origin_h:   height of original image
            origin_w:   width of original image
        @return:
            result_boxes: finally boxes, a boxes numpy, each row is a box [x1, y1, x2, y2]
            result_scores: finally scores, a numpy, each element is the score correspoing to box
            result_classid: finally classid, a numpy, each element is the classid correspoing to box
        """
        # Get the num of boxes detected
        num = int(output[0])
        # Reshape to a two dimentional ndarray
        pred = np.reshape(output[1:], (-1, LEN_ONE_RESULT))[:num, :]
        pred = pred[:, :6]
        # Do nms
        boxes = self.non_max_suppression(
            pred, origin_h, origin_w, conf_thres=CONF_THRESH, nms_thres=IOU_THRESHOLD)
        result_boxes = boxes[:, :4] if len(boxes) else np.array([])
        result_scores = boxes[:, 4] if len(boxes) else np.array([])
        result_classid = boxes[:, 5] if len(boxes) else np.array([])
        return result_boxes, result_scores, result_classid

    def bbox_iou(self, box1, box2, x1y1x2y2=True):
        """
        @description: compute the IoU of two bounding boxes
        @param:
            box1: A box coordinate (can be (x1, y1, x2, y2) or (x, y, w, h))
            box2: A box coordinate (can be (x1, y1, x2, y2) or (x, y, w, h))
            x1y1x2y2: select the coordinate format
        @return:
            iou: computed iou
        """
        if not x1y1x2y2:
            # Transform from center and width to exact coordinates
            b1_x1, b1_x2 = box1[:, 0] - box1[:, 2] / \
                2, box1[:, 0] + box1[:, 2] / 2
            b1_y1, b1_y2 = box1[:, 1] - box1[:, 3] / \
                2, box1[:, 1] + box1[:, 3] / 2
            b2_x1, b2_x2 = box2[:, 0] - box2[:, 2] / \
                2, box2[:, 0] + box2[:, 2] / 2
            b2_y1, b2_y2 = box2[:, 1] - box2[:, 3] / \
                2, box2[:, 1] + box2[:, 3] / 2
        else:
            # Get the coordinates of bounding boxes
            b1_x1, b1_y1, b1_x2, b1_y2 = box1[:,
                                              0], box1[:, 1], box1[:, 2], box1[:, 3]
            b2_x1, b2_y1, b2_x2, b2_y2 = box2[:,
                                              0], box2[:, 1], box2[:, 2], box2[:, 3]

        # Get the coordinates of the intersection rectangle
        inter_rect_x1 = np.maximum(b1_x1, b2_x1)
        inter_rect_y1 = np.maximum(b1_y1, b2_y1)
        inter_rect_x2 = np.minimum(b1_x2, b2_x2)
        inter_rect_y2 = np.minimum(b1_y2, b2_y2)
        # Intersection area
        inter_area = np.clip(inter_rect_x2 - inter_rect_x1 + 1, 0, None) * \
            np.clip(inter_rect_y2 - inter_rect_y1 + 1, 0, None)
        # Union Area
        b1_area = (b1_x2 - b1_x1 + 1) * (b1_y2 - b1_y1 + 1)
        b2_area = (b2_x2 - b2_x1 + 1) * (b2_y2 - b2_y1 + 1)

        iou = inter_area / (b1_area + b2_area - inter_area + 1e-16)

        return iou

    def non_max_suppression(self, prediction, origin_h, origin_w, conf_thres=0.5, nms_thres=0.4):
        """
        description: Removes detections with lower object confidence score than 'conf_thres' and performs
        Non-Maximum Suppression to further filter detections.
        param:
            prediction: detections, (x1, y1, x2, y2, conf, cls_id)
            origin_h: original image height
            origin_w: original image width
            conf_thres: a confidence threshold to filter detections
            nms_thres: a iou threshold to filter detections
        return:
            boxes: output after nms with the shape (x1, y1, x2, y2, conf, cls_id)
        """
        # Get the boxes that score > CONF_THRESH
        boxes = prediction[prediction[:, 4] >= conf_thres]
        # Trandform bbox from [center_x, center_y, w, h] to [x1, y1, x2, y2]
        boxes[:, :4] = self.xywh2xyxy(origin_h, origin_w, boxes[:, :4])
        # clip the coordinates
        boxes[:, 0] = np.clip(boxes[:, 0], 0, origin_w - 1)
        boxes[:, 2] = np.clip(boxes[:, 2], 0, origin_w - 1)
        boxes[:, 1] = np.clip(boxes[:, 1], 0, origin_h - 1)
        boxes[:, 3] = np.clip(boxes[:, 3], 0, origin_h - 1)
        # Object confidence
        confs = boxes[:, 4]
        # Sort by the confs
        boxes = boxes[np.argsort(-confs)]
        # Perform non-maximum suppression
        keep_boxes = []
        while boxes.shape[0]:
            large_overlap = self.bbox_iou(np.expand_dims(
                boxes[0, :4], 0), boxes[:, :4]) > nms_thres
            label_match = boxes[0, -1] == boxes[:, -1]
            # Indices of boxes with lower confidence scores, large IOUs and matching labels
            invalid = large_overlap & label_match
            keep_boxes += [boxes[0]]
            boxes = boxes[~invalid]
        boxes = np.stack(keep_boxes, 0) if len(keep_boxes) else np.array([])
        return boxes

class readImageThread(threading.Thread):
    def __init__(self, video_stream ,frames):
        threading.Thread.__init__(self)
        self.video_stream = video_stream
        self.frames = frames

    def run(self):
        while True:
            in_bytes = self.video_stream.stdout.read(3840 * 2160 * 3 // 2)  # nv12 format
            if not in_bytes:
                break
            in_frame = (
                np
                .frombuffer(in_bytes, np.uint8)
                .reshape([2160 * 3 // 2, 3840])
            )
            image = cv2.cvtColor(in_frame, cv2.COLOR_YUV2BGR_NV12)
            self.video_stream.stdout.flush()
            if self.frames.qsize() <= 1:
                self.frames.put(image)


class carInferThread(threading.Thread):
    def __init__(self, yolov5_wrapper, image):
        threading.Thread.__init__(self)
        self.yolov5_wrapper = yolov5_wrapper
        self.image = image
        self.car_inference_results = None
        self.batch_image_raw = None

    def run(self):
        car_inference_results, batch_image_raw = self.yolov5_wrapper.car_infer(
            self.image)
        self.car_inference_results = car_inference_results
        self.batch_image_raw = batch_image_raw

    def return_result(self):
        return self.car_inference_results, self.batch_image_raw


class armorInferThread(threading.Thread):
    def __init__(self, yolov5_wrapper, car_inference_results):
        threading.Thread.__init__(self)
        self.yolov5_wrapper = yolov5_wrapper
        self.car_inference_results = car_inference_results
        self.batch_image_raw = None
        self.final_inference_results = None

    def run(self):
        self.final_inference_results = []
        for car_result in self.car_inference_results:
            car_result = self.yolov5_wrapper.armor_infer(
                car_result)
            self.final_inference_results.append(car_result)

    def return_result(self):
        return self.final_inference_results


class Car_position_publisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.command_publisher_ = self.create_publisher(
            Float32MultiArray, publish_topic, 10)


def main(args=None):

    # [INITIALIZE]
    

    logger.info("---[Log start]---")
    logger.info("CONFIG:" + str(config))

    logger.info("Initializing node...")
    rclpy.init(args=args)
    publish_node = Car_position_publisher("Car_position_publisher")
    logger.info("[√]Node initialized.")

    logger.info("Loading videostream...")
    try:
        if VIDEO_STREAM_MODE == CAMREA:
            video_stream = cv2.VideoCapture(CAMREA)
            index = CAMREA
            while not video_stream.isOpened():
                index += 1
                if index > 10:
                    index = 0
                
                try:
                    video_stream = cv2.VideoCapture(index)
                    
                except Exception as e:
                    pass
                time.sleep(0.2)
            
            try:
                video_stream.release()
                ffmpeg_input = '/dev/video'+str(index)
                video_stream = (
                    ffmpeg
                    .input(ffmpeg_input,s="3840x2160",pix_fmt="nv12")
                    .output('pipe:', format='rawvideo', pix_fmt='nv12')
                    .run_async(pipe_stdout=True)
                )
            except Exception as e:
                logger.error(e)
        elif VIDEO_STREAM_MODE == VIDEO:
            video_stream = cv2.VideoCapture(video_path)
            if not video_stream.isOpened():
                logger.error("[x]can't open video")
                return
    except Exception as e:
        logger.error("[x]ERROR:")
        logger.error(e)
        return

    if DEBUG:
        try:
            cv2.namedWindow("result", 0)
            cv2.resizeWindow("result", (800, 450))
        except Exception as e:
            logger.error("[x]ERROR:")
            logger.error(e)
    logger.info("[√]Loaded.")

    # load custom plugin and engine
    logger.info("Load custom plugin and engine...")
    ctypes.CDLL(PLUGIN_LIBRARY)
    logger.info("[√]Library loaded.")

    try:
        yolov5_car_wrapper = YOLOv5TRT(car_engine_file_path)
    except Exception as e:
        logger.error("[x]ERROR:")
        logger.error(e)
        return
    logger.info("[√]car engine loaded.")
    try:
        yolov5_armor_wrapper = YOLOv5TRT(armor_engine_file_path)
    except Exception as e:
        logger.error("[x]ERROR:")
        logger.error(e)
        return
    logger.info("[√]armor engine loaded.")

    # [MAIN LOOP]
    try:
        logger.info("[RUNNING MAIN LOOP]")
        frames = Queue()
        read_image_thread = readImageThread(video_stream,frames)
        read_image_thread.start()
        while True:
            try:
                last_time = int(round(time.time() * 1000))

                if VIDEO_STREAM_MODE == VIDEO:
                    ret, image = video_stream.read()
                    if not ret:
                        break
                else:
                    image = frames.get()
                    if not image.size:
                        break
                # create a new thread to do inference
                car_thread = carInferThread(yolov5_car_wrapper, image)
                car_thread.start()
                car_thread.join()

                car_inference_results, batch_image_raw = car_thread.return_result()

                armor_thread = armorInferThread(
                    yolov5_armor_wrapper, car_inference_results)
                armor_thread.start()
                armor_thread.join()

                final_results = exclude_same_classid(
                    armor_thread.return_result())

                message = Float32MultiArray()
                message.data = []

                current_time = int(round(time.time() * 1000))

                logger.info(">[Result count] " + str(len(final_results)) +
                            " [FPS] " + str(int(1000.0/(current_time-last_time))))
                for result in final_results:
                    logger.debug(
                        "[Id]" + categories[int(result.classid)] + "| [Score] " + str(result.score))
                    if FRIEND_SIDE == RED:
                        if int(result.classid) < 7:
                            message = package_message(message, result)
                            if DEBUG:
                                plot_one_box(result.box, batch_image_raw[0],
                                             side=BLUE if int(
                                    result.classid) < 7 else RED,
                                    label="{}:{:.2f}".format(
                                    categories[int(result.classid)], result.score),
                                )

                    elif FRIEND_SIDE == BLUE:
                        if int(result.classid) > 6:
                            message = package_message(message, result)
                            if DEBUG:
                                plot_one_box(result.box, batch_image_raw[0],
                                             side=BLUE if int(
                                                 result.classid) < 7 else RED,
                                             label="{}:{:.2f}".format(
                                                 categories[int(result.classid)], result.score),
                                             )

                    else:
                        message = package_message(message, result)
                        if DEBUG:
                            plot_one_box(result.box, batch_image_raw[0],
                                         side=BLUE if int(
                                result.classid) < 7 else RED,
                                label="{}:{:.2f}".format(
                                categories[int(result.classid)], result.score),
                            )

                publish_node.command_publisher_.publish(message)

                if DEBUG:
                    for i in range(len(batch_image_raw)):

                        cv2.putText(batch_image_raw[i], "Process FPS: " + str(
                            int(1000.0/(current_time-last_time))), (20, 50), 1, 4, (150, 255, 150), 4)

                        cv2.imshow("result", batch_image_raw[i])
                        cv2.waitKey(1)
            except Exception as e:
                logger.error("[x]main loop error:")
                logger.error(e)
                logger.error('\n','>>>' * 20)
                logger.error(traceback.print_exc())
                return

    finally:
        # destroy the instance
        yolov5_car_wrapper.destroy()
        yolov5_armor_wrapper.destroy()
        read_image_thread.join()
    rclpy.spin(publish_node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown()  # 关闭rclpy
