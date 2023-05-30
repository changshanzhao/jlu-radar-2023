#!/usr/bin/env python3

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type
from PIL import ImageFont
from sensor_msgs.msg import Image, CompressedImage
from detection_msgs.msg import BoundingBox, BoundingBoxes


# add yolov5 submodule to path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox
"""
def check_font(font='Arial.ttf', size=10):
    # Return a PIL TrueType Font, downloading to CONFIG_DIR if necessary
    font = Path(font)
    font = font if font.exists() else (CONFIG_DIR / font.name)
    try:
        return ImageFont.truetype(str(font) if font.exists() else font.name, size)
    except Exception as e:  # download if missing
        url = "https://ultralytics.com/assets/" + font.name
        print(f'Downloading {url} to {font}...')
        torch.hub.download_url_to_file(url, str(font), progress=False)
        return ImageFont.truetype(str(font), size)

RANK = int(os.getenv('RANK', -1))
class Annotator:
    if RANK in (-1, 0):
        check_font()  # download TTF if necessary
"""
def scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
    # Rescale coords (xyxy) from img1_shape to img0_shape
    if ratio_pad is None:  # calculate from img0_shape
        gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])  # gain  = old / new
        pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2  # wh padding
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    coords[:, [0, 2, 4, 6]] -= pad[0]  # x padding
    coords[:, [1, 3, 5, 7]] -= pad[1]  # y padding
    coords[:, :8] /= gain
    # clip_coords(coords, img0_shape)
    return coords
@torch.no_grad()
class Yolov5Detector:
    def __init__(self):
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        self.view_image = rospy.get_param("~view_image")
        # Initialize weights 
        weights = rospy.get_param("~weights")
        # Initialize model
        self.device = select_device(str(rospy.get_param("~device","")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h",480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.model.warmup()  # warmup        
        
        # Initialize subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic, _ = get_topic_type("/Gxcam_node/cam_image", blocking = True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                "/Gxcam_node/cam_image", CompressedImage, self.callback, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                "/Gxcam_node/cam_image", Image, self.callback, queue_size=1
            )

        # Initialize prediction publisher
        self.pred_pub = rospy.Publisher(
            "output_topic", BoundingBoxes, queue_size=10
        )
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=10
            )
        
        # Initialize CV_Bridge
        self.bridge = CvBridge()

    def callback(self, data):
        """adapted from yolov5/detect.py"""
        # print(data.header)
        if self.compressed_input:
            im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        im, im0 = self.preprocess(im)
        # print(im.shape)
        # print(img0.shape)
        # print(img.shape)

        # Run inference
        # dt, seen = [0.0, 0.0, 0.0], 0
        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.half else im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]
        pred_raw = self.model(im)[0].detach().cpu()
        pred = []
        for p in pred_raw:
            mask = p[..., 8] > self.conf_thres
            # print(torch.max(p[..., 8]).detach().cpu().item())
            p = p[mask]
            if p.shape[0] > 0:
                # print(p[..., :8].numpy())
                xmin = torch.min(p[..., [0, 2, 4, 6]], dim=1).values.int().numpy()
                xmax = torch.max(p[..., [0, 2, 4, 6]], dim=1).values.int().numpy()
                ymin = torch.min(p[..., [1, 3, 5, 7]], dim=1).values.int().numpy()
                ymax = torch.max(p[..., [1, 3, 5, 7]], dim=1).values.int().numpy()
                bbox = [[int(x1), int(y1), int(x2 - x1), int(y2 - y1)] for x1, x2, y1, y2 in
                        zip(xmin, xmax, ymin, ymax)]
                conf = [float(c) for c in p[..., 8].numpy()]
                cls_color = torch.argmax(p[..., 9:13], dim=-1).numpy()
                cls_number = torch.argmax(p[..., 13:22], dim=-1).numpy()
                cls = cls_color * 9 + cls_number
                ids = cv2.dnn.NMSBoxes(bbox, conf, self.conf_thres, self.iou_thres)
                p = torch.stack([
                    torch.cat([
                        torch.tensor(p[i, :8]).float(), torch.tensor([conf[i]]).float(), torch.tensor([cls[i]]).float()
                    ], dim=0)
                    for i in ids.reshape(ids.shape[0])
                ], dim=0)
            pred.append(p)
        # pred = self.model(im, augment=False, visualize=False)
        # pred = non_max_suppression(
        #     pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
        # )
        
        ### To-do move pred to CPU and fill BoundingBox messages
        
        # Process predictions 
        for i, det in enumerate(pred):  # per image
            # p, s, im0, frame = path, '', im0s.copy(), getattr(dataset, 'frame', 0)
            # p = Path(p)  # to Path
            # save_path = str(save_dir / p.name)  # img.jpg
            # txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # img.txt
            s = ''
            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            # imc = im0.copy() if save_crop else im0  # for save_crop
            # annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :8] = scale_coords(im.shape[2:], det[:, :8], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # 这块儿应该就是新添功能了
                bounding_boxes = BoundingBoxes()
                for d in det:
                    pt0 = (int(d[0]), int(d[1]))
                    pt1 = (int(d[2]), int(d[3]))
                    pt2 = (int(d[4]), int(d[5]))
                    pt3 = (int(d[6]), int(d[7]))
                    # if save_crop:
                    #     # p1 = np.float32([[int(d[1]), int(d[0])], [int(d[7]), int(d[6])], [int(d[3]), int(d[2])]])
                    #     # p2 = np.float32([[0,0], [0,(int(d[7])-int(d[1]))], [(int(d[2])-int(d[0])),0]])
                    #     # M = cv2.getAffineTransform(p1, p2)
                    #     # dst = cv2.warpAffine(img, M, (cols, rows))
                    #     crop = imc[(int(d[1]) - 10): (int(d[3]) + 10),(int(d[0]) - 10):(int(d[4]) + 10), ::1]
                    #     file = save_dir / 'crops' / self.names[int(c)] / p.name
                    #     file.parent.mkdir(parents=True, exist_ok=True)
                    #     f = str(file)
                    #     cv2.imwrite(f, crop)
                    cv2.line(im0, pt0, pt1, (0, 255, 0), 2)
                    cv2.line(im0, pt1, pt2, (0, 255, 0), 2)
                    cv2.line(im0, pt2, pt3, (0, 255, 0), 2)
                    cv2.line(im0, pt3, pt0, (0, 255, 0), 2)
                    cv2.putText(im0, self.names[int(d[-1])], pt0, 1, 1, (0, 255, 0))
                    cv2.putText(im0, f"{torch.sigmoid(d[8]).item():.2f}", pt3, 1, 1, (0, 255, 0))
                    cv2.putText(im0, self.names[int(d[-1])], (pt1[0],pt1[1]+15), 1, 1, (0, 255, 0))
                    cv2.putText(im0, f"{torch.sigmoid(d[8]).item():.2f}", pt2, 1, 1, (0, 255, 0))

                    bounding_box = BoundingBox()
                    #bounding_box.x = int((int(d[0])+int(d[2])+int(d[4])+int(d[6]))/4)
                    #bounding_box.y = int((int(d[1])+int(d[3])+int(d[5])+int(d[7]))/4)
                    #bounding_box.cl = int(d[-1])
					bounding_box.x=1;
					bounding_box.y=2;
					bounding_box.cl=3;
                    bounding_boxes.bounding_boxes.append(bounding_box)
					


        # det = pred[0].cpu().numpy()

        # bounding_boxes = BoundingBoxes()
        # bounding_boxes.header = data.header
        # bounding_boxes.image_header = data.header
        
        # annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
        # if len(det):
        #     # Rescale boxes from img_size to im0 size
        #     det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            # # Write results
            # for *xyxy, conf, cls in reversed(det):
            #     bounding_box = BoundingBox()
            #     c = int(cls)
            #     # Fill in bounding box message
            #     bounding_box.Class = self.names[c]
            #     bounding_box.probability = conf 
            #     bounding_box.xmin = int(xyxy[0])
            #     bounding_box.ymin = int(xyxy[1])
            #     bounding_box.xmax = int(xyxy[2])
            #     bounding_box.ymax = int(xyxy[3])

            #     bounding_boxes.bounding_boxes.append(bounding_box)

            #     # Annotate the image
            #     if self.publish_image or self.view_image:  # Add bbox to image
            #           # integer class
            #         label = f"{self.names[c]} {conf:.2f}"
            #         annotator.box_label(xyxy, label, color=colors(c, True))       

                
                ### POPULATE THE DETECTION MESSAGE HERE

            # Stream results
        # im0 = annotator.result()

        # Publish prediction,这里发送中心坐标信息
        self.pred_pub.publish(bounding_boxes)

        # Publish & visualize images
        if self.view_image:
            cv2.imshow(str(0), im0)
            cv2.waitKey(1)  # 1 millisecond
        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))
        

    def preprocess(self, img):
        """
        Adapted from yolov5/utils/datasets.py LoadStreams class
        """
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0 


if __name__ == "__main__":

    check_requirements(exclude=("tensorboard", "thop"))
    
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    
    rospy.spin()
