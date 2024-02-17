#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import datetime
import numpy as np
import cv2.dnn

from ultralytics.utils import ASSETS, yaml_load

CLASSES = yaml_load("/home/orin/catkin_ws/src/jetbot_pro/scripts/yolov8/coco128.yaml")["names"]
print("CLASSSES: ", CLASSES)
colors = np.random.uniform(0, 255, size=(len(CLASSES), 3))

def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h):
    """
    Draws bounding boxes on the input image based on the provided arguments.

    Args:
        img (numpy.ndarray): The input image to draw the bounding box on.
        class_id (int): Class ID of the detected object.
        confidence (float): Confidence score of the detected object.
        x (int): X-coordinate of the top-left corner of the bounding box.
        y (int): Y-coordinate of the top-left corner of the bounding box.
        x_plus_w (int): X-coordinate of the bottom-right corner of the bounding box.
        y_plus_h (int): Y-coordinate of the bottom-right corner of the bounding box.
    """
    # label = f"{CLASSES[class_id]} ({confidence:.2f})"
    label = str(CLASSES[class_id]) + ' confidence: ' + str(confidence)
    color = colors[class_id]
    cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
    cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

class image_converter:
  def __init__(self):
    self.camera_name = rospy.get_param("~camera_name","csi_cam_0")
    self.topic_name = rospy.get_param("~topic_name","new_object_detect")
    # self.file_name = rospy.get_param("~file_name","../data/haarcascade_frontalface_alt2.xml")
    self.classes = None

    self.bridge = CvBridge()
    # rospy.Subscriber('imagetimer', CompressedImage, self.callback)

    self.image_sub = rospy.Subscriber(self.camera_name+"/image_raw/compressed",CompressedImage,self.callback)
    self.image_pub = rospy.Publisher(self.topic_name+"/compressed",CompressedImage,queue_size=10)
    # self.face_cascade = cv.CascadeClassifier(self.file_name)
    self.speak_pub = rospy.Publisher("speak", String, queue_size=1)

  def get_output_layers(self, net):
    layer_names = net.getLayerNames()
    try:
        output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
    except:
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    return output_layers

  def draw_prediction(self, img, class_id, confidence, x, y, x_plus_w, y_plus_h):
    label = str(self.classes[class_id])
    color = self.COLORS[class_id]
    cv.rectangle(img, (x,y), (x_plus_w,y_plus_h), color, 2)
    cv.putText(img, label, (x-10,y-10), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

  def callback(self,data):
    try:
      # bridge = CvBridge()
      # image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
      #print("==== frame ====")
      #print(frame.shape)
      #print(type(frame))
      # rospy.loginfo(rospy.get_caller_id() + 'image shape: %s', str(image.shape))

      original_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Width = image.shape[1]
    # Height = image.shape[0]
    # scale = 0.00392

    # with open('/home/orin/catkin_ws/src/jetbot_pro/scripts/yolov3/yolov3.txt', 'r') as f:
    #     self.classes = [line.strip() for line in f.readlines()]
    # self.COLORS = np.random.uniform(0, 255, size=(len(self.classes), 3))

    # CLASSES = yaml_load("/home/orin/catkin_ws/src/jetbot_pro/scripts/yolov8/coco128.yaml")["names"]
    # print("CLASSSES: ", CLASSES)

    start = datetime.datetime.now()
    print("image: ", original_image.shape)

    # Load the ONNX model
    onnx_model = '/home/orin/catkin_ws/src/jetbot_pro/scripts/yolov8/yolov8n.onnx'
    # model: cv2.dnn.Net = cv2.dnn.readNetFromONNX(onnx_model)
    model = cv2.dnn.readNetFromONNX(onnx_model)

    [height, width, _] = original_image.shape

    # Prepare a square image for inference
    length = max((height, width))
    # image = np.zeros((length, length, 3), np.uint8)
    # image[0:height, 0:width] = original_image

    # Calculate scale factor
    scale = length / 640

    # Preprocess the image and prepare blob for model
    # blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255, size=(640, 640), swapRB=True)
    blob = cv2.dnn.blobFromImage(original_image, scalefactor=1 / 255, size=(640, 640), swapRB=True)
    model.setInput(blob)

    # Perform inference
    outputs = model.forward()

    # Prepare output array
    outputs = np.array([cv2.transpose(outputs[0])])
    rows = outputs.shape[1]

    boxes = []
    scores = []
    class_ids = []

    # Iterate through output to collect bounding boxes, confidence scores, and class IDs
    for i in range(rows):
        classes_scores = outputs[0][i][4:]
        (minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
        if maxScore >= 0.25:
            box = [
                outputs[0][i][0] - (0.5 * outputs[0][i][2]),
                outputs[0][i][1] - (0.5 * outputs[0][i][3]),
                outputs[0][i][2],
                outputs[0][i][3],
            ]
            boxes.append(box)
            scores.append(maxScore)
            class_ids.append(maxClassIndex)

    # Apply NMS (Non-maximum suppression)
    result_boxes = cv2.dnn.NMSBoxes(boxes, scores, 0.25, 0.45, 0.5)

    detections = []

    # Iterate through NMS results to draw bounding boxes and labels
    for i in range(len(result_boxes)):
        index = result_boxes[i]
        box = boxes[index]
        detection = {
            "class_id": class_ids[index],
            "class_name": CLASSES[class_ids[index]],
            "confidence": scores[index],
            "box": box,
            "scale": scale,
        }
        detections.append(detection)
        draw_bounding_box(
            original_image,
            class_ids[index],
            scores[index],
            round(box[0] * scale),
            round(box[1] * scale),
            round((box[0] + box[2]) * scale),
            round((box[1] + box[3]) * scale),
        )

        class_id = class_ids[index]
        if CLASSES[class_id] == "chair" or CLASSES[class_id] == "backpack":
            self.speak_pub.publish("Chair or Backpack detected")
    end = datetime.datetime.now()
    # fps = f"FPS: {1 / (end-start).total_seconds():.2f}"
    fps = "FPS: " + str(1/(end-start).total_seconds())
    cv.putText(original_image, fps, (50, 50), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
    cv2.imshow("image", original_image)
    cv2.waitKey(3)

    # net = cv.dnn.readNet('/home/orin/catkin_ws/src/jetbot_pro/scripts/yolov3/yolov3.weights', '/home/orin/catkin_ws/src/jetbot_pro/scripts/yolov3/yolov3.cfg')
    # blob = cv.dnn.blobFromImage(image, scale, (416,416), (0,0,0), True, crop=False)
    # net.setInput(blob)

    # outs = net.forward(self.get_output_layers(net))
    '''
    class_ids = []
    confidences = []
    boxes = []
    conf_threshold = 0.5
    nms_threshold = 0.4

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                center_x = int(detection[0] * Width)
                center_y = int(detection[1] * Height)
                w = int(detection[2] * Width)
                h = int(detection[3] * Height)
                x = center_x - w / 2
                y = center_y - h / 2
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h])

    indices = cv.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

    for i in indices:
        try:
            box = boxes[i]
        except:
            i = i[0]
            box = boxes[i]
    
        x = box[0]
        y = box[1]
        w = box[2]
        h = box[3]
        self.draw_prediction(image, class_ids[i], confidences[i], round(x), round(y), round(x+w), round(y+h))
    '''
    '''
    # load the class labels the model was trained on
    class_names = []
    with open("/home/orin/catkin_ws/src/jetbot_pro/scripts/ssd_mobilenet/coco_names.txt", "r") as f:
        class_names = f.read().strip().split("\n")

    colors = np.random.randint(0, 255, size=(len(class_names), 3))
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frame_gray = cv.equalizeHist(frame_gray)

    h = frame_gray.shape[0]
    w = frame_gray.shape[1]

    # faces = self.face_cascade.detectMultiScale(frame_gray,1.2, 5,0,(50,50))
    # net = cv.dnn.readNetFromCaffe('Model/MobileNetSSD_deploy.prototxt.txt', 'Model/MobileNetSSD_deploy.caffemodel')
    # path to the weights and model files
    weights = "/home/orin/catkin_ws/src/jetbot_pro/scripts/ssd_mobilenet/frozen_inference_graph.pb"
    model = "/home/orin/catkin_ws/src/jetbot_pro/scripts/ssd_mobilenet/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
    
    start = datetime.datetime.now()

    # load the MobileNet SSD model trained  on the COCO dataset
    net = cv.dnn.readNetFromTensorflow(weights, model)
    blob = cv.dnn.blobFromImage(frame, 1.0/127.5, (320, 320), [127.5, 127.5, 127.5])

    print("[INFO] computing object detections...")
    net.setInput(blob)
    output = net.forward()

    # loop over the number of detected objects
    for detection in output[0, 0, :, :]: # output[0, 0, :, :] has a shape of: (100, 7)
        # the confidence of the model regarding the detected object
        probability = detection[2]
        # if the confidence of the model is lower than 50%,
        # we do nothing (continue looping)
        if probability < 0.5:
            continue

        # extract the ID of the detected object to get
        # its name and the color associated with it
        class_id = int(detection[1])
        label = class_names[class_id - 1].upper()
        color = colors[class_id]
        B, G, R = int(color[0]), int(color[1]), int(color[2])
        # perform element-wise multiplication to get
        # the (x, y) coordinates of the bounding box
        box = [int(a * b) for a, b in zip(detection[3:7], [w, h, w, h])]
        box = tuple(box)
        # draw the bounding box of the object
        cv.rectangle(frame, box[:2], box[2:], (B, G, R), thickness=2)

        # draw the name of the predicted object along with the probability
        text = f"{label} {probability * 100:.2f}%"
        cv.putText(frame, text, (box[0], box[1]), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    '''
    '''
    # end time to compute the fps
    end = datetime.datetime.now()
    # calculate the frame per second and draw it on the frame
    fps = f"FPS: {1 / (end - start).total_seconds():.2f}"
    cv.putText(image, fps, (50, 50), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 1)
    cv.imshow("Object Detection", image)
    cv.waitKey(100)
    '''

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
