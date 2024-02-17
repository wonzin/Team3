#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import datetime
import cv2
import numpy as np
from multiprocessing import Process, Queue
import yaml
from std_msgs.msg import String

# Load your classes and colors (assuming YAML file is properly formatted)
with open("/home/orin/catkin_ws/src/jetbot_pro/scripts/yolov8/coco128.yaml", 'r') as file:
    CLASSES = yaml.safe_load(file)["names"]
colors = np.random.uniform(0, 255, size=(len(CLASSES), 3))

def image_subscriber(queue):

    def callback(msg):
        br = CvBridge()
        image = br.compressed_imgmsg_to_cv2(msg, "bgr8")
        queue.put(image)
        rospy.loginfo('Image received and put in queue')

    rospy.Subscriber('imagetimer', CompressedImage, callback)
    rospy.spin()

def cal_undistort(img, objpoints, imgpoints):
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[1:], None, None)
    undist = cv2.undistort(img, mtx, dist, None, mtx)
    return undist

def calibration(image):
    npz_path = "/home/orin/catkin_ws/src/jetbot_pro/scripts/calibrate_data/intrinsicCalibration.npz"
    dist_arr = np.load(npz_path)
    objpoints = dist_arr["objpoints"]
    imgpoints = dist_arr["imgpoints"]
    undistorted = cal_undistort(image, objpoints, imgpoints)
    return undistorted

def object_detection(image):
    onnx_model = '/home/orin/catkin_ws/src/jetbot_pro/scripts/yolov8/yolov8m.onnx'
    model: cv2.dnn.Net = cv2.dnn.readNetFromONNX(onnx_model)
    blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255, size=(640, 640), swapRB=True)
    model.setInput(blob)
    # Perform inference
    outputs = model.forward()
    return [image, outputs]

def image_processor(input_queue, output_queue):

    while True:
        if not input_queue.empty():
            image = input_queue.get()
            # Perform image calibration and object detection here
            calibrated_img = calibration(image)
            output = object_detection(calibrated_img)
            # For simplicity, we'll just pass the image through
            output_queue.put(output)
            print("Image processed and put in output queue")

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
    label = f"{CLASSES[class_id]} ({confidence:.2f})"
    color = colors[class_id]
    cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
    cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def display_output(image, outputs):
    # rospy.init_node('image_processor_node', anonymous=True)

    pub = rospy.Publisher('chatter', String, queue_size=10)


    # Prepare output array
    [height, width, _] = image.shape
    # Prepare a square image for inference
    length = max((height, width))
    # Calculate scale factor
    scale = length / 640


    start = datetime.datetime.now() # fps count start

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
            image,
            class_ids[index],
            scores[index],
            round(box[0] * scale),
            round(box[1] * scale),
            round((box[0] + box[2]) * scale),
            round((box[1] + box[3]) * scale),
        )

        class_id = class_ids[index]
        # class_name = CLASSES[class_id]
        if class_id == 39 or class_id == 24 or class_id==0 or class_id == 67 or class_id==62 or class_id==63 or class_id == 56:
            print(CLASSES[class_id], " detected and published!")
            pub.publish("True")
        # print(class_name, " published!")
        # print("class_id: ", class_id)
    end = datetime.datetime.now()
    fps = f"FPS: {1 / (end-start).total_seconds():.2f}"
    print("fps: ", fps)

def main():
    rospy.init_node('image_subscriber_node', anonymous=True)

    input_queue = Queue()
    output_queue = Queue()

    # Process for subscribing to images
    subscriber_process = Process(target=image_subscriber, args=(input_queue,))
    subscriber_process.start()

    # Process for processing images
    processor_process = Process(target=image_processor, args=(input_queue, output_queue))
    processor_process.start()

    # Main loop where you can handle the processed images
    try:
        while True:
            if not output_queue.empty():
                image, outputs = output_queue.get()
                # Handle the processed image (display, save, etc.)
                display_output(image, outputs)
                print("Processed image received in main")

    except KeyboardInterrupt:
        print("Shutting down")
    subscriber_process.join()
    processor_process.join()


if __name__ == '__main__':
    
    main()

