#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import datetime
from std_msgs.msg import Bool

class image_converter:
  def __init__(self):
    # self.camera_name = rospy.get_param("~camera_name","csi_cam_0")
    self.topic_name = rospy.get_param("~topic_name","test_publish_subscribe")

    # self.bridge = CvBridge()
    # self.image_sub = rospy.Subscriber(self.camera_name+"/image_raw/compressed",CompressedImage,self.callback)
    # self.image_pub = rospy.Publisher(self.topic_name+"/compressed",CompressedImage,queue_size=10)
    # self.object_pub = rospy.Publisher("/object_found", Bool, queue_size=10)
    self.test_publish_subscribe = rospy.Subscriber(self.topic_name , Bool, self.callback)

  def callback(self,data):
    if data == None:
        print("data is None!")
    else:
        print("data is not None!", data.shape, data)

    '''  
    try:
      frame = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

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

    target_object_class_id = 76 # assume our target class is cell phone

    # loop over the number of detected objects
    is_object_detected = 0 # Object is not found
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
        if class_id - 1 == target_object_class_id:
            is_object_detected = 1 # Object is found
            print(label, " is found!")
            self.object_pub.publish(is_object_detected)
            # break
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

    # end time to compute the fps
    end = datetime.datetime.now()
    # calculate the frame per second and draw it on the frame
    fps = f"FPS: {1 / (end - start).total_seconds():.2f}"
    cv.putText(frame, fps, (50, 50), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
    cv.imshow("Object Detection", frame)
    cv.waitKey(3)

    # If object is not detected, just publish 0 to jetbot
    self.object_pub.publish(is_object_detected)
    
    # try:
    #   self.image_pub.publish(self.bridge.cv_to_compressed_imgmsg(frame))
    # except CvBridgeError as e:
    #   print(e)
    
    # write the frame to disk
    # writer.write(frame)
    # if cv2.waitKey(10) == ord("q"):
    #    break
    '''
    '''
    for i in np.arange(0, detections.shape[2]):
      # extract the confidence (i.e., probability) associated with the
      # prediction
      confidence = detections[0, 0, i, 2]
      # filter out weak detections by ensuring the `confidence` is
      # greater than the minimum confidence
      if confidence > 40:#args["confidence"]:
          # extract the index of the class label from the `detections`,
          # then compute the (x, y)-coordinates of the bounding box for
          # the object
          idx = int(detections[0, 0, i, 1])
          box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
          (startX, startY, endX, endY) = box.astype("int")
          # display the prediction
          label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
          print("[INFO] {}".format(label))
          cv.rectangle(image, (startX, startY), (endX, endY), COLORS[idx], 2)
          y = startY - 15 if startY - 15 > 15 else startY + 15
          cv.putText(frame, label, (startX, y), cv.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
    '''

    #for (x,y,w,h) in faces:
    #    frame = cv.rectangle(frame, (x, y),(x+w, y+h), ( 0, 255, 0), 2)

    # cv.imshow("Object detection", frame)
    # cv.waitKey(3)

    #try:
    #  self.image_pub.publish(self.bridge.cv_to_compressed_imgmsg(frame))
    #except CvBridgeError as e:
    #  print(e)

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

