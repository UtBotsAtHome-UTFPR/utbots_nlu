#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from vision_msgs.msg import BoundingBoxes
from std_srvs.srv import Empty
from rospkg import RosPack
import requests
import subprocess

class RasaInterface:
    def __init__(self):
        package_path = RosPack().get_path("utbots_nlu")
        subprocess.Popen(["./setup.sh"], cwd=package_path)
        self.RASA_API_URL = 'http://localhost:5005/webhooks/rest/webhook'

        # Node initialization
        rospy.init_node('rasa_inteface_node', anonymous=True)
       
        self.sub_usermessage = rospy.Subscriber('/utbots/voice/stt/whispered', String, self.callback_msg)
        self.pub_response = rospy.Publisher('/utbots/voice/tts/robot_speech', String, queue_size=1)

        self.response = ""

        # Describe ambient action
        self.sub_detections = rospy.Subscriber('/utbots/vision/detection/bounding_boxes', BoundingBoxes, self.callback_bounding_box)
        self.msg_bounding_boxes = BoundingBoxes()
        self.recieved_msg_bbox = False

        self.rate = rospy.Rate(30) # 30hz

        self.main()

    def callback_msg(self, msg):
        payload = {'message': msg.data}
        headers = {'content-type': 'application/json'}
        print("request")

        try:
            self.response = requests.post(self.RASA_API_URL, json = payload, headers=headers)
            self.response = self.response.json()[0]['text']
            
            if self.response != "[ROSACT]DESCRIBE_AMBIENT":
                self.msg_response = String()
                self.msg_response.data = self.response
                print(self.response)
                self.pub_response.publish(self.msg_response)
        except:
            rospy.logwarn("[NLU] Rasa not up yet")

    def callback_bounding_box(self, msg):
        self.msg_bounding_boxes = msg
        self.recieved_msg_bbox = True
        print("recieved")

    def describe_ambient(self):
        self.recieved_msg_bbox = False
        self.response = ""

        objects_count = {}
        msg_yolo_response = String()
        msg_yolo_response.data = "I see the following: "
        for bbox in self.msg_bounding_boxes.bounding_boxes:
            if bbox.Class in objects_count:
                objects_count[bbox.Class] += 1
            else:
                objects_count[bbox.Class] = 1

        for object, count in objects_count.items():
            msg_yolo_response.data += ", " + str(count) + " " + object

        print(msg_yolo_response)

        self.pub_response.publish(msg_yolo_response)

    def main(self):
        while not rospy.is_shutdown():
            if self.response == "[ROSACT]DESCRIBE_AMBIENT":
                print(self.recieved_msg_bbox)
                if self.recieved_msg_bbox == True:
                    self.describe_ambient()
                else:
                    rospy.wait_for_service('/yolov8_server')
                    try:
                        # Create a proxy to call the service
                        empty_service_proxy = rospy.ServiceProxy('/yolov8_server', Empty)

                        # Call the service
                        response = empty_service_proxy()

                        # Print response if needed
                        print("Service call succeeded")
                    except rospy.ServiceException as e:
                        print("Service call failed:", e)

            self.rate.sleep()

if __name__ == "__main__":
    RasaInterface()