#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from vision_msgs.msg import BoundingBoxes
from std_srvs.srv import Empty
from rospkg import RosPack
import requests
import subprocess
import json

class RasaInterface:
    def __init__(self):
        package_path = RosPack().get_path("utbots_nlu")
        subprocess.Popen(["bash", "-c", "rasa run actions"], cwd=package_path)
        subprocess.Popen(["bash", "-c", "rasa run"], cwd=package_path)
        self.RASA_API_URL = 'http://localhost:5005/webhooks/rest/webhook'

        # Node initialization
        rospy.init_node('rasa_inteface_node', anonymous=True)
       
        self.sub_usermessage = rospy.Subscriber('/utbots/voice/stt/whispered', String, self.callback_msg)
        self.pub_response = rospy.Publisher('/utbots/voice/tts/robot_speech', String, queue_size=1)
        self.pub_action = rospy.Publisher('/utbots/voice/nlu/action', String, queue_size=1)
        self.pub_slot = rospy.Publisher('/utbots/voice/nlu/slot', String, queue_size=1)

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
        try:
            self.response = requests.post(self.RASA_API_URL, json = payload, headers=headers)
            self.response = self.response.json()[0]['text']
            rospy.loginfo(f"[NLU] Response: {self.response}")
            try:
                extracted_data = json.loads(self.response)
                if 'ROSACT' in extracted_data or 'SLOT' in extracted_data:
                    if extracted_data["ROSACT"] != "":
                        msg_action = String()
                        msg_action.data = extracted_data["ROSACT"]
                        self.pub_action.publish(msg_action)
                    if extracted_data["SLOT"] != "":
                        msg_slot = String()
                        msg_slot.data = extracted_data["SLOT"]
                        self.pub_slot.publish(msg_slot)
            except:
                self.msg_response = String()
                self.msg_response.data = self.response
                self.pub_response.publish(self.msg_response)
        except IndexError:
            rospy.logwarn("[NLU] No response for this request")
        except KeyError as e:
            rospy.logwarn(f"[NLU] Error: {e}")
        except Exception as e:
            if str(type(e)) == "<class 'requests.exceptions.ConnectionError'>":
                rospy.logwarn("[NLU] Rasa not up yet")
            else:
                rospy.logwarn(f"[NLU] An unexpected error occurred: {e}")

    def callback_bounding_box(self, msg):
        self.msg_bounding_boxes = msg
        self.recieved_msg_bbox = True
        print("recieved")

    def main(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    RasaInterface()