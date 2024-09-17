#!/usr/bin/venv_utbots_nlu/bin/python

import rospy
import roslib
roslib.load_manifest('utbots_nlu')
from std_msgs.msg import String
from rospkg import RosPack
import actionlib
from utbots_actions.msg import InterpretNLUAction, InterpretNLUResult
import requests
import subprocess
import json
from std_msgs.msg import String

class RasaInterface:
    def __init__(self):
        package_path = RosPack().get_path("utbots_nlu")
        subprocess.Popen(["bash", "-c", "rasa run actions"], cwd=package_path)
        subprocess.Popen(["bash", "-c", "rasa run"], cwd=package_path)
        self.RASA_API_URL = 'http://localhost:5005/webhooks/rest/webhook'

        is_rasa_running = False

        #Only start server when rasa is up and running
        while(is_rasa_running == False):
            is_rasa_running = self.is_rasa_running(self.RASA_API_URL)

        # Node initialization
        rospy.init_node('rasa_inteface_node', anonymous=True)
        
        # Action server initialization
        self.server = actionlib.SimpleActionServer('interpret_nlu', InterpretNLUAction, self.execute, False)
        self.server.start()

        # Publishers and Subscribers
        self.sub_usermessage = rospy.Subscriber('/utbots/voice/stt/whispered', String, self.callback_msg)
        self.pub_response = rospy.Publisher('/utbots/voice/tts/robot_speech', String, queue_size=1)

        self.response = ""
        self.enable_nlu = False

        self.msg_whisper = String()
        self.new_msg = False

        self.rate = rospy.Rate(1) # 30hz

        self.main()

    def is_rasa_running(self,rasa_api_url):
        try:
            response = requests.get(rasa_api_url)
            if response.status_code == 405:
                #Rasa is up and running!
                return True
            else:
                #Rasa is up but not running
                return False
        except requests.ConnectionError:
            #Failed to connect to Rasa.
            return False

    def callback_msg(self, msg):
        self.new_msg = True
        self.msg_whisper = msg

    def execute(self, goal):
        rospy.loginfo(f"[NLU] Goal received, waiting for request")
        while self.new_msg == False:
            self.rate.sleep()
            if self.server.is_preempt_requested():
                rospy.loginfo("[NLU] Action preempted")
                self.server.set_preempted()
                return
        
        payload = {'message': self.msg_whisper.data}
        headers = {'content-type': 'application/json'}
        try:
            self.response = requests.post(self.RASA_API_URL, json = payload, headers=headers)
            self.response = self.response.json()[0]['text']
            rospy.loginfo(f"[NLU] Request: {self.msg_whisper.data}")
            rospy.loginfo(f"[NLU] Response: {self.response}")
            action_res = InterpretNLUResult()
            action_res.NLUInput = String(self.msg_whisper.data)
            action_res.NLUOutput = String(self.response)
        
            try:
                extracted_data = json.loads(self.response)
                if 'ROSACT' in extracted_data or 'SLOT' in extracted_data:
                    if extracted_data["ROSACT"] != "":
                        action_res.Task = String(extracted_data["ROSACT"])
                    if extracted_data["SLOT"] != "":
                        action_res.Data =  String(extracted_data["SLOT"])
            except:
                rospy.loginfo("[NLU] No command interpreted, TTS response published")
                action_res.Task =  String()
                action_res.Data =  String()
                self.msg_response = String()
                self.msg_response.data = self.response
                self.pub_response.publish(self.msg_response)
        except IndexError:
            rospy.logwarn("[NLU] No response for this request")
            action_res = InterpretNLUResult()
            action_res.NLUInput = String(self.msg_whisper.data)
            action_res.NLUOutput = String("No response for this request")
        except KeyError as e:
            rospy.logwarn(f"[NLU] Error: {e}")
            self.server.set_aborted()
        except Exception as e:
            if str(type(e)) == "<class 'requests.exceptions.ConnectionError'>":
                rospy.logwarn("[NLU] Rasa not up yet")
                rospy.loginfo("[NLU] Action aborted")
                self.server.set_aborted()
            else:
                rospy.logwarn(f"[NLU] An unexpected error occurred: {e}")
                rospy.loginfo("[NLU] Action aborted")
                self.server.set_aborted()

        self.new_msg = False
        rospy.loginfo("[NLU] Action succeded. Sending result")
        self.server.set_succeeded(action_res)

    def main(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    RasaInterface()
