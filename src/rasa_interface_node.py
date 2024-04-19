#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import requests

class RasaInterface:
    def __init__(self):
        self.RASA_API_URL = 'http://localhost:5005/webhooks/rest/webhook'

        # Node initialization
        rospy.init_node('rasa_inteface_node', anonymous=True)

        self.sub_usermessage = rospy.Subscriber('/whispered', String, self.callback_msg)
        self.pub_response = rospy.Publisher('/response', String, queue_size=1)
        
        self.rate = rospy.Rate(30) # 30hz

        self.main()

    def callback_msg(self, msg):
        payload = {'message': msg.data}
        headers = {'content-type': 'application/json'}

        response = requests.post(self.RASA_API_URL, json = payload, headers=headers)
        response = response.json()[0]['text']
        
        self.msg_response = String()
        self.msg_response.data = response
        print(response)
        self.pub_response.publish(self.msg_response)

    def main(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    RasaInterface()