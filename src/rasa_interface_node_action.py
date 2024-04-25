#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from rospkg import RosPack
import requests
import subprocess

from aiohttp import web
import asyncio

package_path = RosPack().get_path("utbots_nlu")
subprocess.Popen(["rasa", "run"], cwd=package_path)

# Node initialization
rospy.init_node('rasa_interface_node', anonymous=True)

RASA_API_URL = 'http://localhost:5005/webhooks/rest/webhook'
RASA_ACTIONS_URL = 'http://localhost:5055/webhooks/rest/webhook'

def callback_msg(msg):
    payload = {'message': msg.data}
    headers = {'content-type': 'application/json'}

    try:
        response = requests.post(RASA_API_URL, json = payload, headers=headers)
        response = response.json()[0]['text']
        
        msg_response = String()
        msg_response.data = response
        print(response)
        pub_response.publish(msg_response)
    except:
        rospy.logwarn("[NLU] Rasa not up yet")

sub_usermessage = rospy.Subscriber('/utbots/voice/stt/whispered', String, callback_msg)
pub_response = rospy.Publisher('/utbots/voice/tts/robot_speech', String, queue_size=1)

async def rasa_action(request):
    # Handle webhook payload here
    payload = await request.json()
    print("Received webhook payload:", payload["tracker"])
    return web.Response(text="Webhook received")

async def webhook_listener_setup():
    app = web.Application()
    app.router.add_post('/webhook', rasa_action)

    runner = web.AppRunner(app)
    await runner.setup()

    site = web.TCPSite(runner, 'localhost', 5055)
    await site.start()

    print("Webhook server started")
    # Keep the event loop running
    await asyncio.Event().wait()

if __name__ == "__main__":
    asyncio.run(webhook_listener_setup())
    rospy.spin()

