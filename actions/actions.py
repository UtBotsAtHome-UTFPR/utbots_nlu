# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions

from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher

import json

class DescribeAmbient(Action):
    def name(self) -> Text:
        return "action_describe_ambient"
    
    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        string_action_data = {
            "ROSACT": "DESCRIBE_AMBIENT",
            "SLOT": ""
        }
        json_string = json.dumps(string_action_data)

        dispatcher.utter_message(json_string)

        return []

class GoToLocation(Action):
    def name(self) -> Text:
        return "action_go_to_location"
    
    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        location_entity = next(tracker.get_latest_entity_values('room'), None)

        string_action_data = {
            "ROSACT": "GO_TO",
            "SLOT": "KITCHEN"
        }
        json_string = json.dumps(string_action_data)

        if location_entity:
            dispatcher.utter_message(json_string)
            #dispatcher.utter_message(text=f"I will navigate to {location_entity}")
        else:
            dispatcher.utter_message(text=f"I could not identify the destiny location")

        return []
    
