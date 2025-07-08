import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from utbots_actions.action import InterpretNLU
from rasa.core.agent import Agent
import json
import asyncio
import os
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor

class RasaNLUInterpreter(Node):
    def __init__(self):
        super().__init__('rasa_nlu_interpreter')
        default_path= get_package_share_directory("utbots_nlu").rsplit("install")[0] \
        + "src/utbots_nlu/rasa/models/20250528-170450-black-bollard.tar.gz"
        default_path= default_path if os.path.exists(default_path) \
            else default_path.rsplit("utbots_nlu")[0]         \
                + "utbots_voice/utbots_nlu/rasa/models/20250528-170450-black-bollard.tar.gz"

        self.declare_parameter(
            'model_path',
            default_path,
            ParameterDescriptor(description='Rasa custom model path. Default is a sample model path.'),
        )
        self.declare_parameter(
            'verbose',
            False,
            descriptor=ParameterDescriptor(
                description='Enable verbose logging for the RASA NLU interpreter. Default is False.') 
        )


        self.nlu_interpreter = None
        self._model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not self._model_path:
            self.get_logger().warn("No RASA NLU model path provided. Using default path.")
            self._model_path = default_path
        self.get_logger().info(f"RASA NLU model path: {self._model_path}")
        # Load the RASA NLU model
        if not os.path.exists(self._model_path):
            self.get_logger().warning(f"Model path '{self._model_path}' does not exist. Using default model path.")
            self._model_path = default_path
        self.verbose= self.get_parameter('verbose').get_parameter_value().bool_value
        
        if not self.verbose:
            import logging
            # Suppress RASA and TensorFlow logs if verbose is False
            logging.basicConfig(level=logging.ERROR)
            logging.getLogger('rasa.nlu').setLevel(logging.ERROR)
            logging.getLogger('rasa.nlu.classifiers').setLevel(logging.ERROR)
            logging.getLogger('rasa.nlu.extractors').setLevel(logging.ERROR)
            logging.getLogger('rasa.nlu.interpreter').setLevel(logging.ERROR)
            logging.getLogger('rasa.nlu.training_data').setLevel(logging.ERROR)
            logging.getLogger('rasa.nlu.utils').setLevel(logging.ERROR)
            logging.getLogger('rasa.nlu.config').setLevel(logging.ERROR)
            logging.getLogger('rasa.nlu.model').setLevel(logging.ERROR)
            logging.getLogger('rasa.nlu.components').setLevel(logging.ERROR)
            logging.getLogger('rasa.nlu.registry').setLevel(logging.ERROR)
            logging.getLogger('rasa.nlu.training').setLevel(logging.ERROR)
            logging.getLogger('rasa.nlu.training_data.loading').setLevel(logging.ERROR)
            logging.getLogger('rasa.nlu.training_data.formats').setLevel(logging.ERROR) 
            logging.getLogger('rasa').setLevel(logging.ERROR)
            logging.getLogger('rasa.core').setLevel(logging.ERROR)
            logging.getLogger('rasa.model').setLevel(logging.ERROR)
            logging.getLogger('tensorflow').setLevel(logging.ERROR)
            logging.getLogger('apscheduler').setLevel(logging.ERROR)

        # Initialize the RASA NLU interpreter
        self.get_logger().info("Loading Rasa model...")
        self._load_rasa_model()
        self.get_logger().info("Sucess! Rasa model loaded.")

        # Create an action server for interpreting NLU input
        self._action_server = ActionServer(
            self,
            InterpretNLU,
            '/utbots/interpret_nlu',
            self.execute_callback)
        self.get_logger().info("RASA NLU Interpreter Node has been initialized.")

    def _load_rasa_model(self):
        if self._model_path:
            try:
                self.nlu_interpreter = Agent.load(self._model_path,)
                self.get_logger().info(f"RASA NLU model loaded from: {self._model_path}")
            except Exception as e:
                self.nlu_interpreter = None
                self.get_logger().warning(f"Could not load model from '{self._model_path}'. Ensure the model files are in this directory. Error: {e}")
        else:
            self.get_logger().warn("RASA model path not provided.")

    def execute_callback(self, goal):
        self.get_logger().info(f'Executing goal for input: "{goal.request.nlu_input.data}"')
      
        result = InterpretNLU.Result()
        result.nlu_input.data = goal.request.nlu_input.data

        if self.nlu_interpreter:
            try:
                # Parse the input text using the RASA NLU interpreter
                rasa_output = asyncio.run(self.nlu_interpreter.parse_message(goal.request.nlu_input.data))

                # Format the entities and intent for the result
                entities_list = rasa_output.get('entities', [])
                intent = rasa_output.get('intent', {}).get('name', '')

                result.nlu_output.data = json.dumps(rasa_output) # Keep the full output if needed for debugging
                result.task.data = intent
                result.data.data = json.dumps(entities_list)
                if(self.verbose):
                    self.get_logger().info(f"RASA NLU Output: {result.nlu_output.data}")
                    self.get_logger().info(f"Intent: {intent}")
                    self.get_logger().info(f"Entities: {entities_list}")

                # Indicate successful completion of the goal
                goal.succeed()
            except Exception as e:
                self.get_logger().error(f"Error during NLU interpretation: {e}")
                goal.abort()
        else:
            self.get_logger().warn("RASA NLU model not loaded. Cannot process request.")
            result.nlu_output.data = "RASA model not loaded."
            result.task.data = "error"
            result.data.data = ""
            goal.abort()

        return result

def main(args=None):
    rclpy.init(args=args)
    rasa_nlu_interpreter = RasaNLUInterpreter()
    rclpy.spin(rasa_nlu_interpreter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
