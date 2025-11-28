import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from utbots_actions.action import InterpretNLU, InterpretNLUBuffer
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
        + "src/utbots_nlu/rasa/models/20250709-163318-median-reflection.tar.gz"
        default_path= default_path if os.path.exists(default_path) \
            else default_path.rsplit("utbots_nlu")[0]         \
                + "utbots_voice/utbots_nlu/rasa/models/20250709-163318-median-reflection.tar.gz"

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
        self.declare_parameter(
            'transcription_topic',
            '/utbots/voice/stt/whispered',
            ParameterDescriptor(description='Topic name for incoming transcription messages.')
        )
        self.declare_parameter(
            'transcription_buffer',
            10,
            ParameterDescriptor(description='Buffer size for storing incoming transcription messages.')
        )

        # Parameters
        self.verbose= self.get_parameter('verbose').get_parameter_value().bool_value
        self._model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self._transcription_topic = self.get_parameter('transcription_topic').get_parameter_value().string_value
        self._transcription_buffer_capacity = self.get_parameter('transcription_buffer').get_parameter_value().integer_value
        
        # Variables
        self.nlu_interpreter = None
        self._transcription_buffer = []

        # Publishers/Subscribers
        self.create_subscription(
            String,
            self._transcription_topic,
            self.subscriber_callback,
            self._transcription_buffer_capacity
        )

        # Action Servers
        self._action_server = ActionServer(
            self,
            InterpretNLU,
            '/utbots/interpret_nlu',
            self.execute_callback)

        self._action_server_buffer = ActionServer(
            self,
            InterpretNLUBuffer,
            '/utbots/interpret_nlu_buffer',
            self.buffer_execute_callback)

        # Load the RASA NLU model
        if not self._model_path:
            self.get_logger().warn("[NLU] No RASA NLU model path provided. Using default path.")
            self._model_path = default_path
        self.get_logger().info(f"[NLU] RASA NLU model path: {self._model_path}")

        if not os.path.exists(self._model_path):
            self.get_logger().warning(f"[NLU] Model path '{self._model_path}' does not exist. Using default model path.")
            self._model_path = default_path
        
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
        self.get_logger().info("[NLU] Loading Rasa model...")
        self._load_rasa_model()
        self.get_logger().info("[NLU] Sucess! Rasa model loaded.")
        self.get_logger().info("[NLU] RASA NLU Interpreter Node has been successfully initialized.")

    def _load_rasa_model(self):
        if self._model_path:
            try:
                self.nlu_interpreter = Agent.load(self._model_path,)
                self.get_logger().info(f"[NLU] RASA NLU model loaded from: {self._model_path}")
            except Exception as e:
                self.nlu_interpreter = None
                self.get_logger().warning(f"[NLU] Could not load model from '{self._model_path}'. Ensure the model files are in this directory. Error: {e}")
        else:
            self.get_logger().warn("[NLU] RASA model path not provided.")

    def interpret_nlu(self, text, goal, result):
        if self.nlu_interpreter:
            try:
                # Parse the input text using the RASA NLU interpreter (for intent and entities)
                rasa_output = asyncio.run(self.nlu_interpreter.parse_message(text))

                # Get the full response using handle_text (includes bot responses)
                response_messages = asyncio.run(self.nlu_interpreter.handle_text(text))

                # Format the entities and intent for the result
                entities_list = rasa_output.get('entities', [])
                intent = rasa_output.get('intent', {}).get('name', '')

                # Extract the bot response text
                bot_response = ""
                if response_messages:
                    # Get the last message text from the response
                    for message in response_messages:
                        if message.get('text'):
                            bot_response = message.get('text')

                result.nlu_output.data = json.dumps(rasa_output) # Keep the full NLU output for debugging
                result.task.data = intent
                result.data.data = json.dumps(entities_list)
                
                # Add bot response to the dedicated bot_response field
                if bot_response:
                    result.bot_response.data = bot_response
                    self.get_logger().info(f"[NLU] Bot Response: {bot_response}")
                else:
                    result.bot_response.data = "No response generated"

                if(self.verbose):
                    self.get_logger().info(f"[NLU] RASA NLU Output: {rasa_output}")
                    self.get_logger().info(f"[NLU] Intent: {intent}")
                    self.get_logger().info(f"[NLU] Entities: {entities_list}")
                    self.get_logger().info(f"[NLU] Bot Response: {bot_response}")

                # Indicate successful completion of the goal
                goal.succeed()
            except Exception as e:
                self.get_logger().error(f"[NLU] Error during NLU interpretation: {e}")
                goal.abort()
        else:
            self.get_logger().warn("[NLU] RASA NLU model not loaded. Cannot process request.")
            result.nlu_output.data = "RASA model not loaded."
            result.task.data = "error"
            result.data.data = ""
            goal.abort()

        return result

    def execute_callback(self, goal):
        self.get_logger().info(f'[NLU] Executing goal for input: "{goal.request.nlu_input.data}"')
      
        result = InterpretNLU.Result()
        result.nlu_input.data = goal.request.nlu_input.data

        result = self.interpret_nlu(goal.request.nlu_input.data, goal, result)

        return result
    
    def buffer_execute_callback(self, goal):
        buffered_input = self._transcription_buffer[-1] if self._transcription_buffer else ""
        self._transcription_buffer.pop(0) if self._transcription_buffer else None

        self.get_logger().info(f'[NLU] Executing goal for input: "{buffered_input}"')

        result = InterpretNLUBuffer.Result()
        result.nlu_input.data = buffered_input

        result = self.interpret_nlu(buffered_input, goal, result)

        return result

    def subscriber_callback(self, msg):
        self.get_logger().info(
            f"[NLU] Received transcription on '{self._transcription_topic}': {msg.data}" 
        )
        self._transcription_buffer.append(msg.data)
        if len(self._transcription_buffer) > self._transcription_buffer_capacity:
            self._transcription_buffer.pop(0)  # discard oldest entry
            self.get_logger().info(
            f"[NLU] Buffer full. Discarded oldest transcription.")

def main(args=None):
    rclpy.init(args=args)
    rasa_nlu_interpreter = RasaNLUInterpreter()
    rclpy.spin(rasa_nlu_interpreter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
