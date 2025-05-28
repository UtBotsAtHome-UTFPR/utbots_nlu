import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from your_package_name.action import InterpretNLU  # Replace 'your_package_name'
from rasa.nlu.model import Interpreter
import json
import os
from ament_index_python.packages import get_package_share_directory

class RasaNLUAInterpreter(Node):
    def __init__(self):
        super().__init__('rasa_nlu_interpreter')
        self.declare_parameter(
            'model_path',
            os.path.join(get_package_share_directory(self.get_name()), 'rasa', 'models')
        )
        self._action_server = ActionServer(
            self,
            InterpretNLU,
            'interpret_nlu',
            self.execute_callback)
        self.nlu_interpreter = None
        self._model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self._load_rasa_model()

    def _load_rasa_model(self):
        if self._model_path:
            try:
                self.nlu_interpreter = Interpreter.load(self._model_path)
                self.get_logger().info(f"RASA NLU model loaded from: {self._model_path}")
            except Exception as e:
                self.get_logger().warning(f"Could not load model from '{self._model_path}'. Ensure the model files are in this directory. Error: {e}")
                self.nlu_interpreter = None
        else:
            self.get_logger().warn("RASA model path not provided.")

    def execute_callback(self, goal):
        self.get_logger().info(f'Executing goal for input: "{goal.request.nlu_input.data}"')
      
        result = InterpretNLU.Result()
        result.nlu_input = goal.request.nlu_input

        if self.nlu_interpreter:
            try:
                # Parse the input text using the RASA NLU interpreter
                rasa_output = self.nlu_interpreter.parse(goal.request.nlu_input.data)

                # Format the entities and intent for the result
                entities_list = rasa_output.get('entities', [])
                intent = rasa_output.get('intent', {}).get('name', '')

                result.nlu_output.data = json.dumps(rasa_output) # Keep the full output if needed for debugging
                result.task.data = intent
                result.data.data = json.dumps(entities_list)

                # Indicate successful completion of the goal
                goal.succeed(result)
            except Exception as e:
                self.get_logger().error(f"Error during NLU interpretation: {e}")
                goal.abort()
        else:
            self.get_logger().warn("RASA NLU model not loaded. Cannot process request.")
            result.nlu_output.data = "RASA model not loaded."
            result.task.data = "error"
            result.data.data = ""
            goal.abort(result=result)

        return result

def main(args=None):
    rclpy.init(args=args)
    rasa_nlu_action_server = RasaNLUInterpreter()
    rclpy.spin(rasa_nlu_interpreter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
