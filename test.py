from rasa.nlu.model import Interpreter

# Path to your trained NLU model directory
nlu_model_path = "models/20240716-185035-sour-animation.tar.gz"  # Adjust this path

# Load the trained NLU model
interpreter = Interpreter.load(nlu_model_path)

# Example user message
message = "What is your name?"

# Parse the message
result = interpreter.parse(message)

# Print the results
print(result)
