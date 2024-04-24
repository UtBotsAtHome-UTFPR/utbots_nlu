```bash
python3 -m virtualenv env
pip install rasa

cd ~/catkin_ws/src/utbots_nlu

# Initialize new project
rasa init

# Train model
rasa train

# Start server
rasa run
```
