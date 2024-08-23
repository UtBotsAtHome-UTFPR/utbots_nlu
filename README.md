# utbots_nlu
- ROS package that implements [RASA](https://rasa.com/) 
- Can be trained for detecting several intentions by generalizing text examples (data/nlu.yaml and domain.yaml files).
- Can be trained for detecting entities names (name of a room, an object, etc.) based in context (data/nlu.yaml and domain.yaml files).
- Intentions can have strict responses or responses based on history context (responsed defined in domain.yml, strict intention responses in rules.yml and context-based in stories.yml)
- The integration in ROS is made with actions and can detect text-based responses (sent to TTS) and a operator-given command with associated data.

## Installation

### Building (if downloaded outside utbots_voice)

```bash
cd ~/catkin_ws/src
git clone https://github.com/UtBotsAtHome-UTFPR/utbots_nlu.git
cd ..
catkin_make
```

### Dependencies

This package must be used alongside the [utbots_dependencies](https://github.com/UtBotsAtHome-UTFPR/utbots_dependencies) as it uses some of the message and action definitions. You can do this with:

```bash
cd ~/catkin_ws/src/
git clone https://github.com/UtBotsAtHome-UTFPR/utbots_dependencies.git
cd ..
catkin_make
```

The code runs on Python 3 and you must use a virtualenv (Install with `pip install virtualenv`) with the path `/usr/bin/utbots_nlu_env/bin/python` as the node expects its existence to run.. Install RASA and other Python requirements:

```bash
cd /usr/bin
sudo python3 -m virtualenv utbots_nlu_env --python=$(which python3)
roscd utbots_nlu
/usr/bin/utbots_nlu_env/bin/python -m pip install -r requirements.txt
```

## Running
To run interface RASA, you first must enable the node for listening to STT text, that must be done sending an empty goal to the action server. Then, when an STT callback occurs, it will process the text and output the result, disabling the NLU wait for STT text. If the intention detected is a verbal response, it will publish in the TTS topic. If the intention detected is an operator command, it will return in the results a Task information and a Data information for task-associated data. In both cases, the NLU input and output are sent as results, for log purposes.

First, initialize ROS (if not already):

```bash
roscore
```

Then, run the node:

```bash
rosrun utbots_nlu rasa_interface_node.py
```

### Getting NLU responses

Every new NLU process must be enabled with an action cal. The enable control can be made inside a script with an ActionClient. To enable in the terminal:

```bash
rostopic pub /interpret_nlu/goal utbots_actions/InterpretNLUActionGoal "header:  
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal: {}"               
```

Then it starts to wait for an STT callback. To test in the terminal, change the `<nlu input>`:

```bash
rostopic pub /utbots/voice/stt/whispered std_msgs/String "<nlu input>"   
```

To see the action result in the terminal:
```bash
rostopic echo /interpret_nlu/result
```

## Training

### Model training examples
- **domain.yml**: program new *intentions*, *entities*, *responses* and *actions* (different than ROS actions, allows treatment of entity values in Python code)
- **data/nlu.yml**: write new examples for intents
- **data/rules.yml**: program direct responses for intentions
- **data/stories.yml**: program context-based responses (responses changed bases with different previous intents)
- **actions/actions.py**: actions programmed in the domain.yml must be programmed in this file too

### Train
To train a new model, in the terminal:
```bash
roscd utbots_nlu
rasa train
```

### Debugging dialog
A dialog with the RASA tool can be done without ROS, in the terminal:
```bash
roscd utbots_nlu
rasa shell
```
