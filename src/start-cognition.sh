######################################################
# Roboy Cognition env
######################################################

export COGNITION_WS="/home/roboy/workspace/Roboy"
source $COGNITION_WS/devel/setup.bash

export NEO4J_USERNAME="memory"
export NEO4J_ADDRESS=""
export NEO4J_PASSWORD=""

######################################################

# if [ ! -d /var/run/neo4j ]; then
# 	sudo mkdir /var/run/neo4j
	
# fi
# sudo neo4j start

# { # try
#     rostopic list
# } || { # catch
#     echo "[INFO]: ROS core wasn't running. Starting now..."
#     roscore
# }

# sleep 4

cd $COGNITION_WS
source devel/setup.bash
roslaunch roboy_cognition_manager roboy.launch &
sleep 5
echo "STARTED roboy_cognition_manager"
cd src/DeepQA
./advertise_gnlp_service &
echo "STARTED roboy_generative_nlp"


cd $COGNITION_WS/src/roboy_dialog/roboy_memory/target
java -jar roboy_memory-1.0.0-jar-with-dependencies.jar &
echo "STARTED roboy_memory"

rosservice call /roboy/cognition/generative_nlp/answer "text_input: 'hey ho'"
