PROJECT( morphoMotion )
cmake_minimum_required(VERSION 2.8)

### dependencies
include(FindOpenRAVE.cmake)
if( NOT OPENRAVE_FOUND )
  message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
endif( NOT OPENRAVE_FOUND )
#~ MESSAGE("OPENRAVE_LIBRARIES : ${OPENRAVE_LIBRARIES}")
#~ MESSAGE("OpenRAVE_LIBRARIES : ${OpenRAVE_LIBRARIES}")
MESSAGE("OPENRAVE_LIBRARY : ${OPENRAVE_LIBRARY}")
MESSAGE("OPENRAVE_CORE_LIBRARY : ${OPENRAVE_CORE_LIBRARY}")

FIND_PACKAGE( OpenCV REQUIRED )
MESSAGE("OpenCV_LIBS : ${OpenCV_LIBS}")

### include dirs
INCLUDE_DIRECTORIES(
Evolution Robot SimulationOpenRave Y1ModularRobot Controller FileHandler
Flood/Utilities Flood/MultilayerPerceptron
Flood/ObjectiveFunctional Flood/TrainingAlgorithm
GraphFile OscillationAnalyzer_OutputSignal rs232 VisualTracker
ServoFeedback SerialCommunication
)

### objects
add_executable(Evolve
  Evolution/EvolveController.cpp
  Flood/Perceptron/Perceptron.cpp
  Flood/MultilayerPerceptron/MultilayerPerceptron.cpp
  Flood/ObjectiveFunctional/ObjectiveFunctional.cpp
  Flood/TrainingAlgorithm/TrainingAlgorithm.cpp
  Flood/TrainingAlgorithm/EvolutionaryAlgorithm.cpp
  Evolution/Evolution.cpp
  Robot/Robot.cpp
  SimulationOpenRave/SimulationOpenRave.cpp
  Y1ModularRobot/Y1ModularRobot.cpp
  Controller/Controller.cpp
  ServoFeedback/ServoFeedback.cpp
  FileHandler/FileHandler.cpp
  GraphFile/GraphFile.cpp
  OscillationAnalyzer_OutputSignal/OscillationAnalyzer_OutputSignal.cpp
  VisualTracker/VisualTracker.cpp
  VisualTracker/disjoint_sets2.cpp
  VisualTracker/comp_labeller_interface.cpp
  SerialCommunication/SerialCommunication.c
)

#TARGET_LINK_LIBRARIES(Evolve ${OPENRAVE_CORE_LIBRARY})
TARGET_LINK_LIBRARIES(Evolve ${OPENRAVE_CORE_LIBRARY} ${OpenCV_LIBS})

add_executable(Evaluate
  Evaluation/EvaluateController.cpp
  Flood/Perceptron/Perceptron.cpp
  Flood/MultilayerPerceptron/MultilayerPerceptron.cpp
  Flood/ObjectiveFunctional/ObjectiveFunctional.cpp  # Only for resolving dependency of TrainingAlgorithm.cpp
  Flood/TrainingAlgorithm/TrainingAlgorithm.cpp  # Only for resolving dependency of EvolutionaryAlgorithm.cpp
  Flood/TrainingAlgorithm/EvolutionaryAlgorithm.cpp  # Only for resolving dependency of FileHandler.cpp
  Robot/Robot.cpp
  SimulationOpenRave/SimulationOpenRave.cpp
  Y1ModularRobot/Y1ModularRobot.cpp
  Controller/Controller.cpp
  ServoFeedback/ServoFeedback.cpp
  FileHandler/FileHandler.cpp
  GraphFile/GraphFile.cpp
  OscillationAnalyzer_OutputSignal/OscillationAnalyzer_OutputSignal.cpp
  VisualTracker/VisualTracker.cpp
  VisualTracker/disjoint_sets2.cpp
  VisualTracker/comp_labeller_interface.cpp
  SerialCommunication/SerialCommunication.c
)

### opencv executable
TARGET_LINK_LIBRARIES(Evaluate ${OPENRAVE_CORE_LIBRARY} ${OpenCV_LIBS})


