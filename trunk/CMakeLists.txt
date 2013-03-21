PROJECT( morphoMotion )
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_BUILD_TYPE RelWithDebInfo)
# A variable which controls the type of build when using a single-configuration generator like the Makefile generator. CMake will create by default the following variables when using a single-configuration generator:
# None (CMAKE_C_FLAGS or CMAKE_CXX_FLAGS used)
# Debug (CMAKE_C_FLAGS_DEBUG or CMAKE_CXX_FLAGS_DEBUG)
# Release (CMAKE_C_FLAGS_RELEASE or CMAKE_CXX_FLAGS_RELEASE)
# RelWithDebInfo (CMAKE_C_FLAGS_RELWITHDEBINFO or CMAKE_CXX_FLAGS_RELWITHDEBINFO
# MinSizeRel (CMAKE_C_FLAGS_MINSIZEREL or CMAKE_CXX_FLAGS_MINSIZEREL)

### dependencies
include(FindOpenRAVE.cmake)
if( NOT OPENRAVE_FOUND )
  message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
endif( NOT OPENRAVE_FOUND )
MESSAGE("OPENRAVE_LIBRARY : ${OPENRAVE_LIBRARY}")
MESSAGE("OPENRAVE_CORE_LIBRARY : ${OPENRAVE_CORE_LIBRARY}")

FIND_PACKAGE( OpenCV REQUIRED )
MESSAGE("OpenCV_LIBS : ${OpenCV_LIBS}")

### include dirs
INCLUDE_DIRECTORIES(
Evolution Evaluation Robot SimulationOpenRave Y1ModularRobot Controller ServoFeedback
Flood/Utilities Flood/MultilayerPerceptron Flood/ObjectiveFunctional Flood/TrainingAlgorithm
GraphFile FileHandler OscillationAnalyzer_OutputSignal VisualTracker SerialCommunication
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


### Neural Model Evaluator
add_executable(NeuralModelEvaluator
  Evaluation/NeuralModelEvaluator.cpp
  Flood/Perceptron/Perceptron.cpp
  Flood/MultilayerPerceptron/MultilayerPerceptron.cpp
  Flood/ObjectiveFunctional/ObjectiveFunctional.cpp  # Only for resolving dependency of TrainingAlgorithm.cpp
  Flood/TrainingAlgorithm/TrainingAlgorithm.cpp  # Only for resolving dependency of EvolutionaryAlgorithm.cpp
  Flood/TrainingAlgorithm/EvolutionaryAlgorithm.cpp  # Only for resolving dependency of FileHandler.cpp
  Robot/Robot.cpp
  SimulationOpenRave/SimulationOpenRave.cpp
  #Y1ModularRobot/Y1ModularRobot.cpp
  Controller/Controller.cpp
  ServoFeedback/ServoFeedback.cpp
  FileHandler/FileHandler.cpp
  GraphFile/GraphFile.cpp
  OscillationAnalyzer_OutputSignal/OscillationAnalyzer_OutputSignal.cpp
  #VisualTracker/VisualTracker.cpp
  #VisualTracker/disjoint_sets2.cpp
  #VisualTracker/comp_labeller_interface.cpp
  #SerialCommunication/SerialCommunication.c
)

TARGET_LINK_LIBRARIES(NeuralModelEvaluator ${OPENRAVE_CORE_LIBRARY})


