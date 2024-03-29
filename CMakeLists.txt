#-------CMakeList for Ubuntu 12.04 with OpenRAVE 0.8-----------#

cmake_minimum_required (VERSION 2.6.0)
PROJECT( morphoMotion )
set(CMAKE_VERBOSE_MAKEFILE true)

find_package(OpenRAVE 0.8 REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT COMPONENTS iostreams python thread)


set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Make sure the openmr plugin is installed!
# Get the openrave share directory
EXEC_PROGRAM("openrave-config --share-dir" OUTPUT_VARIABLE OpenRAVE_SHARE_DIR)

find_library(OPENMR openmr ${OpenRAVE_SHARE_DIR}/plugins)
message(STATUS "Looking for OpenMR... ${OPENMR}")

if (NOT OPENMR)
  message(SEND_ERROR "OPENMR not found")
endif()


# Include and link directories
include_directories(${OpenRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} )
if( Boost_INCLUDE_DIRS )
  include_directories(${Boost_INCLUDE_DIRS})
endif()

link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

FIND_PACKAGE( OpenCV REQUIRED )
MESSAGE("OpenCV_LIBS : ${OpenCV_LIBS}")

ADD_DEFINITIONS(-std=c++0x)
ADD_DEFINITIONS(-pthread)
#ADD_DEFINITIONS(-llapack)
#ADD_DEFINITIONS(-lblas)

### include dirs
INCLUDE_DIRECTORIES(
Evolution Evaluation Robot SimulationOpenRave Y1ModularRobot Controller HybridController SineController SimpleController
InverseSineController InverseSineController_V2 InverseSineController_V3 FourierController TriangleSquareController
Flood/Utilities Flood/MultilayerPerceptron Flood/ObjectiveFunctional Flood/TrainingAlgorithm
ServoFeedback ExtKalmanFilter_SinusModel GraphFile FileHandler OscillationAnalyzer_OutputSignal VisualTracker SerialCommunication
)


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
  HybridController/HybridController.cpp
  SineController/SineController.cpp
  SimpleController/SimpleController.cpp
  InverseSineController/InverseSineController.cpp
  InverseSineController_V2/InverseSineController_V2.cpp
  InverseSineController_V3/InverseSineController_V3.cpp
  FourierController/FourierController.cpp
  TriangleSquareController/TriangleSquareController.cpp
  ServoFeedback/ServoFeedback.cpp
  ExtKalmanFilter_SinusModel/ExtKalmanFilter_SinusModel.cpp
  FileHandler/FileHandler.cpp
  GraphFile/GraphFile.cpp
  OscillationAnalyzer_OutputSignal/OscillationAnalyzer_OutputSignal.cpp
  VisualTracker/VisualTracker.cpp
  VisualTracker/disjoint_sets2.cpp
  VisualTracker/comp_labeller_interface.cpp
  SerialCommunication/SerialCommunication.c
)

### executable
set_target_properties(Evolve PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Evolve PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Evolve ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY} ${OpenCV_LIBS} -std=c++0x -pthread -larmadillo -g)
#target_link_libraries(Evolve ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY} ${OpenCV_LIBS} -llapack -lblas)


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
  HybridController/HybridController.cpp
  SineController/SineController.cpp
  SimpleController/SimpleController.cpp
  InverseSineController/InverseSineController.cpp
  InverseSineController_V2/InverseSineController_V2.cpp
  InverseSineController_V3/InverseSineController_V3.cpp
  FourierController/FourierController.cpp
  TriangleSquareController/TriangleSquareController.cpp
  ServoFeedback/ServoFeedback.cpp
  ExtKalmanFilter_SinusModel/ExtKalmanFilter_SinusModel.cpp
  FileHandler/FileHandler.cpp
  GraphFile/GraphFile.cpp
  OscillationAnalyzer_OutputSignal/OscillationAnalyzer_OutputSignal.cpp
  VisualTracker/VisualTracker.cpp
  VisualTracker/disjoint_sets2.cpp
  VisualTracker/comp_labeller_interface.cpp
  SerialCommunication/SerialCommunication.c
)

### opencv executable
set_target_properties(Evaluate PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Evaluate PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Evaluate ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY} ${OpenCV_LIBS} -std=c++0x -pthread -larmadillo -g)
#target_link_libraries(Evaluate ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY} ${OpenCV_LIBS} -llapack -lblas)


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
  Controller/Controller.cpp
  HybridController/HybridController.cpp
  SineController/SineController.cpp
  SimpleController/SimpleController.cpp
  InverseSineController/InverseSineController.cpp
  InverseSineController_V2/InverseSineController_V2.cpp
  InverseSineController_V3/InverseSineController_V3.cpp
  FourierController/FourierController.cpp
  TriangleSquareController/TriangleSquareController.cpp
  ServoFeedback/ServoFeedback.cpp
  ExtKalmanFilter_SinusModel/ExtKalmanFilter_SinusModel.cpp
  FileHandler/FileHandler.cpp
  GraphFile/GraphFile.cpp
  OscillationAnalyzer_OutputSignal/OscillationAnalyzer_OutputSignal.cpp
)

set_target_properties(NeuralModelEvaluator PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(NeuralModelEvaluator PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(NeuralModelEvaluator ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY} -std=c++0x -pthread -larmadillo)
#target_link_libraries(NeuralModelEvaluator ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY} -llapack -lblas)

### Serial Communication Evaluator
add_executable(SerialCommunicationEvaluator
  SerialCommunication/SerialCommunicationEvaluator.cpp
  SerialCommunication/SerialCommunication.c
)
TARGET_LINK_LIBRARIES(SerialCommunicationEvaluator -std=c++0x -pthread)


### VisualTracker Evaluator
add_executable(VisualTrackerEvaluator
  VisualTracker/VisualTrackerApplication.cpp
  VisualTracker/VisualTracker.cpp
  VisualTracker/disjoint_sets2.cpp
  VisualTracker/comp_labeller_interface.cpp
)

# target_link_libraries(VisualTrackerEvaluator ${OpenCV_LIBS} VisualTracker)
target_link_libraries(VisualTrackerEvaluator ${OpenCV_LIBS})


### Extended Kalman Filter Evaluator
add_executable(ExtKalmanFilterEvaluator
  ExtKalmanFilter_SinusModel/ExtKalmanFilterEvaluator.cpp
  ExtKalmanFilter_SinusModel/ExtKalmanFilter_SinusModel.cpp
)

target_link_libraries(ExtKalmanFilterEvaluator -larmadillo)


### HOAP Evaluator
add_executable(Hoap
  Evaluation/HOAP.cpp
  Robot/Robot.cpp
  SimulationOpenRave/SimulationOpenRave.cpp
  ServoFeedback/ServoFeedback.cpp
  ExtKalmanFilter_SinusModel/ExtKalmanFilter_SinusModel.cpp
)

### executable
set_target_properties(Hoap PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Hoap PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Hoap ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY} -std=c++0x -pthread -larmadillo -g)

