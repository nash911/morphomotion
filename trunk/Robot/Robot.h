/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   R O B O T [B A S E]   C L A S S   H E A D E R                                                              */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/


#ifndef ROBOT_H
#define ROBOT_H

// System includes
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <vector>


#include <unistd.h>  // TODO: Not sure if this should be here or in SimulationOpenRave.h

#include "ServoFeedback.h"

#define CUMULATIVE_DISTENCE_MEASUREMENT_RESOLUTION 2

using namespace std;

class Robot
{

public:

  enum RobotType{CubeN_ServoFeedBack, Tripod, Quadpod, Ybot4_ServoFeedBack, Lizard_ServoFeedBack};
  enum RobotEnvironment{SimulationOpenRave, RealWorld};
  enum EvaluationMethod{Euclidean_Distance_Final, Euclidean_Distance_Cumulative};

  // DEFAULT CONSTRUCTOR
  Robot(void);


  // DESTRUCTOR
  virtual ~Robot(void){ }

  // METHODS
  void set_robot_environment(const std::string&);
  std::string get_robot_environment(void) const;
  void set_robot_type(const std::string&);
  std::string get_robot_type(void) const;
  void set_evaluation_method(const std::string&);
  std::string get_evaluation_method(void) const;
  void set_number_of_modules(unsigned int);
  unsigned int get_number_of_modules(void) const;
  double get_distance_travelled(void);
  
  // OVERLOADED METHODS
  //Robot & operator=(Robot &rhs);

  // VIRTUAL FUNCTIONS
  virtual void reset_robot(void) = 0;
  virtual void set_moduleServo_position(unsigned int, double) = 0;
  virtual double get_moduleServo_position(unsigned int) = 0;
  virtual std::vector<double> get_all_moduleServo_position(void) = 0;  // TODO: This should be removed after implementing get_all_moduleServo_position_with_time().
  virtual void get_all_moduleServo_position_with_time(vector<ServoFeedback*>&) = 0;
  virtual void init_elapsed_evaluation_time(void) = 0;
  virtual unsigned long get_elapsed_evaluation_time(void) = 0;
  virtual double calculate_distance_travelled_euclidean(void) = 0;
  virtual void measure_cumulative_distance(void) = 0;
  virtual unsigned long step(const std::string&) = 0;

protected:
  RobotEnvironment robot_environment;
  RobotType robot_type;
  EvaluationMethod evaluation_method;
  unsigned int number_of_modules;
  unsigned long elapsed_evaluation_time;
  double distance_travelled;
};

#endif

