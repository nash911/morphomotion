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

  enum RobotType{CubeN_ServoFeedBack, Tripod, Quadpod, Ybot4_ServoFeedBack, Lizard, Quad_3D, Bunny,
                 Bunny_StiffSpine, Lizard_3D, MultiDof_7_tripod, MultiDof_9_quad, MultiDof_11_4};
  enum RobotPriority{Robot_Primary, Robot_Secondary};
  enum RobotEnvironment{SimulationOpenRave, Y1};
  enum EvaluationMethod{Euclidean_Distance_Final, Euclidean_Distance_Cumulative};

  //-- DEFAULT CONSTRUCTOR
  Robot();


  //-- DESTRUCTOR
  virtual ~Robot(void){ }

  //-- METHODS
  void set_robot_environment(const std::string&);
  std::string get_robot_environment(void) const;
  void set_robot_priority(const std::string&);
  std::string get_robot_priority(void) const;
  void set_robot_type(const std::string&);
  std::string get_robot_type(void) const;
  void set_evaluation_method(const std::string&);
  std::string get_evaluation_method(void) const;
  void set_number_of_modules(unsigned int);
  unsigned int get_number_of_modules(void) const;
  unsigned long get_initial_evaluation_time(void);
  double get_distance_travelled(void);
  void set_receive_broadcast(bool);
  bool get_receive_broadcast();
  void set_broadcast_thread(bool);
  bool get_broadcast_thread();
  bool get_processing_flag(void);
  void set_processing_flag(bool);
  double calculate_random_uniform(double, double);
  
  //-- VIRTUAL FUNCTIONS
  virtual void copy(const Robot*) = 0;
  virtual void reset_robot(void) = 0;
  virtual void reset_modules(void) = 0;
  virtual void reset_comm_link(void) = 0;
  virtual void set_sinusoidal_controller_parameters(const vector<double>&, const vector<double>&, const vector<double>&, const double) = 0;
  virtual void stop_sinusoidal_controller(void) = 0;
  virtual void set_moduleServo_position(unsigned int, double) = 0;
  virtual void set_all_moduleServo_position(const vector<double>&) = 0;
  virtual double get_moduleServo_position(unsigned int) = 0;
  virtual bool get_all_moduleServo_position(vector<ServoFeedback*>&) = 0;
  virtual unsigned long get_elapsed_evaluation_time(void) = 0;
  virtual unsigned long get_previous_read_evaluation_time(void) = 0;
  virtual double calculate_distance_travelled_euclidean(void) = 0;
  virtual void measure_cumulative_distance(void) = 0;
  virtual double get_robot_X(void) = 0;
  virtual double get_robot_Y(void) = 0;
  virtual void step(const std::string&) = 0;

  //unsigned long counter;

protected:
  RobotEnvironment robot_environment;
  RobotPriority robot_priority;
  RobotType robot_type;
  EvaluationMethod evaluation_method;

  std::vector<double> previous_control_signal;
  unsigned int number_of_modules;
  unsigned long initial_evaluation_time;
  unsigned long previous_read_evaluation_time;
  unsigned long elapsed_evaluation_time; //--microseconds
  double distance_travelled;

  bool receive_broadcast;
  bool broadcast_thread;

  bool processing_flag;
};

#endif

