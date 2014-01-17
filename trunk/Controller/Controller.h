/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   C O N T R O L L E R   C L A S S   H E A D E R                                                              */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/


#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <math.h>
#include <unistd.h> // TODO: Not sure if this is needed.

#include <cmath>
#include <vector>


#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>

#include "Robot.h"
#include "MultilayerPerceptron.h"
#include "ServoFeedback.h"
#include "GraphFile.h"
#include "OscillationAnalyzer_OutputSignal.h"

#define _USE_MATH_DEFINES

#define Q 81
#define q 113
#define SPACE 32

#define ACTIVITY_LOG
//#define DEBUGGER
//#define EVERY_STEP_DEBUGGER

//#define CUMULATIVE_DISTANCE
#define CUMULATIVE_DISTANCE_STEPSIZE 10

//#define NOISE
#define NOISE_MEAN 0
#define NOISE_SD 5

using namespace std;

class Controller
{

public:

  enum StartAngleType{Zero, Random, RandomEqual, Predefined, RunTime};
  enum ControllerType{Neural_Controller, Naive_Controller, Simple_Controller, Hybrid_Controller, Semi_Hybrid_Controller, Sine_Controller};

  // DEFAULT CONSTRUCTOR
  Controller(void);

  // CONSTRUCTOR WITH MLP and ROBOT OBJECT
  Controller(Flood::MultilayerPerceptron*, Robot*);

  // CONSTRUCTOR WITH MLP and PRIMARY and SECONDARY ROBOT OBJECTS
  Controller(Flood::MultilayerPerceptron*, Robot*, Robot*);


  // DESTRUCTOR
  virtual ~Controller(void);

  // METHODS

  void reset_controller(void);
  void set_oscillation_analyzer(OscillationAnalyzer_OutputSignal*);


  //void actuate_with_sine_controller(const unsigned int, const double, Flood::Vector<double>&);
  //void actuate_with_neural_controller(const unsigned int, Flood::Vector<double>&);
  //void actuate_with_simple_controller(const unsigned int, Flood::Vector<double>&);
  //void actuate_with_hybrid_controller(const unsigned int, Flood::Vector<double>&);
  void actuate_module(const unsigned int, double);
  void actuate_all_modules(const Flood::Vector<double>&);
  bool read_servo_positions_with_time(void); // TODO: This should be implemented as a seperate thread.
  double calculate_servo_delta(const unsigned int, double);
  double calculate_servo_derivative_time(const unsigned int, vector<vector<ServoFeedback*> >&);

  void set_controller_type(const std::string&);
  std::string get_controller_type(void);
  void set_robot_secondary(Robot*);
  Robot* get_robot_secondary(void);
  void set_evaluation_period(unsigned int);
  unsigned int get_evaluation_period(void);
  void set_servo_max(double);
  double get_servo_max(void);
  void set_servo_min(double);
  double get_servo_min(void);
  void set_start_angle_type(const std::string&);
  std::string get_start_angle_type(void);
  void set_predef_start_angle_values(Flood::Vector<double>);

  void set_servo_delta_threshold(double);
  double get_servo_delta_threshold(void);
  void set_servo_derivative_threshold(double);
  double get_servo_derivative_threshold(void);
  void set_servo_derivative_epsilon(unsigned long);
  unsigned long get_servo_derivative_epsilon(void);

  void set_oscillator_amplitude(double);
  double get_oscillator_amplitude(void);
  void set_oscillator_offset(double);
  double get_oscillator_offset(void);

  void load_sine_control_parameters(void);
  void set_sine_amplitude(const double, const unsigned int, const unsigned int);
  void set_sine_offset(const double, const unsigned int, const unsigned int);
  void set_sine_phase(const double, const unsigned int, const unsigned int);
  void set_sine_frequency(const double, const unsigned int);

  double get_sine_amplitude(const unsigned int);
  double get_sine_offset(const unsigned int);
  double get_sine_phase(const unsigned int);
  double get_sine_frequency(void);

  double calculate_random_uniform(double, double);
  double calculate_random_normal(double, double);
  double scale_to_range(double, double, double, double, double);

  void changemode(int);
  int kbhit (void);
  /*void record_servo_graph_simulated(int);
  void record_servo_graph_real(int);*/


  //-- VIRTUAL FUNCTIONS
  virtual void set_default(void);
  virtual void init_controller(void);
  /*virtual void init_local_variables(Flood::Vector<double>&,
                            Flood::Vector<double>&,
                            Flood::Vector<bool>&);*/
  virtual bool run_Controller(const std::string&, std::stringstream&, int, int, int) = 0;
  //virtual bool run_Controller(const std::string&, std::stringstream&, int, int, int){}


protected:
  Flood::MultilayerPerceptron *mlp;
  Robot *robot_primary;
  Robot *robot_secondary;
  OscillationAnalyzer_OutputSignal *oscAnlz;
  ControllerType controller_type;
  double servo_max;
  double servo_min;
  unsigned int number_of_modules;
  unsigned int evaluation_period;
  vector<ServoFeedback*> servo_feedback;
  StartAngleType start_angle_type;
  Flood::Vector <double> predef_start_angle;
  double servo_delta_threshold;
  double servo_derivative_threshold;
  unsigned long servo_derivative_epsilon;  // BUG FIX: Was previously --> unsigned servo_derivative_epsilon;
  double oscillator_amplitude;
  double oscillator_offset;

  vector<double> sine_amplitude;
  vector<double> sine_offset;
  vector<double> sine_phase;
  double sine_frequency;

  //double Pi;
};

#endif
