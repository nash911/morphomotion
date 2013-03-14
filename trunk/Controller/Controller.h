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

#include "Robot.h"
#include "MultilayerPerceptron.h"
#include "ServoFeedback.h"
#include "GraphFile.h"
#include "OscillationAnalyzer_OutputSignal.h"

#include <unistd.h> // TODO: Not sure if this is needed.


#define ACTIVITY_LOG
//#define DEBUGGER
//#define EVERY_STEP_DEBUGGER

//#define CUMULATIVE_DISTANCE
#define CUMULATIVE_DISTANCE_STEPSIZE 10

//#define RECORD_SERVO
//#define RECORD_FREQUENCY
//#define RECORD_PHASE
//#define RECORD_TRAJECTORY
#define RESOLUTION 100

//#define RECORD_REF_PHI
//#define RECORD_REF
//#define RECORD_PHI

#define ACTUATE_SIMULATED_MODULES
#define SIMULATION_RELTIME

#define ACTUATE_REAL_MODULES
#define READ_SERVO_REAL_MODULES

#ifndef ACTUATE_REAL_MODULES
  #define SIMULATION_RELTIME
#endif

//#define NOISE
#define NOISE_MEAN 0
#define NOISE_SD 5


using namespace std;

class Controller
{

public:

  enum StartAngleType{Zero, Random, RandomEqual, Predefined, RunTime};

  //enum RobotType{Cube2_Sym_ServoFeedBack, Star3_ServoFeedBack, Star3_II_ServoFeedBack, CubeN_ServoFeedBack, CubePYP_ServoFeedBack, CubePYPYP_ServoFeedBack, Ybot4_ServoFeedBack, Ybot5_ServoFeedBack, QuadP_ServoFeedBack, DashHound_ServoFeedBack, Lizard_ServoFeedBack}; // TODO: To be removed

  enum ControllerType{Sinusoidal_Controller, Neural_Controller, Naive_Controller, Simple_Controller, Hybrid_Controller, Semi_Hybrid_Controller};

  // DEFAULT CONSTRUCTOR
  Controller(void);

  // CONSTRUCTOR WITH MLP and ROBOT OBJECTS
  Controller(Flood::MultilayerPerceptron*, Robot*);


  // DESTRUCTOR
  virtual ~Controller(void);

  // METHODS

  void reset_controller(void);
  void set_default(void);
  void set_oscillation_analyzer(OscillationAnalyzer_OutputSignal*);
  void init_controller(void);
  void init_local_variables(Flood::Vector<double>&,
                            Flood::Vector<double>&,
                            Flood::Vector<bool>&);

  bool run_Controller(const std::string&, int, int, int);
  void actuate_with_neural_controller(int, Flood::Vector<double>&);
  void actuate_with_simple_controller(int, Flood::Vector<double>&);
  void actuate_with_hybrid_controller(int, Flood::Vector<double>&);
  void actuate_module(int, double);
  void read_servo_positions_with_time(void); // TODO: This should be implemented as a seperate thread.
  double calculate_servo_delta(int, double);
  double calculate_servo_derivative_time(unsigned int, vector<vector<ServoFeedback*> >&);

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

  void load_sinusoidal_control_parameters(void);
  void set_sinusoidal_amplitude(const double, const unsigned int, const unsigned int);
  void set_sinusoidal_offset(const double, const unsigned int, const unsigned int);
  void set_sinusoidal_phase(const double, const unsigned int, const unsigned int);
  void set_sinusoidal_frequency(const double, const unsigned int);

  double calculate_random_uniform(double, double);
  double calculate_random_normal(double, double);
  double scale_to_range(double, double, double, double, double);
  /*void record_servo_graph_simulated(int);
  void record_servo_graph_real(int);*/


private:
  Flood::MultilayerPerceptron *mlp;
  Robot *robot_primary;
  Robot *robot_secondary;
  OscillationAnalyzer_OutputSignal *oscAnlz;

  //RobotType robot_type; // TODO: To be removed
  ControllerType controller_type;
  double servo_max;
  double servo_min;
  int number_of_modules;
  unsigned int evaluation_period;
  vector<ServoFeedback*> servo_feedback;
  StartAngleType start_angle_type;
  Flood::Vector <double> predef_start_angle;
  double servo_delta_threshold;
  double servo_derivative_threshold;
  unsigned servo_derivative_epsilon;
  double oscillator_amplitude;
  double oscillator_offset;

  vector<double> sinusoidal_amplitude;
  vector<double> sinusoidal_offset;
  vector<double> sinusoidal_phase;
  double sinusoidal_frequency;

  // TODO: To be removed
  /*GraphFile *real_servo_position_graph_file;
  GraphFile *frequency_graph_file;
  GraphFile *phase_graph_file_180;
  GraphFile *phase_graph_file_360;
  GraphFile *trajectory_file;*/
};

#endif
