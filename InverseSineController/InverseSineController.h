/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   I N V E R S E S I N E   C O N T R O L L E R   C L A S S   H E A D E R                                      */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/


#ifndef INVERSESINECONTROLLER_H
#define INVERSESINECONTROLLER_H

#include "Controller.h"

class InverseSineController: public Controller
{

public:

  // DEFAULT CONSTRUCTOR
  InverseSineController(void);

  // CONSTRUCTOR WITH MLP and ROBOT OBJECT
  InverseSineController(Flood::MultilayerPerceptron*, Robot*);

  // CONSTRUCTOR WITH MLP and PRIMARY and SECONDARY ROBOT OBJECTS
  InverseSineController(Flood::MultilayerPerceptron*, Robot*, Robot*);

  // METHODS
  void init_local_variables(Flood::Vector<double>&,
                            Flood::Vector<double>&,
                            vector<bool>&);
  void actuate_with_inverse_sine_controller(const unsigned int, const double, Flood::Vector<double>&);
  void update_tPrime_and_Yi(unsigned int);
  double calculate_actual_diff(double, double);
  double calculate_tPrime(double, double);
  double inverse_sinewave_function(double, double);
  double getCurrentVelocityDirection(double);


  //-- VIRTUAL FUNCTIONS
  virtual void set_default(void);
  virtual void init_controller(void);
  virtual void start_Controller(const std::string&, std::stringstream&, int);
  virtual void run_Controller(const std::string&, std::stringstream&, int);

private:
  vector<double> Yi;
  vector<double> tPrime;
  vector<double> iteration_start_time;

};

#endif
