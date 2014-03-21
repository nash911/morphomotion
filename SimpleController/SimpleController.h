/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   S I M P L E   C O N T R O L L E R   C L A S S   H E A D E R                                                */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/


#ifndef SIMPLECONTROLLER_H
#define SIMPLECONTROLLER_H

#include "Controller.h"

class SimpleController: public Controller
{

public:

  // DEFAULT CONSTRUCTOR
  SimpleController(void);

  // CONSTRUCTOR WITH MLP and ROBOT OBJECT
  SimpleController(Flood::MultilayerPerceptron*, Robot*);

  // CONSTRUCTOR WITH MLP and PRIMARY and SECONDARY ROBOT OBJECTS
  SimpleController(Flood::MultilayerPerceptron*, Robot*, Robot*);

  // METHODS
  void init_local_variables(Flood::Vector<double>&, Flood::Vector<double>&, Flood::Vector<bool>&);
  void actuate_with_simple_controller(const unsigned int, Flood::Vector<double>&);


  //-- VIRTUAL FUNCTIONS
  virtual void set_default(void);
  virtual void init_controller(void);
  virtual void start_Controller(const std::string&, std::stringstream&, int);
  virtual void run_Controller(const std::string&, std::stringstream&, int);
};

#endif
