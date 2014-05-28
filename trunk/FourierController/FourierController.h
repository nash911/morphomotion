/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   F O U R I E R   C O N T R O L L E R   C L A S S   H E A D E R                                              */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/


#ifndef FOURIERCONTROLLER_H
#define FOURIERCONTROLLER_H

#include "Controller.h"

class FourierController: public Controller
{

public:

  // DEFAULT CONSTRUCTOR
  FourierController(void);

  // CONSTRUCTOR WITH MLP and ROBOT OBJECT
  FourierController(Flood::MultilayerPerceptron*, Robot*);

  // CONSTRUCTOR WITH MLP and PRIMARY and SECONDARY ROBOT OBJECTS
  FourierController(Flood::MultilayerPerceptron*, Robot*, Robot*);

  // METHODS
  void init_local_variables(Flood::Vector<double>&);
  void estimate_fourier_control_normalizing_factor();
  void actuate_with_fourier_controller(const unsigned int, const double, Flood::Vector<double>&);


  //-- VIRTUAL FUNCTIONS
  virtual void set_default(void);
  virtual void init_controller(const double);
  virtual void start_Controller(const std::string&, std::stringstream&, int);
  virtual void run_Controller(const std::string&, std::stringstream&, int);

private:
  vector<double> normalizing_factor;

};

#endif
