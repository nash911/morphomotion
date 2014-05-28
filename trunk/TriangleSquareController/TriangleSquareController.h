/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   T R I A N G L E S Q U A R E   C O N T R O L L E R   C L A S S   H E A D E R                                */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/


#ifndef TRIANGLESQUARECONTROLLER_H
#define TRIANGLESQUARECONTROLLER_H

#include "Controller.h"

class TriangleSquareController: public Controller
{

public:

  // DEFAULT CONSTRUCTOR
  TriangleSquareController(void);

  // CONSTRUCTOR WITH MLP and ROBOT OBJECT
  TriangleSquareController(Flood::MultilayerPerceptron*, Robot*);

  // CONSTRUCTOR WITH MLP and PRIMARY and SECONDARY ROBOT OBJECTS
  TriangleSquareController(Flood::MultilayerPerceptron*, Robot*, Robot*);

  // METHODS
  void init_local_variables(Flood::Vector<double>&);
  double sawtooth(const double, const unsigned int);
  double triangle(const double, const unsigned int);
  double triangle_asymmetric(const double, const double, const unsigned int);
  void actuate_with_trianglesquare_controller(const unsigned int, const double, Flood::Vector<double>&);


  //-- VIRTUAL FUNCTIONS
  virtual void set_default(void);
  virtual void init_controller(const double);
  virtual void start_Controller(const std::string&, std::stringstream&, int);
  virtual void run_Controller(const std::string&, std::stringstream&, int);
};

#endif
