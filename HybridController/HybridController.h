/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   H Y B R I D   C O N T R O L L E R   C L A S S   H E A D E R                                                */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/


#ifndef HYBRIDCONTROLLER_H
#define HYBRIDCONTROLLER_H

#include "Controller.h"

class HybridController: public Controller
{

public:

  // DEFAULT CONSTRUCTOR
  HybridController(void);

  // CONSTRUCTOR WITH MLP and ROBOT OBJECT
  HybridController(Flood::MultilayerPerceptron*, Robot*);

  // CONSTRUCTOR WITH MLP and PRIMARY and SECONDARY ROBOT OBJECTS
  HybridController(Flood::MultilayerPerceptron*, Robot*, Robot*);

  // METHODS
  void init_local_variables(Flood::Vector<double>&,
                            Flood::Vector<double>&,
                            Flood::Vector<bool>&);
  void actuate_with_hybrid_controller(const unsigned int, Flood::Vector<double>&);


  //-- VIRTUAL FUNCTIONS
  virtual void set_default(void);
  virtual void init_controller(void);
  virtual bool run_Controller(const std::string&, std::stringstream&, int, int, int);
};

#endif
