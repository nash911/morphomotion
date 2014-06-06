/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   E V O L U T I O N   C L A S S                                                                              */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#include <iostream>  // TODO: May be not needed. Confirm and remove
#include <math.h>  // TODO: May be not needed. Confirm and remove

#include "Evolution.h"

namespace Flood
{

Evolution::Evolution()
{
  robot = NULL;
  controller = NULL;
}

Evolution::Evolution(MultilayerPerceptron* mlp)
:ObjectiveFunctional(mlp)
{
}

//-- GENERAL CONSTRUCTOR WITH SIMULATION ENVIRONMENT OBJECT
Evolution::Evolution(MultilayerPerceptron* mlp, Robot* robot_pointer, Controller* controller_pointer)
:ObjectiveFunctional(mlp)
{
  robot = robot_pointer;
  controller = controller_pointer;
}


//-- DESTRUCTOR
Evolution::~Evolution(void)
{
}


//-- METHODS

//-- Default objective function for debugging purpose.
double Evolution::calculate_objective(void)
//double Evolution::calculate_objective(int generation, int individual, int evaluation_sample_size)
{
  double evaluation;
  evaluation =  controller->calculate_random_uniform(0.1,5.0);
  return evaluation;
}


/*double Evolution::calculate_objective(int generation, int individual, int evaluation_sample_size)
{
  double evaluation;
  evaluation =  controller->calculate_random_uniform(0.1,5.0);
  std::cout << std::endl << generation << " -- " << individual << ":";
  std::cout << "    (1) " << evaluation << "        ";

  std::cout << "    I_Parameters: ";
  for(unsigned int i=0; i<controller->get_mlp()->get_independent_parameters_number(); i++)
  {
    std::cout << "[" << i+1 << "] " << controller->get_mlp()->get_independent_parameter(i) << "    ";
  }

  return evaluation;
}*/

double Evolution::calculate_objective(int generation, int individual, int evaluation_sample_size)
{
  Vector<double> evaluation(evaluation_sample_size);
  double total_distance = 0;
  double mean_distance = 0;
  double mean_speed = 0;
  bool previous = false;
  bool invalid_gene = false;

  evaluation.initialize(0.0);

  std::cout << std::endl << generation << " -- " << individual << ":";

  std::cout << "  I_Para: ";
  for(unsigned int n=0; n<controller->get_mlp()->get_independent_parameters_number(); n++)
  {
    std::cout << "[" << n+1 << "] " << controller->get_mlp()->get_independent_parameter(n) << "  ";
  }
  std::cout << "  Evaluation: ";

  for(int i=0; i<evaluation_sample_size; i++)
  {

    std::string result;

    do
    {
        std::stringstream SS;

        //-- Initialise the robot with 0 degrees to the motor and move it to the initial position.
        robot->reset_robot();

        //-- Run controller.
        controller->start_Controller("evolution", SS, generation);

        SS >> result;
    }while(result == "REDO");

    if(result == "CANCEL")
    {
        std::cout << "  Aborted  " << std::endl;
        evaluation[i] = 0;
    }
    else if(result == "PREVIOUS")
    {
        std::cout << std::endl << "                     PREVIOUS...  " << std::endl;
        previous = true;
        break;
    }
    else if(result == "INVALID_AMP+OFF")
    {
        std::cout << std::endl << "  INVALID_AMP+OFF" << std::endl;
        invalid_gene = true;
        break;
    }
    else
    {
        robot->reset_modules();

        //-- Calculate distance travelled by the robot.
        evaluation[i] = robot->get_distance_travelled();
    }

    std::cout << "  (" << i+1 << ") " << evaluation[i];
  }

  if(previous)
  {
    mean_speed = -12.34;
  }
  else if(invalid_gene)
  {
    mean_speed = -22.44;
  }
  else
  {
    for(int i=0; i<evaluation_sample_size; i++)
    {
      total_distance = total_distance + evaluation[i];
    }

    mean_distance = total_distance/evaluation_sample_size;
    mean_speed = (mean_distance/controller->get_evaluation_period())*100;  //-- Mean speed calculated as cms/second.

    //--Adding a small random noise to avoid fitness value of zero.
    mean_speed = mean_speed + controller->calculate_random_uniform(0.001,0.01);
  }

  return(mean_speed);
}

}

