/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   E V A L U A T E   C O N T R O L L E R                                                                      */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

// System includes
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include "Robot.h"
#include "SimulationOpenRave.h"
#include "Y1ModularRobot.h"
#include "Controller.h"
#include "FileHandler.h"
#include "OscillationAnalyzer_OutputSignal.h"

#define BAUD_RATE 115200


int main(int argc, char* argv[])
{
  char* gene_file;

  SimulationOpenRave simuOR_robot;
  Robot *robot_primary = &simuOR_robot;
  Robot *robot_secondary = NULL;

  // Multilayer perceptron object
  Flood::MultilayerPerceptron mlp(0,0,0);
  mlp.set_independent_parameters_number(0);

  Controller controller(&mlp, &simuOR_robot);

  // Elite population gene
  Flood::Matrix<double> population;

  if(argc == 1) // TODO: Need to change this to include all commande line parameter possibilities.
  {
    gene_file = "/home/nash/Dropbox/PhD/modularRobotics/morphoMotion/Evolution_Files/Ybot4_ServoFeedBack/Hybrid_Controller/Gene_Files/SimulationOpenRave_11_08_23_03_elite_population.gne";  // TODO: Has to be changed in the future.
  }
  else
  {
    gene_file = argv[2];
  }

  FileHandler geneFileHandler(gene_file, &simuOR_robot, &simuOR_robot, &controller, &mlp, &population);

  // Cross Evaluation
  /*simuOR_robot.set_scene_file_name("../models/Minicube-I.env.xml");
  robot_primary->set_number_of_modules(2);*/

  /*simuOR_robot.set_scene_file_name("../models/Tripod.env.xml");
  robot_primary->set_number_of_modules(3);*/

  /*simuOR_robot.set_scene_file_name("../models/Quadpod.env.xml");
  robot_primary->set_number_of_modules(4);*/

  /*simuOR_robot.set_scene_file_name("../models/Ybot4.env.xml");
  robot_primary->set_number_of_modules(4);*/

  /*simuOR_robot.set_scene_file_name("../models/Lizard/Lizard.env.xml");
  robot_primary->set_number_of_modules(6);*/

  /*simuOR_robot.set_scene_file_name("../models/Leggy_3DOF/Leggy_3DOF.env.xml");
  robot_primary->set_number_of_modules(15);*/

  //controller.set_controller_type("Sinusoidal_Controller");

  simuOR_robot.init_simu_env(controller.get_controller_type());
  controller.init_controller();
  population.subtract_row(0);

  //Y1ModularRobot y1_robot;
  /*Y1ModularRobot y1_robot(robot_primary);
  robot_secondary = &y1_robot;
  y1_robot.set_serial_port(argv[1], BAUD_RATE);
  controller.set_robot_secondary(&y1_robot);*/

  //robot_primary->set_evaluation_method("Euclidean_Distance_Final");  // Debugger;
  robot_primary->set_evaluation_method("Euclidean_Distance_Cumulative");  // Debugger;

  // Hidden Layer Activation Function
  Flood::Vector<std::string> hiddenLayerActivation(mlp.get_hidden_layers_number());
  hiddenLayerActivation[0] = "HyperbolicTangent";

  // Output Layer Activation Function
  mlp.set_output_layer_activation_function("HyperbolicTangent");

  controller.set_evaluation_period(30);

  Flood::Vector<double> individual(mlp.get_parameters_number());
  int population_size = population.get_rows_number();

  int x; // Debugger

  for(int i=population_size-1; i>=0; i--)
  //for(int i=0; i<=population_size-1; i++)
  {
    robot_primary->reset_robot();

    if(robot_secondary)
    {
      robot_secondary->reset_robot();
    }

    // Debugger: To evaluate random solutions
    /*individual[0] = -1.450586;
    individual[1] = -1.171314;
    individual[2] = 1.261777;
    individual[3] = 0.18068;
    individual[4] = -1.473637;
    individual[5] = -1.49753;
    individual[6] = -1.109373;
    individual[7] = -1.701846;
    individual[8] = -1.674622;
    individual[9] = 0.56627;*/

    individual = population.get_row(i);
    mlp.set_parameters(individual);

    OscillationAnalyzer_OutputSignal oscAnlz(robot_primary);
    controller.set_oscillation_analyzer(&oscAnlz);

    oscAnlz.set_record_servo(true);
    oscAnlz.set_record_frequency(true);
    oscAnlz.set_record_phase(true);

    controller.run_Controller("evaluation",1,i,1);

    std::cout << "    (" << i+1 << ") " << "Simulated Robot: Distance travelled = " << robot_primary->get_distance_travelled() << std::endl;

    if(robot_secondary)
    {
      std::cout << "    (" << i+1 << ") " << "Simulated Robot: Distance travelled = " << robot_secondary->get_distance_travelled() << std::endl;
    }

    std::cout << "  Population Size: " << population_size << std::endl << "  Select individual to be evaluated number:  " << std::endl;
    //std::cin >> x;
  }

}
