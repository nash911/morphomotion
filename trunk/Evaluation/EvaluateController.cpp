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
//#include <thread>

#include "Robot.h"
#include "SimulationOpenRave.h"
#include "Y1ModularRobot.h"
#include "Controller.h"
#include "HybridController.h"
#include "SineController.h"
#include "SimpleController.h"
#include "InverseSineController.h"
#include "FileHandler.h"
#include "OscillationAnalyzer_OutputSignal.h"

#define BAUD_RATE 115200

#define ROBOT_PRIMARY_OPENRAVE
//#define ROBOT_PRIMARY_Y1

//#define ROBOT_SECONDARY

#define EVALUATION_PERIOD 30

int main(int argc, char* argv[])
{
  bool evaluate_best_individual = false;

  char* gene_file = NULL;
  char* fitness_file = NULL;

  Robot *robot_primary = NULL;
  Robot *robot_secondary = NULL;

  SimulationOpenRave simuOR_robot;
  Y1ModularRobot y1_robot;

#ifdef ROBOT_PRIMARY_OPENRAVE
  robot_primary = &simuOR_robot;
#elif defined(ROBOT_PRIMARY_Y1)
  y1_robot.set_serial_port(argv[1], BAUD_RATE);
  robot_primary = &y1_robot;
#else
  std::cerr << "MorphoMotion Error: EvaluateController." << std::endl
            << "int main(int, char*) method." << std::endl
            << "Robot Environment needs to be defined!. " << std::endl;
  exit(1);
#endif

#ifdef ROBOT_SECONDARY
  if(robot_primary->get_robot_environment() == "SimulationOpenRave")
  {
    y1_robot.set_serial_port(argv[1], BAUD_RATE);
    robot_secondary = &y1_robot;
  }
  else if(robot_primary->get_robot_environment() == "Y1")
  {
    robot_secondary = &simuOR_robot;
  }
#endif

  // Multilayer perceptron object
  Flood::MultilayerPerceptron mlp(0,0,0);
  mlp.set_independent_parameters_number(0);

  Controller *controller = NULL;

  std::string controller_type;
  FileHandler geneFile;
  controller_type = geneFile.get_controller_type(argv[2]);

  if(controller_type == "Hybrid_Controller")
  {
    controller = new HybridController(&mlp, robot_primary);
  }
  else if(controller_type == "Sine_Controller")
  {
    controller = new SineController(&mlp, robot_primary);
  }
  else if(controller_type == "Simple_Controller")
  {
    controller = new SimpleController(&mlp, robot_primary);
  }
  else if(controller_type == "InverseSine_Controller")
  {
    controller = new InverseSineController(&mlp, robot_primary);
  }
  else
  {
    std::cerr << "Morphomotion Error: EvaluateController." << std::endl
              << "int main(int, char*) method."
              << "Unknown controller type: " << controller_type << std::endl;
    exit(1);
  }


  // Elite population gene
  Flood::Matrix<double> population;
  std::vector<std::string> generation_index;

  std::vector<double> elite_fitness;

  if(argc < 3)
  {
    std::cerr << "MorphoMotion Error: EvaluateController." << std::endl
              << "int main(int, char*) method." << std::endl
              << "Insufficient parameters. " << std::endl;
    exit(1);
  }
  else if(argc == 3)
  {
    gene_file = argv[2];

    if(robot_primary->get_robot_environment() == "SimulationOpenRave" || robot_secondary)
    {
      FileHandler geneFileHandler(gene_file, robot_primary, &simuOR_robot, controller, &mlp, &population, &generation_index);
    }
    else
    {
      FileHandler geneFileHandler(gene_file, robot_primary, NULL, controller, &mlp, &population, &generation_index);
    }
  }
  else if(argc == 4)
  {
    evaluate_best_individual = true;

    gene_file = argv[2];

    if(argv[3][0] == '-' && argv[3][1] == 'b')
    {
      std::string fitness_file_string;
      fitness_file_string = argv[2];
      fitness_file_string.erase(fitness_file_string.end()-62, fitness_file_string.end()-32);
      fitness_file_string.erase(fitness_file_string.end()-20, fitness_file_string.end());

      fitness_file_string.insert(fitness_file_string.end()-12, 'F');
      fitness_file_string.insert(fitness_file_string.end()-12, 'i');
      fitness_file_string.insert(fitness_file_string.end()-12, 't');
      fitness_file_string.insert(fitness_file_string.end()-12, 'n');
      fitness_file_string.insert(fitness_file_string.end()-12, 'e');
      fitness_file_string.insert(fitness_file_string.end()-12, 's');
      fitness_file_string.insert(fitness_file_string.end()-12, 's');
      fitness_file_string.insert(fitness_file_string.end()-12, 'G');
      fitness_file_string.insert(fitness_file_string.end()-12, 'r');
      fitness_file_string.insert(fitness_file_string.end()-12, 'a');
      fitness_file_string.insert(fitness_file_string.end()-12, 'p');
      fitness_file_string.insert(fitness_file_string.end()-12, 'h');
      fitness_file_string.insert(fitness_file_string.end()-12, '_');
      fitness_file_string.insert(fitness_file_string.end()-12, 'F');
      fitness_file_string.insert(fitness_file_string.end()-12, 'i');
      fitness_file_string.insert(fitness_file_string.end()-12, 'l');
      fitness_file_string.insert(fitness_file_string.end()-12, 'e');
      fitness_file_string.insert(fitness_file_string.end()-12, 's');
      fitness_file_string.insert(fitness_file_string.end()-12, '/');

      fitness_file_string.insert(fitness_file_string.end(), 'F');
      fitness_file_string.insert(fitness_file_string.end(), 'i');
      fitness_file_string.insert(fitness_file_string.end(), 't');
      fitness_file_string.insert(fitness_file_string.end(), 'n');
      fitness_file_string.insert(fitness_file_string.end(), 'e');
      fitness_file_string.insert(fitness_file_string.end(), 's');
      fitness_file_string.insert(fitness_file_string.end(), 's');
      fitness_file_string.insert(fitness_file_string.end(), 'G');
      fitness_file_string.insert(fitness_file_string.end(), 'r');
      fitness_file_string.insert(fitness_file_string.end(), 'a');
      fitness_file_string.insert(fitness_file_string.end(), 'p');
      fitness_file_string.insert(fitness_file_string.end(), 'h');
      fitness_file_string.insert(fitness_file_string.end(), '.');
      fitness_file_string.insert(fitness_file_string.end(), 'd');
      fitness_file_string.insert(fitness_file_string.end(), 'a');
      fitness_file_string.insert(fitness_file_string.end(), 't');

      fitness_file = new char[fitness_file_string.size()+1];

      unsigned int i;
      for(i=0; i<fitness_file_string.size(); i++)
      {
          fitness_file[i] = fitness_file_string[i];
      }
      fitness_file[i] = '\0';
    }
    else
    {
      fitness_file = argv[3];
    }

    if(robot_primary->get_robot_environment() == "SimulationOpenRave" || robot_secondary)
    {
      FileHandler gene_fitness_FileHandler(gene_file, fitness_file, robot_primary, &simuOR_robot, controller, &mlp, &population, &generation_index, &elite_fitness);
    }
    else
    {
      FileHandler gene_fitness_FileHandler(gene_file, fitness_file, robot_primary, NULL, controller, &mlp, &population, &generation_index, &elite_fitness);
    }

    if(fitness_file != NULL)
    {
      delete[] fitness_file;
    }
  }

  // Cross Evaluation
  /*simuOR_robot.set_scene_file_name("../models/Minicube-I.env.xml");
  robot_primary->set_number_of_modules(2);*/

  /*simuOR_robot.set_scene_file_name("../models/Minicube-I-Sym.env.xml");
  robot_primary->set_number_of_modules(2);*/

  /*simuOR_robot.set_scene_file_name("../models/Tripod.env.xml");
  robot_primary->set_number_of_modules(3);*/

  /*simuOR_robot.set_scene_file_name("../models/Quadpod.env.xml");
  robot_primary->set_number_of_modules(4);*/

  /*simuOR_robot.set_scene_file_name("../models/Ybot4.env.xml");
  robot_primary->set_number_of_modules(4);*/

  /*simuOR_robot.set_scene_file_name("../models/Lizard/Lizard.env.xml");
  robot_primary->set_number_of_modules(6);*/

  /*simuOR_robot.set_scene_file_name("../models/Lizard.env.xml");
  robot_primary->set_number_of_modules(6);*/

  /*simuOR_robot.set_scene_file_name("../models/Leggy_3DOF/Leggy_3DOF.env.xml");
  robot_primary->set_number_of_modules(15);*/

  if(robot_primary->get_robot_environment() == "SimulationOpenRave" || robot_secondary)
  {
    simuOR_robot.init_simu_env(controller->get_controller_type());
  }

  controller->init_controller();
  population.subtract_row(0);

  robot_primary->set_evaluation_method("Euclidean_Distance_Final");
  //robot_primary->set_evaluation_method("Euclidean_Distance_Cumulative");

#ifdef ROBOT_SECONDARY
  if(robot_primary->get_robot_environment() == "SimulationOpenRave")
  {
    robot_secondary->copy(robot_primary);
    controller->set_robot_secondary(robot_secondary);
  }
  else if(robot_primary->get_robot_environment() == "Y1")
  {
    robot_secondary->copy(robot_primary);
    controller->set_robot_secondary(robot_secondary);
  }
#endif

  //--Hidden Layer Activation Function
  Flood::Vector<std::string> hiddenLayerActivation(mlp.get_hidden_layers_number());
  hiddenLayerActivation[0] = "HyperbolicTangent";

  //--Output Layer Activation Function
  mlp.set_output_layer_activation_function("HyperbolicTangent");

  controller->set_evaluation_period(EVALUATION_PERIOD);

  Flood::Vector<double> individual(mlp.get_parameters_number());
  unsigned int population_size = population.get_rows_number();

  if(evaluate_best_individual)
  {
    unsigned int best_individual_fitness_index = 0;
    for(unsigned int i=1; i<elite_fitness.size();i++)
    {
      if(elite_fitness[i] >= elite_fitness[best_individual_fitness_index])
      {
        best_individual_fitness_index = i;
      }
    }

    best_individual_fitness_index++;
    bool best_fitness_gene_found = false;
    do
    {
      best_individual_fitness_index--;
      std::stringstream generation_no;
      generation_no << "Generation_" << best_individual_fitness_index+1 << ":";
      for(unsigned int i=0; i<=population_size-1; i++)
      {
        if(generation_no.str().compare(generation_index[i]) == 0)
        {
          individual = population.get_row(i);
          std::cout << std::endl << "Best_Fitness: Generation " << best_individual_fitness_index+1 << std::endl << individual << std::endl << std::endl;
          best_fitness_gene_found = true;
          break;
        }
      }
    }while(!best_fitness_gene_found);

    unsigned int n = 1;
    while(n)
    {
      robot_primary->reset_robot();

      if(robot_secondary)
      {
        robot_secondary->reset_robot();
      }

      mlp.set_parameters(individual);

      OscillationAnalyzer_OutputSignal oscAnlz(robot_primary);

      controller->set_oscillation_analyzer(&oscAnlz);
      oscAnlz.set_record_servo(true);
      oscAnlz.set_record_ref(true);
      oscAnlz.set_record_amplitude(true);
      oscAnlz.set_record_offset(true);
      oscAnlz.set_record_frequency(true);
      oscAnlz.set_record_phase(true);
      oscAnlz.set_record_trajectory(true);

      std::stringstream SS;

      //controller->run_Controller("evaluation", SS, best_individual_fitness_index+1);
      controller->start_Controller("evaluation", SS, best_individual_fitness_index+1);

      std::cout << "    (" << best_individual_fitness_index+1 << ") " << "Simulated Robot: Distance travelled = " << robot_primary->get_distance_travelled() << std::endl;

      if(robot_secondary)
      {
        //std::cout << "    (" << best_individual_fitness_index+1 << ") " << "Simulated Robot: Distance travelled = " << robot_secondary->get_distance_travelled() << std::endl;
      }

      std::cout << "  Select individual to be evaluated number:  " << std::endl;

      //std::cin >> n;
    }
  }
  else
  {
    for(unsigned int i=population_size-1; i>=0; i--)
    //for(unsigned int i=0; i<=population_size-1; i++)
    {
      robot_primary->reset_robot();

      if(robot_secondary)
      {
        robot_secondary->reset_robot();
      }

      individual = population.get_row(i);

      //-------------------------To Be Removed-----------------------------//
      //--InverseSine Controller
      // [1] 53.2143   [2] 458005   [3] 10.5565   [4] 49.9843   [5] 0.935509 -- Minibot Crashed
      // 38.9258 284639 79.446 4.22644 0.553617  -- Y-bot Evolved
      //[1] 44.6612  [2] 67520.8  [3] 89.4786  [4] 74.9153  [5] 1.32266 -- Lizard
      /*individual[0] = 44.6612;
      individual[1] = 67520;
      individual[2] = 89.4786;
      individual[3] = 74.9153;
      individual[4] = 1.32266;*/
      //-------------------------To Be Removed-----------------------------//

      //-------------------------To Be Removed-----------------------------//
      //--Amplitude
      /*individual[0] = 0;
      individual[1] = 0;
      individual[2] = 0;
      individual[3] = 0;

      //--Offset
      individual[4] = 0;
      individual[5] = 0;
      individual[6] = 0;
      individual[7] = 0;

      //--Phase
      individual[8] = 0;
      individual[9] = 0;
      individual[10] = 0;
      individual[11] = 0;

      //--Frequency
      individual[12] = 0.9999;*/
      //-------------------------To Be Removed-----------------------------//

      //-------------------------To Be Removed-----------------------------//
      //--Amplitude
      /*individual[0] = 60;
      individual[1] = 60;

      //--Offset
      individual[2] = 0;
      individual[3] = 0;

      //--Phase
      individual[4] = 0;
      individual[5] = 90;

      //--Frequency
      individual[6] = 0.99;*/
      //-------------------------To Be Removed-----------------------------//

      mlp.set_parameters(individual);

      std::cout << std::endl << individual << std::endl << std::endl; // TODO: Debugger to be removed.

      OscillationAnalyzer_OutputSignal oscAnlz(robot_primary);

      controller->set_oscillation_analyzer(&oscAnlz);
      oscAnlz.set_record_servo(true);
      oscAnlz.set_record_ref(true);
      oscAnlz.set_record_amplitude(true);
      oscAnlz.set_record_offset(true);
      oscAnlz.set_record_frequency(true);
      oscAnlz.set_record_phase(true);
      oscAnlz.set_record_trajectory(true);

      std::stringstream SS;

      //controller->run_Controller("evaluation", SS, i);
      controller->start_Controller("evaluation", SS, i);

      robot_primary->reset_modules();
      std::cout << "    (" << i+1 << ") " << robot_primary->get_robot_environment() << ": Distance travelled = " << robot_primary->get_distance_travelled() << std::endl;

      if(robot_secondary)
      {
        //std::cout << "    (" << i+1 << ") " << robot_secondary->get_robot_environment() << ": Distance travelled = " << robot_secondary->get_distance_travelled() << std::endl;
      }

      std::cout << "  Population Size: " << population_size << std::endl << "  Select individual to be evaluated number:  " << std::endl;

      unsigned int x;
      std::cin >> x;
    }
  }

  delete controller;
}
