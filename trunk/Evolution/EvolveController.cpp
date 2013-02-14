/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   E V O L V E   C O N T R O L L E R   A P P L I C A T I O N                                                  */
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
#include <time.h>
#include <stdexcept>
#include <cstdlib>

#include "EvolutionaryAlgorithm.h"

#include "Evolution.h"
#include "Robot.h"
#include "SimulationOpenRave.h"
#include "Y1ModularRobot.h"
#include "Controller.h"
#include "FileHandler.h"

#define EVALUATION_SAMPLE_SIZE 3

#define POPULATION_SIZE 100
#define GENERATIONS 50

#define CROSSOVER_RATIO 0.5
#define ELITISM_RATIO 0.25

//#define CUMUALATIVE_DISTANCE

#define GEN_0_MIN_EVAL_VAL 0.8
//#define GEN_0_PREEVAL

#define GEN_0_POPULATION_SIZE 300
//#define GEN_0_AUGMENTED_POPULATION

#define FITNESS_GRAPH_FILE
#define ELITE_POPULATION_FILE



//std::string note("Evaluation Period -- 100 Seconds ");
std::string note("No Notes");

int main(int argc, char* argv[])
{
  srand( (unsigned)time( NULL ) );
  char* parameter_file;
  bool gen0_preeval=false;
  bool gen0_aug_pop=false;

  SimulationOpenRave simuOR_robot;

  // Multilayer perceptron object
  Flood::MultilayerPerceptron mlp(0,0,0);
  mlp.set_independent_parameters_number(0);

  Controller controller(&mlp, &simuOR_robot);

   if(argc == 1) // TODO: Need to change this to include all commande line parameter possibilities.
   {
      parameter_file = "/home/nash/Dropbox/PhD/modularRobotics/morphoMotion/Evolution_Files/Ybot4_ServoFeedBack/Hybrid_Controller/ybot4_ServoFeedBack.dat";
   }
   else
   {
      parameter_file = argv[1];
   }

  FileHandler parametersFileHandler(parameter_file, &simuOR_robot, &simuOR_robot, &controller, &mlp);

  // Initialise objects after parameters loaded from the parameter file.
  Robot *robot = &simuOR_robot;
  simuOR_robot.init_simu_env(controller.get_controller_type());
  controller.init_controller();

#ifdef CUMUALATIVE_DISTANCE
  robot->set_evaluation_method("Euclidean_Distance_Cumulative");
#else
  robot->set_evaluation_method("Euclidean_Distance_Final");
#endif

  // Hidden Layer Activation Function
  Flood::Vector<std::string> hiddenLayerActivation(mlp.get_hidden_layers_number());
  hiddenLayerActivation[0] = "HyperbolicTangent";

  // Output Layer Activation Function
  mlp.set_output_layer_activation_function("HyperbolicTangent");

  // Cube Revolution object
  Flood::Evolution evolve(&mlp, &simuOR_robot, &controller);

  // Evolutionary algorithm object
  Flood::EvolutionaryAlgorithm ea(&evolve);

  ea.set_population_size(POPULATION_SIZE);
  ea.set_maximum_generations_number(GENERATIONS);
  ea.initialize_population_uniform_independent_parameters(mlp.get_independent_parameters_minimum(), mlp.get_independent_parameters_maximum());
  ea.set_display_period(1);

  ea.set_crossover_percentage(CROSSOVER_RATIO);
  ea.set_elitism_percentage(ELITISM_RATIO);

  ea.set_fitness_assignment_method("MyLinearRanking");
  ea.set_selection_method("MyRouletteWheel");
  ea.set_recombination_method("MyIntermediateWithElitismAndES");
  ea.set_mutation_method("MyNormalWithES");
  ea.set_evaluation_sample_size(EVALUATION_SAMPLE_SIZE);
  ea.set_gen_0_min_evaluation_value(GEN_0_MIN_EVAL_VAL);
  ea.set_gen_0_population_size(GEN_0_POPULATION_SIZE);
#ifdef GEN_0_PREEVAL
  ea.populate_generation_0();
  gen0_preeval=true;
#endif

#ifdef GEN_0_AUGMENTED_POPULATION
  ea.populate_generation_0_augmented_polulation();
  gen0_aug_pop=true;
#endif

#ifdef FITNESS_GRAPH_FILE
  GraphFile fitness_graph_file(robot->get_robot_type(), controller.get_controller_type());
  ea.set_fitness_graph_history(true);
  ea.set_fitness_graph_file(&fitness_graph_file);
#endif

#ifdef ELITE_POPULATION_FILE
  ea.set_elite_population_history(true);

  // With Flood::EvolutionaryAlgorithm*
  FileHandler elite_population_file(note,
                               gen0_preeval,
                               gen0_aug_pop,
                               &ea,
                               &simuOR_robot,
                               &simuOR_robot,
                               &controller,
                               &mlp);

  ea.set_elite_population_file(&elite_population_file);
#endif

  ea.train();

  Flood::Vector<double> minimal_argument = mlp.get_independent_parameters();

  std::cout << "Minimal argument:" << std::endl << minimal_argument << std::endl;

   return(0);
}
