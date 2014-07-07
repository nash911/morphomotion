/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   M Y F I L E   C L A S S                                                                                    */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#include "FileHandler.h"

FileHandler::FileHandler()
{
}


// With Flood::EvolutionaryAlgorithm*
// CONSTRUCTOR FOR SAVING ELITE GENE POPULATION
/*FileHandler::FileHandler(std::string note,
               bool gen0_preeval,
               bool gen0_aug_pop,
               Flood::EvolutionaryAlgorithm *ea,
               Robot *robot,
               SimulationOpenRave *simuOR,
               Controller *controller,
               Flood::MultilayerPerceptron *mlp)
{
  time_t rawtime;
  struct tm * timeinfo;
  char filename[100];
  stringstream ss;

  time ( &rawtime );
  timeinfo = localtime ( &rawtime );
  strftime (filename, 100, "_%m_%d_%H_%M_elite_population.gne", timeinfo);
  ss << "../Evolution_Files/" << robot->get_robot_environment() << "/" << robot->get_robot_type() << "/" << controller->get_controller_type() << "/Gene_Files/" << robot->get_robot_environment() << filename;

  myFile_name = ss.str();
  cout <<" Opening elite_population data file: " << myFile_name << endl;
  myFile.open(myFile_name.c_str(), fstream::in | fstream::out | fstream::app);

  myFile << "<Morphomotion: Flood + OpenRave + Y1>" << std::endl;
  myFile << std::endl << "<FileType>" << std::endl << "   GeneFile" << std::endl << "</FileType>" << std::endl;

  if(note != "No Notes")
  {
    myFile << std::endl << "<Note>" << std::endl << note << std::endl << "</Note>" << std::endl;
  }

  myFile << std::endl << "<Gene_File>" << std::endl << myFile_name << std::endl << "</Gene_File>" << std::endl;

  char fitness_filename[100];
  std::string fitnessFile;
  strftime (fitness_filename, 100, "_%m_%d_%H_%M_FitnessGraph.dat", timeinfo);
  stringstream fitness_file;
  fitness_file << "../Evolution_Files/" << robot->get_robot_environment() << "/" << robot->get_robot_type() << "/" << controller->get_controller_type() << "/FitnessGraph_Files/" << fitness_filename;
  fitnessFile = fitness_file.str();
  myFile << std::endl << "<Fitness_File>" << std::endl << fitness_file.str() << std::endl << "</Fitness_File>" << std::endl;

  char evol_filename[100];
  std::string evoFile;
  strftime (evol_filename, 100, "_%m_%d_%H_%M_evolution.evo", timeinfo);
  stringstream evol_file;
  evol_file << "../Evolution_Files/" << robot->get_robot_environment() << "/" << robot->get_robot_type() << "/" << controller->get_controller_type() << "/Evolution/" << robot->get_robot_environment() << evol_filename;
  evoFile = evol_file.str();
  myFile << std::endl << "<Evolution_File>" << std::endl << evol_file.str() << std::endl << "</Evolution_File>" << std::endl;

  myFile << std::endl << "<Evolution>" << std::endl;
  myFile << "\t<FitnessAssignmentMethod>" << std::endl << "\t   " << ea->get_fitness_assignment_method_name() << std::endl << "\t</FitnessAssignmentMethod>" << std::endl;
  myFile << std::endl << "\t<SelectionMethod>" << std::endl << "\t   " << ea->get_selection_method_name() << std::endl << "\t</SelectionMethod>" << std::endl;
  myFile << std::endl << "\t<RecombinationMethod>" << std::endl << "\t   " << ea->get_recombination_method_name() << std::endl << "\t</RecombinationMethod>" << std::endl;
  myFile << std::endl << "\t<MutationMethod>" << std::endl << "\t   " << ea->get_mutation_method_name() << std::endl << "\t</MutationMethod>" << std::endl;
  myFile << std::endl << "\t<EvaluationMethod>" << std::endl << "\t   " << robot->get_evaluation_method() << std::endl << "\t</EvaluationMethod>" << std::endl;
  myFile << std::endl << "\t<PopulationSize>" << std::endl << "\t   " << ea->get_population_size() << std::endl << "\t</PopulationSize>" << std::endl;
  myFile << std::endl << "\t<CrossoverPercentage>" << std::endl << "\t   " << ea->get_crossover_percentage() << std::endl << "\t</CrossoverPercentage>" << std::endl;
  myFile << std::endl << "\t<Elitism>" << std::endl << "\t   " << ea->get_elitism() << std::endl << "\t</Elitism>" << std::endl;
  myFile << std::endl << "\t<ElitismPercentage>" << std::endl << "\t   " << ea->get_elitism_percentage() << std::endl << "\t</ElitismPercentage>" << std::endl;
  myFile << std::endl << "\t<SelectivePressure>" << std::endl << "\t   " << ea->get_selective_pressure() << std::endl << "\t</SelectivePressure>" << std::endl;
  myFile << std::endl << "\t<RecombinationSize>" << std::endl << "\t   " << ea->get_recombination_size() << std::endl << "\t</RecombinationSize>" << std::endl;
  myFile << std::endl << "\t<MutationRate>" << std::endl << "\t   " << ea->get_mutation_rate() << std::endl << "\t</MutationRate>" << std::endl;
  myFile << std::endl << "\t<MutationRateForES>" << std::endl << "\t   " << ea->get_mutation_rate_for_ES() << std::endl << "\t</MutationRateForES>" << std::endl;
  myFile << std::endl << "\t<MutationRange>" << std::endl << "\t   " << ea->get_mutation_range() << std::endl << "\t</MutationRange>" << std::endl;
  myFile << std::endl << "\t<MutationRangeForES>" << std::endl << "\t   " << ea->get_mutation_range_for_ES() << std::endl << "\t</MutationRangeForES>" << std::endl;
  myFile << std::endl << "\t<EvaluationSampleSize>" << std::endl << "\t   " << ea->get_evaluation_sample_size() << std::endl << "\t</EvaluationSampleSize>" << std::endl;
  if(gen0_preeval)
  {
    myFile << std::endl << "\t<Gen0_Pre-Evaluation>" << std::endl << "\t   " << "True" << std::endl << "\t</Gen0_Pre-Evaluation>" << std::endl;
  }
  else
  {
    myFile << std::endl << "\t<Gen0_Pre-Evaluation>" << std::endl << "\t   " << "False" << std::endl << "\t</Gen0_Pre-Evaluation>" << std::endl;
  }
  if(gen0_aug_pop)
  {
    myFile << std::endl << "\t<Gen0_AugmentedPopulation>" << std::endl << "\t   " << "True" << std::endl << "\t</Gen0_AugmentedPopulation>" << std::endl;
  }
  else
  {
    myFile << std::endl << "\t<Gen0_AugmentedPopulation>" << std::endl << "\t   " << "False" << std::endl << "\t</Gen0_AugmentedPopulation>" << std::endl;
  }
  myFile << std::endl << "\t<Gen0_MinimumEvaluationValue>" << std::endl << "\t   " << ea->get_gen_0_min_evaluation_value() << std::endl << "\t</Gen0_MinimumEvaluationValue>" << std::endl;
  myFile << std::endl << "\t<Gen0_PopulationSize>" << std::endl << "\t   " << ea->get_gen_0_population_size() << std::endl << "\t</Gen0_PopulationSize>" << std::endl;
  myFile << "</Evolution>" << std::endl;

  myFile << std::endl << "<Robot>" << std::endl;
  myFile << "\t<RobotEnvironment>" << std::endl << "\t   " << robot->get_robot_environment() << std::endl << "\t</RobotEnvironment>" << std::endl;
  myFile << std::endl << "\t<RobotType>" << std::endl << "\t   " << robot->get_robot_type() << std::endl << "\t</RobotType>" << std::endl;
  myFile << std::endl << "\t<NumberOfModules>" << std::endl << "\t   " << robot->get_number_of_modules() << std::endl << "\t</NumberOfModules>" << std::endl;
  myFile << "</Robot>" << std::endl;

  if(robot->get_robot_environment() == "SimulationOpenRave")
  {
    if(simuOR)
    {
      myFile << std::endl << "<SimulationEnvironment>" << std::endl;
      myFile << "\t<SceneFileName>" << std::endl << "\t   " << simuOR->get_scene_file_name() << std::endl << "\t</SceneFileName>" << std::endl;
      myFile << std::endl << "\t<SimResolution>" << std::endl << "\t   " << simuOR->get_simu_resolution_microseconds() << std::endl << "\t</SimResolution>" << std::endl;
      myFile << "</SimulationEnvironment>" << std::endl;
    }
    else
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "FileHandler(std::string, bool, bool, Flood::EvolutionaryAlgorithm*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*)" << std::endl
                << "robot_pointer = NULL " << std::endl;
      exit(1);
    }
  }

  myFile << std::endl << "<Controller>" << std::endl;
  myFile << "\t<ControllerType>" << std::endl << "\t   " << controller->get_controller_type() << std::endl << "\t</ControllerType>" << std::endl;
  if(controller->get_controller_type() == "Fourier_Controller")
  {
    myFile << std::endl << "\t<FrequencyDomainSize>" << std::endl << "\t   " << controller->get_frequency_domain_size() << std::endl << "\t</FrequencyDomainSize>" << std::endl;
  }
  myFile << std::endl << "\t<EvaluationPeriod>" << std::endl << "\t   " << controller->get_evaluation_period() << std::endl << "\t</EvaluationPeriod>" << std::endl;
  myFile << std::endl << "\t<ServoMax>" << std::endl << "\t   " << controller->get_servo_max() << std::endl << "\t</ServoMax>" << std::endl;
  myFile << std::endl << "\t<ServoMin>" << std::endl << "\t   " << controller->get_servo_min() << std::endl << "\t</ServoMin>" << std::endl;
  if(controller->get_controller_type() != "Sine_Controller" &&
     controller->get_controller_type() != "InverseSine_Controller" &&
     controller->get_controller_type() != "Fourier_Controller" &&
     controller->get_controller_type() != "TriangleSquare_Controller")
  {
    myFile << std::endl << "\t<StartAngleType>" << std::endl << "\t   " << controller->get_start_angle_type() << std::endl << "\t</StartAngleType>" << std::endl;
    myFile << std::endl << "\t<ServoDeltaThreshold>" << std::endl << "\t   " << controller->get_servo_delta_threshold() << std::endl << "\t</ServoDeltaThreshold>" << std::endl;
  }
  if(controller->get_controller_type() == "Neural_Controller" || controller->get_controller_type() == "Naive_Controller")
  {
    myFile << std::endl << "\t<ServoDerivativeThreshold>" << std::endl << "\t   " << controller->get_servo_derivative_threshold() << std::endl << "\t</ServoDerivativeThreshold>" << std::endl;
  }
  if(controller->get_controller_type() == "Neural_Controller" || controller->get_controller_type() == "Naive_Controller" || controller->get_controller_type() == "Semi_Hybrid_Controller")
  {
    myFile << std::endl << "\t<ServoDerivativeEpsilon>" << std::endl << "\t   " << controller->get_servo_derivative_epsilon() << std::endl << "\t</ServoDerivativeEpsilon>" << std::endl;
  }
  myFile << "</Controller>" << std::endl;

  myFile << std::endl << "<ExtendedKalmanFilter>" << std::endl;
  myFile << "\t<dt>" << std::endl << "\t   " << controller->get_EKF_dt() << std::endl << "\t</dt>" << std::endl;
  myFile << std::endl << "\t<r>" << std::endl << "\t   " << controller->get_EKF_r() << std::endl << "\t</r>" << std::endl;
  myFile << std::endl << "\t<qf>" << std::endl << "\t   " << controller->get_EKF_qf() << std::endl << "\t</qf>" << std::endl;
  myFile << "</ExtendedKalmanFilter>" << std::endl;

  if(mlp->get_inputs_number() > 0)
  {
    myFile << std::endl << "<NeuralNetwork>" << std::endl;
    myFile << "\t<NumberOfNNInputs>" << std::endl << "\t   " << mlp->get_inputs_number() << std::endl << "\t</NumberOfNNInputs>" << std::endl;
    if(mlp->get_hidden_layers_number() > 0)
    {
      myFile << std::endl << "\t<HiddenLayers>"; //<< "\t   " << number_of_inputs_to_NN << std::endl << "\t</NumberOfNNInputs>" << std::endl;
      for(int i=0; i<mlp->get_hidden_layers_number(); i++)
      {
        myFile << std::endl << "\t   <Layer>" << std::endl << "\t      " << mlp->get_hidden_layer_size(i) << std::endl << "\t   </Layer>" << std::endl;
      }
      myFile << "\t</HiddenLayers>" << std::endl;
    }
    myFile << std::endl << "\t<NumberOfNNOutputs>" << std::endl << "\t   " << mlp->get_outputs_number() << std::endl << "\t</NumberOfNNOutputs>" << std::endl;
    myFile << "</NeuralNetwork>" << std::endl;
  }

  if(mlp->get_independent_parameters_number() > 0)
  {
    myFile << std::endl << "<IndependentParameters>";
    for(int i=0; i<mlp->get_independent_parameters_number(); i++)
    {
      myFile << std::endl << "\t<Parameter>";
      myFile << std::endl << "\t   <Name>" << std::endl << "\t      " << mlp->get_independent_parameter_name(i) << std::endl << "\t   </Name>" << std::endl;
      myFile << std::endl << "\t   <Minimum>" << std::endl << "\t      " << mlp->get_independent_parameter_minimum(i) << std::endl << "\t   </Minimum>" << std::endl;
      myFile << std::endl << "\t   <Maximum>" << std::endl << "\t      " << mlp->get_independent_parameter_maximum(i) << std::endl << "\t   </Maximum>" << std::endl;
      myFile << "\t</Parameter>" << std::endl;
    }

    myFile << "</IndependentParameters>" << std::endl;
  }

  myFile << std::endl << "<Gene>" << std::endl;
  myFile << std::endl << "</Gene>" << std::endl;
  myFile << std::endl << "</Morphomotion>";

  myFile.close();
}*/


// CONSTRUCTOR FOR EXTRACTING PARAMETERS FROM PARAMETERS FILE
FileHandler::FileHandler(char* filename, Robot *robot, SimulationOpenRave *simuEnv, Controller *controller, Flood::MultilayerPerceptron *mlp)
{
  std::fstream file;
  file.open(filename, std::ios::in);
  if(!file.is_open())
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*)" << std::endl
              << "Cannot open Parameter file: "<< filename  << std::endl;

    exit(1);
  }

  myFile_name = filename;

  std::string line;
  std::string word;

  getline(file, line);

  if(line != "<Morphomotion: Flood + OpenRave + Y1>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*)" << std::endl
              << "Unknown file declaration: " << line << std::endl;

    exit(1);
  }

  //--File type
  file >> word;

  if(word != "<FileType>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*)" << std::endl
              << "Unknown file type begin tag: " << word << std::endl;

    exit(1);
  }

  file >> word;

  if(word != "Evolution_Parameters" && word != "GeneFile")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*)" << std::endl
              << "Unknown file type: " << word << std::endl;

    exit(1);
  }

  file >> word;

  if(word != "</FileType>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*)" << std::endl
              << "Unknown file type end tag: " << word << std::endl;

    exit(1);
  }

  do
  {
    file >> word;

    if(word == "<Robot>")
    {
      load_Robot_parameters(file, robot);
    }

    else if(word == "<SimulationEnvironment>")
    {
      load_SimEnv_parameters(file, simuEnv);
    }

    else if(word == "<Controller>")
    {
      load_Controller_parameters(file, controller);
    }

    else if(word == "<ExtendedKalmanFilter>")
    {
      load_ExtendedKalmanFilter_parameters(file, controller);
    }

    else if(word == "<NeuralNetwork>")
    {
      load_NN_parameters(file, mlp);
    }

    else if(word == "<IndependentParameters>")
    {
      load_independent_parameters(file, mlp, controller);
    }
  }while(word != "</Morphomotion>");
  file.close();
}

// CONSTRUCTOR FOR EXTRACTING PARAMETERS FROM GENE FILE
FileHandler::FileHandler(char* gene_filename, Robot *robot, SimulationOpenRave *simuEnv, Controller *controller, Flood::MultilayerPerceptron *mlp, Flood::Matrix<double>* population, std::vector<std::string>* generation_index)
{
  std::fstream gene_file;
  gene_file.open(gene_filename, std::ios::in);
  if(!gene_file.is_open())
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, std::vector<std::string>*)" << std::endl
              << "Cannot open Parameter file: "<< gene_filename  << std::endl;
    exit(1);
  }

  std::string line;
  std::string word;

  getline(gene_file, line);

  if(line != "<Morphomotion: Flood + OpenRave + Y1>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*)" << std::endl
              << "Unknown file declaration: " << line << std::endl;

    exit(1);
  }

  //--File type
  gene_file >> word;

  if(word != "<FileType>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*)" << std::endl
              << "Unknown file type begin tag: " << word << std::endl;

    exit(1);
  }

  gene_file >> word;

  if(word != "GeneFile")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*)" << std::endl
              << "Unknown file type: " << word << std::endl;

    exit(1);
  }

  gene_file >> word;

  if(word != "</FileType>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*)" << std::endl
              << "Unknown file type end tag: " << word << std::endl;

    exit(1);
  }

  do
  {
    gene_file >> word;

    if(word == "<Robot>")
    {
      load_Robot_parameters(gene_file, robot);
    }

    else if(word == "<SimulationEnvironment>")
    {
      if(simuEnv)
      {
        load_SimEnv_parameters(gene_file, simuEnv);
      }
    }

    else if(word == "<Controller>")
    {
      load_Controller_parameters(gene_file, controller);
    }

    else if(word == "<ExtendedKalmanFilter>")
    {
      load_ExtendedKalmanFilter_parameters(gene_file, controller);
    }

    else if(word == "<NeuralNetwork>")
    {
      load_NN_parameters(gene_file, mlp);
    }

    else if(word == "<IndependentParameters>")
    {
      load_independent_parameters(gene_file, mlp, controller);
    }

    else if(word == "<Gene>")
    {
      load_genes(gene_file, mlp, population, generation_index);
    }
  }while(word != "</Morphomotion>");
  gene_file.close();
}


//--CONSTRUCTOR FOR EXTRACTING PARAMETERS FROM GENE FILE AND FITNESS FILE
FileHandler::FileHandler(char* gene_filename, char* fitness_filename, Robot *robot, SimulationOpenRave *simuEnv, Controller *controller, Flood::MultilayerPerceptron *mlp, Flood::Matrix<double>* population, std::vector<std::string>* generation_index, std::vector<double>* elite_fitness)
{
  std::fstream gene_file;
  std::fstream fitness_file;

  gene_file.open(gene_filename, std::ios::in);
  if(!gene_file.is_open())
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, Flood::Vector<std::string>*, Flood::Vector<double>*)" << std::endl
              << "Cannot open Parameter file: "<< gene_filename  << std::endl;
    exit(1);
  }

  fitness_file.open(fitness_filename, std::ios::in);
  if(!fitness_file.is_open())
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, Flood::Vector<std::string>*, Flood::Vector<double>*)" << std::endl
              << "Cannot open Fitness file: "<< fitness_filename  << std::endl;
    exit(1);
  }

  std::string line;
  std::string word;

  getline(gene_file, line);

  if(line != "<Morphomotion: Flood + OpenRave + Y1>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, Flood::Vector<std::string>*, Flood::Vector<double>*)" << std::endl
              << "Unknown file declaration: " << line << std::endl;

    exit(1);
  }

  //--File type
  gene_file >> word;

  if(word != "<FileType>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, Flood::Vector<std::string>*, Flood::Vector<double>*)" << std::endl
              << "Unknown file type begin tag: " << word << std::endl;

    exit(1);
  }

  gene_file >> word;

  if(word != "GeneFile")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, Flood::Vector<std::string>*, Flood::Vector<double>*)" << std::endl
              << "Unknown file type: " << word << std::endl;

    exit(1);
  }

  gene_file >> word;

  if(word != "</FileType>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, char*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, Flood::Vector<std::string>*, Flood::Vector<double>*)" << std::endl
              << "Unknown file type end tag: " << word << std::endl;

    exit(1);
  }

  do
  {
    gene_file >> word;

    if(word == "<Robot>")
    {
      load_Robot_parameters(gene_file, robot);
    }

    else if(word == "<SimulationEnvironment>")
    {
      if(simuEnv)
      {
        load_SimEnv_parameters(gene_file, simuEnv);
      }
    }

    else if(word == "<Controller>")
    {
      load_Controller_parameters(gene_file, controller);
    }

    else if(word == "<ExtendedKalmanFilter>")
    {
      load_ExtendedKalmanFilter_parameters(gene_file, controller);
    }

    else if(word == "<NeuralNetwork>")
    {
      load_NN_parameters(gene_file, mlp);
    }

    else if(word == "<IndependentParameters>")
    {
      load_independent_parameters(gene_file, mlp, controller);
    }

    else if(word == "<Gene>")
    {
      load_genes(gene_file, mlp, population, generation_index);
    }
  }while(word != "</Morphomotion>");
  gene_file.close();

  load_elite_fitness(fitness_file, elite_fitness);
}


//--CONSTRUCTOR FOR EXTRACTING MLP PARAMETERS FROM GENE FILE -- For Neural Evaluator
FileHandler::FileHandler(char* gene_filename, Flood::MultilayerPerceptron *mlp, Flood::Matrix<double>* population, std::vector<std::string>* generation_index)
{
  std::fstream gene_file;
  gene_file.open(gene_filename, std::ios::in);
  if(!gene_file.is_open())
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, std::vector<std::string>*)" << std::endl
              << "Cannot open Parameter file: "<< gene_filename  << std::endl;
    exit(1);
  }

  std::string line;
  std::string word;

  getline(gene_file, line);

  if(line != "<Morphomotion: Flood + OpenRave + Y1>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, std::vector<std::string>*)" << std::endl
              << "Unknown file declaration: " << line << std::endl;

    exit(1);
  }

  //--File type
  gene_file >> word;

  if(word != "<FileType>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, std::vector<std::string>*)" << std::endl
              << "Unknown file type begin tag: " << word << std::endl;

    exit(1);
  }

  gene_file >> word;

  if(word != "GeneFile")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, std::vector<std::string>*)" << std::endl
              << "Unknown file type: " << word << std::endl;

    exit(1);
  }

  gene_file >> word;

  if(word != "</FileType>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, std::vector<std::string>*)" << std::endl
              << "Unknown file type end tag: " << word << std::endl;

    exit(1);
  }

  do
  {
    gene_file >> word;

    if(word == "<NeuralNetwork>")
    {
      load_NN_parameters(gene_file, mlp);
    }

    else if(word == "<IndependentParameters>")
    {
      //load_independent_parameters(gene_file, mlp);
    } // TODO: This should be removed after confirming that this is not needed.

    else if(word == "<Gene>")
    {
      load_genes(gene_file, mlp, population, generation_index);
    }
  }while(word != "</Morphomotion>");
  gene_file.close();
}


//--CONSTRUCTOR FOR EXTRACTING MLP PARAMETERS FROM GENE FILE AND FITNESS FILE -- For Neural Evaluator
FileHandler::FileHandler(char* gene_filename, char* fitness_filename, Flood::MultilayerPerceptron *mlp, Flood::Matrix<double>* population, std::vector<std::string>* generation_index, std::vector<double>* elite_fitness)
{
  std::fstream gene_file;
  std::fstream fitness_file;

  gene_file.open(gene_filename, std::ios::in);
  if(!gene_file.is_open())
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, char*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, Flood::Vector<std::string>*, Flood::Vector<double>*)" << std::endl
              << "Cannot open Parameter file: "<< gene_filename  << std::endl;
    exit(1);
  }

  fitness_file.open(fitness_filename, std::ios::in);
  if(!fitness_file.is_open())
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, char*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, Flood::Vector<std::string>*, Flood::Vector<double>*)" << std::endl
              << "Cannot open Fitness file: "<< fitness_filename  << std::endl;
    exit(1);
  }

  std::string line;
  std::string word;

  getline(gene_file, line);

  if(line != "<Morphomotion: Flood + OpenRave + Y1>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, char*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, Flood::Vector<std::string>*, Flood::Vector<double>*)" << std::endl
              << "Unknown file declaration: " << line << std::endl;

    exit(1);
  }

  //--File type
  gene_file >> word;

  if(word != "<FileType>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, char*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, Flood::Vector<std::string>*, Flood::Vector<double>*)" << std::endl
              << "Unknown file type begin tag: " << word << std::endl;

    exit(1);
  }

  gene_file >> word;

  if(word != "GeneFile")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, char*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, Flood::Vector<std::string>*, Flood::Vector<double>*)" << std::endl
              << "Unknown file type: " << word << std::endl;

    exit(1);
  }

  gene_file >> word;

  if(word != "</FileType>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "FileHandler(char*, char*, Flood::MultilayerPerceptron*, Flood::Matrix<double>*, Flood::Vector<std::string>*, Flood::Vector<double>*)" << std::endl
              << "Unknown file type end tag: " << word << std::endl;

    exit(1);
  }

  do
  {
    gene_file >> word;

    if(word == "<NeuralNetwork>")
    {
      load_NN_parameters(gene_file, mlp);
    }

    else if(word == "<IndependentParameters>")
    {
      //load_independent_parameters(gene_file, mlp);
    }

    else if(word == "<Gene>")
    {
      load_genes(gene_file, mlp, population, generation_index);
    }
  }while(word != "</Morphomotion>");
  gene_file.close();

  load_elite_fitness(fitness_file, elite_fitness);
}


//--DESTRUCTOR
FileHandler::~FileHandler(void)
{
}

//--METHODS

void FileHandler::open(const char* filename)
{
  myFile.open(filename, fstream::in | fstream::out | fstream::app);

  if(!myFile.is_open())
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "open(const char*)" << std::endl
              << "Cannot open file: "<< filename << std::endl;
    exit(1);
  }

  myFile_name = filename;
  myFile.close();
}


//-- FOR SAVING ELITE GENE POPULATION
void FileHandler::init_gene_file(std::string note,
               bool gen0_preeval,
               bool gen0_aug_pop,
               Flood::EvolutionaryAlgorithm *ea,
               Robot *robot,
               SimulationOpenRave *simuOR,
               Controller *controller,
               Flood::MultilayerPerceptron *mlp)
{
  time_t rawtime;
  struct tm * timeinfo;
  char filename[100];
  stringstream ss;

  time ( &rawtime );
  timeinfo = localtime ( &rawtime );
  strftime (filename, 100, "_%m_%d_%H_%M_elite_population.gne", timeinfo);
  ss << "../Evolution_Files/" << robot->get_robot_environment() << "/" << robot->get_robot_type() << "/" << controller->get_controller_type() << "/Gene_Files/" << robot->get_robot_environment() << filename;

  myFile_name = ss.str();
  cout <<" Opening elite_population data file: " << myFile_name << endl;
  myFile.open(myFile_name.c_str(), fstream::in | fstream::out | fstream::app);

  myFile << "<Morphomotion: Flood + OpenRave + Y1>" << std::endl;
  myFile << std::endl << "<FileType>" << std::endl << "   GeneFile" << std::endl << "</FileType>" << std::endl;

  if(note != "No Notes")
  {
    myFile << std::endl << "<Note>" << std::endl << note << std::endl << "</Note>" << std::endl;
  }

  myFile << std::endl << "<Gene_File>" << std::endl << myFile_name << std::endl << "</Gene_File>" << std::endl;

  char fitness_filename[100];
  std::string fitnessFile;
  strftime (fitness_filename, 100, "%m_%d_%H_%M_FitnessGraph.dat", timeinfo);
  stringstream fitness_file;
  fitness_file << "../Evolution_Files/" << robot->get_robot_environment() << "/" << robot->get_robot_type() << "/" << controller->get_controller_type() << "/FitnessGraph_Files/" << fitness_filename;
  fitnessFile = fitness_file.str();
  myFile << std::endl << "<Fitness_File>" << std::endl << fitness_file.str() << std::endl << "</Fitness_File>" << std::endl;

  char evol_filename[100];
  std::string evoFile;
  strftime (evol_filename, 100, "_%m_%d_%H_%M_evolution.evo", timeinfo);
  stringstream evol_file;
  evol_file << "../Evolution_Files/" << robot->get_robot_environment() << "/" << robot->get_robot_type() << "/" << controller->get_controller_type() << "/Evolution/" << robot->get_robot_environment() << evol_filename;
  evoFile = evol_file.str();
  myFile << std::endl << "<Evolution_File>" << std::endl << evol_file.str() << std::endl << "</Evolution_File>" << std::endl;

  myFile << std::endl << "<Evolution>" << std::endl;
  myFile << "\t<FitnessAssignmentMethod>" << std::endl << "\t   " << ea->get_fitness_assignment_method_name() << std::endl << "\t</FitnessAssignmentMethod>" << std::endl;
  myFile << std::endl << "\t<SelectionMethod>" << std::endl << "\t   " << ea->get_selection_method_name() << std::endl << "\t</SelectionMethod>" << std::endl;
  myFile << std::endl << "\t<RecombinationMethod>" << std::endl << "\t   " << ea->get_recombination_method_name() << std::endl << "\t</RecombinationMethod>" << std::endl;
  myFile << std::endl << "\t<MutationMethod>" << std::endl << "\t   " << ea->get_mutation_method_name() << std::endl << "\t</MutationMethod>" << std::endl;
  myFile << std::endl << "\t<EvaluationMethod>" << std::endl << "\t   " << robot->get_evaluation_method() << std::endl << "\t</EvaluationMethod>" << std::endl;
  myFile << std::endl << "\t<PopulationSize>" << std::endl << "\t   " << ea->get_population_size() << std::endl << "\t</PopulationSize>" << std::endl;
  myFile << std::endl << "\t<CrossoverPercentage>" << std::endl << "\t   " << ea->get_crossover_percentage() << std::endl << "\t</CrossoverPercentage>" << std::endl;
  myFile << std::endl << "\t<Elitism>" << std::endl << "\t   " << ea->get_elitism() << std::endl << "\t</Elitism>" << std::endl;
  myFile << std::endl << "\t<ElitismPercentage>" << std::endl << "\t   " << ea->get_elitism_percentage() << std::endl << "\t</ElitismPercentage>" << std::endl;
  myFile << std::endl << "\t<SelectivePressure>" << std::endl << "\t   " << ea->get_selective_pressure() << std::endl << "\t</SelectivePressure>" << std::endl;
  myFile << std::endl << "\t<RecombinationSize>" << std::endl << "\t   " << ea->get_recombination_size() << std::endl << "\t</RecombinationSize>" << std::endl;
  myFile << std::endl << "\t<MutationRate>" << std::endl << "\t   " << ea->get_mutation_rate() << std::endl << "\t</MutationRate>" << std::endl;
  myFile << std::endl << "\t<MutationRateForES>" << std::endl << "\t   " << ea->get_mutation_rate_for_ES() << std::endl << "\t</MutationRateForES>" << std::endl;
  myFile << std::endl << "\t<MutationRange>" << std::endl << "\t   " << ea->get_mutation_range() << std::endl << "\t</MutationRange>" << std::endl;
  myFile << std::endl << "\t<MutationRangeForES>" << std::endl << "\t   " << ea->get_mutation_range_for_ES() << std::endl << "\t</MutationRangeForES>" << std::endl;
  myFile << std::endl << "\t<EvaluationSampleSize>" << std::endl << "\t   " << ea->get_evaluation_sample_size() << std::endl << "\t</EvaluationSampleSize>" << std::endl;
  if(gen0_preeval)
  {
    myFile << std::endl << "\t<Gen0_Pre-Evaluation>" << std::endl << "\t   " << "True" << std::endl << "\t</Gen0_Pre-Evaluation>" << std::endl;
  }
  else
  {
    myFile << std::endl << "\t<Gen0_Pre-Evaluation>" << std::endl << "\t   " << "False" << std::endl << "\t</Gen0_Pre-Evaluation>" << std::endl;
  }
  if(gen0_aug_pop)
  {
    myFile << std::endl << "\t<Gen0_AugmentedPopulation>" << std::endl << "\t   " << "True" << std::endl << "\t</Gen0_AugmentedPopulation>" << std::endl;
  }
  else
  {
    myFile << std::endl << "\t<Gen0_AugmentedPopulation>" << std::endl << "\t   " << "False" << std::endl << "\t</Gen0_AugmentedPopulation>" << std::endl;
  }
  myFile << std::endl << "\t<Gen0_MinimumEvaluationValue>" << std::endl << "\t   " << ea->get_gen_0_min_evaluation_value() << std::endl << "\t</Gen0_MinimumEvaluationValue>" << std::endl;
  myFile << std::endl << "\t<Gen0_PopulationSize>" << std::endl << "\t   " << ea->get_gen_0_population_size() << std::endl << "\t</Gen0_PopulationSize>" << std::endl;
  myFile << "</Evolution>" << std::endl;

  myFile << std::endl << "<Robot>" << std::endl;
  myFile << "\t<RobotEnvironment>" << std::endl << "\t   " << robot->get_robot_environment() << std::endl << "\t</RobotEnvironment>" << std::endl;
  myFile << std::endl << "\t<RobotType>" << std::endl << "\t   " << robot->get_robot_type() << std::endl << "\t</RobotType>" << std::endl;
  myFile << std::endl << "\t<NumberOfModules>" << std::endl << "\t   " << robot->get_number_of_modules() << std::endl << "\t</NumberOfModules>" << std::endl;
  myFile << "</Robot>" << std::endl;

  if(robot->get_robot_environment() == "SimulationOpenRave")
  {
    if(simuOR)
    {
      myFile << std::endl << "<SimulationEnvironment>" << std::endl;
      myFile << "\t<SceneFileName>" << std::endl << "\t   " << simuOR->get_scene_file_name() << std::endl << "\t</SceneFileName>" << std::endl;
      myFile << std::endl << "\t<SimResolution>" << std::endl << "\t   " << simuOR->get_simu_resolution_microseconds() << std::endl << "\t</SimResolution>" << std::endl;
      myFile << "</SimulationEnvironment>" << std::endl;
    }
    else
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "FileHandler(std::string, bool, bool, Flood::EvolutionaryAlgorithm*, Robot*, SimulationOpenRave*, Controller*, Flood::MultilayerPerceptron*)" << std::endl
                << "robot_pointer = NULL " << std::endl;
      exit(1);
    }
  }

  myFile << std::endl << "<Controller>" << std::endl;
  myFile << "\t<ControllerType>" << std::endl << "\t   " << controller->get_controller_type() << std::endl << "\t</ControllerType>" << std::endl;
  if(controller->get_controller_type() == "Fourier_Controller")
  {
    myFile << std::endl << "\t<FrequencyDomainSize>" << std::endl << "\t   " << controller->get_frequency_domain_size() << std::endl << "\t</FrequencyDomainSize>" << std::endl;
  }
  myFile << std::endl << "\t<EvaluationPeriod>" << std::endl << "\t   " << controller->get_evaluation_period() << std::endl << "\t</EvaluationPeriod>" << std::endl;
  myFile << std::endl << "\t<ServoMax>" << std::endl << "\t   " << controller->get_servo_max() << std::endl << "\t</ServoMax>" << std::endl;
  myFile << std::endl << "\t<ServoMin>" << std::endl << "\t   " << controller->get_servo_min() << std::endl << "\t</ServoMin>" << std::endl;
  if(controller->get_controller_type() != "Sine_Controller" &&
     controller->get_controller_type() != "InverseSine_Controller" &&
     controller->get_controller_type() != "InverseSine_Controller_V2" &&
     controller->get_controller_type() != "InverseSine_Controller_V3" &&
     controller->get_controller_type() != "Fourier_Controller" &&
     controller->get_controller_type() != "TriangleSquare_Controller")
  {
    myFile << std::endl << "\t<StartAngleType>" << std::endl << "\t   " << controller->get_start_angle_type() << std::endl << "\t</StartAngleType>" << std::endl;
    myFile << std::endl << "\t<ServoDeltaThreshold>" << std::endl << "\t   " << controller->get_servo_delta_threshold() << std::endl << "\t</ServoDeltaThreshold>" << std::endl;
  }
  if(controller->get_controller_type() == "Neural_Controller" || controller->get_controller_type() == "Naive_Controller")
  {
    myFile << std::endl << "\t<ServoDerivativeThreshold>" << std::endl << "\t   " << controller->get_servo_derivative_threshold() << std::endl << "\t</ServoDerivativeThreshold>" << std::endl;
  }
  if(controller->get_controller_type() == "Neural_Controller" || controller->get_controller_type() == "Naive_Controller" || controller->get_controller_type() == "Semi_Hybrid_Controller")
  {
    myFile << std::endl << "\t<ServoDerivativeEpsilon>" << std::endl << "\t   " << controller->get_servo_derivative_epsilon() << std::endl << "\t</ServoDerivativeEpsilon>" << std::endl;
  }
  myFile << "</Controller>" << std::endl;

  myFile << std::endl << "<ExtendedKalmanFilter>" << std::endl;
  myFile << "\t<dt>" << std::endl << "\t   " << controller->get_EKF_dt() << std::endl << "\t</dt>" << std::endl;
  myFile << std::endl << "\t<r>" << std::endl << "\t   " << controller->get_EKF_r() << std::endl << "\t</r>" << std::endl;
  myFile << std::endl << "\t<qf>" << std::endl << "\t   " << controller->get_EKF_qf() << std::endl << "\t</qf>" << std::endl;
  myFile << "</ExtendedKalmanFilter>" << std::endl;

  if(mlp->get_inputs_number() > 0)
  {
    myFile << std::endl << "<NeuralNetwork>" << std::endl;
    myFile << "\t<NumberOfNNInputs>" << std::endl << "\t   " << mlp->get_inputs_number() << std::endl << "\t</NumberOfNNInputs>" << std::endl;
    if(mlp->get_hidden_layers_number() > 0)
    {
      myFile << std::endl << "\t<HiddenLayers>"; //<< "\t   " << number_of_inputs_to_NN << std::endl << "\t</NumberOfNNInputs>" << std::endl;
      for(int i=0; i<mlp->get_hidden_layers_number(); i++)
      {
        myFile << std::endl << "\t   <Layer>" << std::endl << "\t      " << mlp->get_hidden_layer_size(i) << std::endl << "\t   </Layer>" << std::endl;
      }
      myFile << "\t</HiddenLayers>" << std::endl;
    }
    myFile << std::endl << "\t<NumberOfNNOutputs>" << std::endl << "\t   " << mlp->get_outputs_number() << std::endl << "\t</NumberOfNNOutputs>" << std::endl;
    myFile << "</NeuralNetwork>" << std::endl;
  }

  if(mlp->get_independent_parameters_number() > 0)
  {
    myFile << std::endl << "<IndependentParameters>";
    for(int i=0; i<mlp->get_independent_parameters_number(); i++)
    {
      myFile << std::endl << "\t<Parameter>";
      myFile << std::endl << "\t   <Name>" << std::endl << "\t      " << mlp->get_independent_parameter_name(i) << std::endl << "\t   </Name>" << std::endl;
      myFile << std::endl << "\t   <Minimum>" << std::endl << "\t      " << mlp->get_independent_parameter_minimum(i) << std::endl << "\t   </Minimum>" << std::endl;
      myFile << std::endl << "\t   <Maximum>" << std::endl << "\t      " << mlp->get_independent_parameter_maximum(i) << std::endl << "\t   </Maximum>" << std::endl;
      myFile << "\t</Parameter>" << std::endl;
    }

    myFile << "</IndependentParameters>" << std::endl;
  }

  myFile << std::endl << "<Gene>" << std::endl;
  myFile << std::endl << "</Gene>" << std::endl;
  myFile << std::endl << "</Morphomotion>";

  myFile.close();
}


//-- FOR SAVING EVOLUTION HISTORY
void FileHandler::init_evol_file(Robot *robot, Controller *controller)
{
  time_t rawtime;
  struct tm * timeinfo;
  char filename[100];
  stringstream ss;

  time ( &rawtime );
  timeinfo = localtime ( &rawtime );
  strftime (filename, 100, "_%m_%d_%H_%M_evolution.evo", timeinfo);
  //strftime (filename, 100, file_format, timeinfo);
  ss << "../Evolution_Files/" << robot->get_robot_environment() << "/" << robot->get_robot_type() << "/" << controller->get_controller_type() << "/Evolution/" << robot->get_robot_environment() << filename;

  myFile_name = ss.str();
  cout <<" Opening file: " << myFile_name << endl;
  myFile.open(myFile_name.c_str(), fstream::in | fstream::out | fstream::app);

  myFile << "<Morphomotion: Flood + OpenRave + Y1>" << std::endl;
  myFile << std::endl << "<FileType>" << std::endl << "   EvolutionFile" << std::endl << "</FileType>" << std::endl;
  myFile << std::endl << "</Morphomotion>";

  myFile.close();
}


void FileHandler::save_gene(int generation, Flood::Vector<double> elite_gene)
{
  myFile.open(myFile_name.c_str(), fstream::in | fstream::out);

  if(!myFile.is_open())
  {
    std::cerr << std::endl
              << "Morphomotion Error: FileHandler class." << std::endl
              << "void save_gene(int, vector<double>) method." << std::endl
              << "Cannot open Gene file."  << std::endl;
    exit(1);
  }
  else
  {
    std::cout << std::endl
              << "Saving elite members to gene file..." << std::endl;
  }

  /*std::string word;
  do
  {
    myFile >> word;
  }while(word != "</Gene>");*/
  myFile.seekp (-25, ios::end);

  if(generation != 1)
  {
    myFile << std::endl;
  }
  myFile << "Generation_" << generation << ":" << std::endl;
  myFile << elite_gene << std::endl;

  myFile << "</Gene>" << std::endl;
  myFile << std::endl << "</Morphomotion>" << std::endl;

  myFile.close();
}


void FileHandler::save_generation_history(const int generation, const int individual, Flood::Vector<double> gene)
{
  myFile.open(myFile_name.c_str(), fstream::in | fstream::out);

  if(!myFile.is_open())
  {
    std::cerr << std::endl
              << "Morphomotion Error: FileHandler class." << std::endl
              << "void save_generation_history(const int, const int, const vector<double>) method." << std::endl
              << "Cannot open Gene file."  << std::endl;
    exit(1);
  }

  if(individual == 1)
  {
    myFile.seekp (-16, ios::end);

    if(generation == 1)
    {
       myFile << std::endl;
    }

    myFile << "<Generation_" << generation << ">" << std::endl << "<Gene>" << std::endl;
  }
  else
  {
    if(generation >= 10)
    {
      myFile.seekp (-42, ios::end);
    }
    else
    {
      myFile.seekp (-41, ios::end);
    }
    myFile << std::endl;
  }

  myFile << "Candidate_" << individual << ":" << std::endl << gene << std::endl;
  myFile << "</Gene>" << std::endl << "</Generation_" << generation << ">" << std::endl;
  myFile << std::endl << "</Morphomotion>" << std::endl;

  myFile.close();
}


void FileHandler::save_fitness_history(const int generation, const Flood::Vector<double> fitness)
{
  myFile.open(myFile_name.c_str(), fstream::in | fstream::out);

  if(!myFile.is_open())
  {
    std::cerr << std::endl
              << "Morphomotion Error: FileHandler class." << std::endl
              << "void save_fitness_history(const int, const vector<double>) method." << std::endl
              << "Cannot open Gene file."  << std::endl;
    exit(1);
  }

  if(generation >= 10)
  {
    myFile.seekp (-34, ios::end);
  }
  else
  {
    myFile.seekp (-33, ios::end);
  }
  myFile << std::endl << "<Fitness>" << std::endl;

  for(unsigned int i=0; i<fitness.get_size(); i++)
  {
    myFile << i+1 << ":" << std::endl << fitness[i] << std::endl;
  }
  myFile << "</Fitness>" << std::endl << "</Generation_" << generation << ">" << std::endl;
  myFile << std::endl << "</Morphomotion>" << std::endl;

  myFile.close();
}


void FileHandler::save_fitness_individual(const int generation, const unsigned int individual, const double fitness)
{
  myFile.open(myFile_name.c_str(), fstream::in | fstream::out);

  if(!myFile.is_open())
  {
    std::cerr << std::endl
              << "Morphomotion Error: FileHandler class." << std::endl
              << "void save_fitness_history(const int, const vector<double>) method." << std::endl
              << "Cannot open Gene file."  << std::endl;
    exit(1);
  }

  std::string word;
  std::stringstream gen;

  gen << "<Generation_" << generation << ">";

  do
  {
    myFile >> word;

    if(word == gen.str())
    {
      break;
    }
  }while(!myFile.eof());

  if(myFile.eof())
  {

  }
  else
  {
    do
    {
      myFile >> word;

      if(word == "</Gene>")
      {
        break;
      }
    }while(word.compare(0,13,"</Generation_"));

    if(individual == 1)
    {
      myFile << std::endl << std::endl << "<Fitness>";
    }
    else
    {
      std::stringstream ss;
      ss << individual-1 << ":";

      do
      {
        myFile >> word;

        if(word == ss.str())
        {
          myFile >> word;
          break;
        }
      }while(!myFile.eof());
    }
    myFile  << std::endl << individual << ":" << std::endl << fitness << std::endl;
    myFile << "</Fitness>" << std::endl << "</Generation_" << generation << ">" << std::endl;
    myFile << std::endl << "</Morphomotion>" << std::endl;
  }

  myFile.close();
}

bool FileHandler::find_tag(const std::string tag)
{
  myFile.open(myFile_name.c_str(), fstream::in);

  if(!myFile.is_open())
  {
    std::cerr << std::endl
              << "Morphomotion Error: FileHandler class." << std::endl
              << "bool find_tag(const std::string) method." << std::endl
              << "Cannot open Evolution file."  << std::endl;
    exit(1);
  }

  std::string word;

  do
  {
    myFile >> word;

    if(word == tag)
    {
      myFile.close();
      return true;
    }
  }while(!myFile.eof());

  myFile.close();

  return false;
}

void FileHandler::populate_from_evolution_file(Flood::EvolutionaryAlgorithm *ea)
{
  myFile.open(myFile_name.c_str(), fstream::in | fstream::out);

  if(!myFile.is_open())
  {
    std::cerr << std::endl
              << "Morphomotion Error: FileHandler class." << std::endl
              << "void populate_from_evolution_file(Flood::EvolutionaryAlgorithm*) method." << std::endl
              << "Cannot open Evolution file."  << std::endl;
    exit(1);
  }

  std::string line;
  std::string word;
  //std::string controller_type;

  unsigned int generations = 0;

  getline(myFile, line);

  if(line != "<Morphomotion: Flood + OpenRave + Y1>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "void populate_from_evolution_file(Flood::EvolutionaryAlgorithm*) method." << std::endl
              << "Unknown file declaration: " << line << std::endl;

    exit(1);
  }

  //--File type
  myFile >> word;

  if(word != "<FileType>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "void populate_from_evolution_file(Flood::EvolutionaryAlgorithm*) method." << std::endl
              << "Unknown file type begin tag: " << word << std::endl;

    exit(1);
  }

  myFile >> word;

  if(word != "EvolutionFile")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "void populate_from_evolution_file(Flood::EvolutionaryAlgorithm*) method." << std::endl
              << "Unknown file type: " << word << std::endl;

    exit(1);
  }

  myFile >> word;

  if(word != "</FileType>")
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "void populate_from_evolution_file(Flood::EvolutionaryAlgorithm*) method." << std::endl
              << "Unknown file type end tag: " << word << std::endl;

    exit(1);
  }

  do
  {
    myFile >> word;

    if(word.compare(0,12, "<Generation_") == 0)
    {
      generations++;
    }

  }while(word != "</Morphomotion>");
  std::cout << std::endl << "Generations: " << generations << std::endl; // TODO: Debugger to be removed
  myFile.close();

  if(generations > 0)
  {
    myFile.open(myFile_name.c_str(), fstream::in | fstream::out);

    unsigned int neural_parameters_number = ea->get_objective_functional_pointer()->get_multilayer_perceptron_pointer()->get_neural_parameters_number();
    unsigned int independent_parameters_number = ea->get_objective_functional_pointer()->get_multilayer_perceptron_pointer()->get_independent_parameters_number();
    unsigned int parameters_number = neural_parameters_number + independent_parameters_number;

    Flood::Matrix<double> new_population(ea->get_population_size(), parameters_number);
    Flood::Vector<double> individual(parameters_number);

    //std::cout << std::endl << "parameters_number: " << parameters_number << std::endl; // TODO: Debugger to be removed

    std::stringstream ss;
    ss << "<Generation_" << generations << ">";

    do
    {
      myFile >> word;

      if(word == ss.str())
      {
        break;
      }
    }while(word != ss.str() && !myFile.eof());

    if(myFile.eof())
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "void populate_from_evolution_file(Flood::EvolutionaryAlgorithm*) method." << std::endl
                << "Reached EOF. Cannot find tag: " << ss.str() << std::endl;

      exit(1);
    }

    myFile >> word;
    if(word != "<Gene>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "void populate_from_evolution_file(Flood::EvolutionaryAlgorithm*) method." << std::endl
                << "Unknown Gene begin tag: " << word << std::endl;

      exit(1);
    }

    for(unsigned int i=0; i<ea->get_population_size(); i++)
    {
      myFile >> word;
      std::stringstream candidate;
      candidate << "Candidate_" << i+1 << ":";
      if(word != candidate.str())
      {
        std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                  << "void populate_from_evolution_file(Flood::EvolutionaryAlgorithm*) method." << std::endl
                  << "Unknown Candidate: " << word << std::endl;

        exit(1);
      }
      else
      {
        for(unsigned int j=0; j<parameters_number; j++)
        {
          myFile >> individual[j];
        }
        new_population.set_row(i, individual);
      }
    }

    myFile >> word;
    if(word != "</Gene>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "void populate_from_evolution_file(Flood::EvolutionaryAlgorithm*) method." << std::endl
                << "Unknown Gene end tag: " << word << std::endl;

      exit(1);
    }

    ea->set_population(new_population);
    ea->set_generation_size(generations);

    myFile >> word;
    if(word == "<Fitness>")
    {
      unsigned int eval_indx = 0;
      Flood::Vector<double> evaluation(ea->get_population_size());
      evaluation.initialize(0);

      do
      {
        std::stringstream ss;
        ss << eval_indx+1 << ":";

        myFile >> word;
        if(word == ss.str())
        {
          myFile >> evaluation[eval_indx];
          eval_indx++;
        }
      }while(word != "</Fitness>");

      ea->set_evaluation(evaluation);
      ea->set_evaluation_index(eval_indx);
    }

    myFile.close();
  }
  else
  {
    std::cerr << "Morphomotion Error: FileHandler class." << std::endl
              << "void populate_from_evolution_file(Flood::EvolutionaryAlgorithm*) method." << std::endl
              << "No genes to populate in the Evolution file" << std::endl;

    exit(1);
  }
}


void FileHandler::load_Robot_parameters(std::fstream& file, Robot *robot)
{
  std::string word;

  do
  {
    file >> word;

    //-- TODO: This was previouslt uncommented. Need to be looked at
    /*if(word == "<RobotEnvironment>")
    {
      std::string new_robot_environment;
      file >> new_robot_environment;
      robot->set_robot_environment(new_robot_environment);

      file >> word;

      if(word != "</RobotEnvironment>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_Robot_parameters(std::fstream&, Robot*) method." << std::endl
                  << "Unknown Robot Environment end tag: " << word << std::endl;

        exit(1);
      }
    }*/

    if(word == "<RobotType>")
    {
      std::string new_robot_type;
      file >> new_robot_type;
      robot->set_robot_type(new_robot_type);

      file >> word;

      if(word != "</RobotType>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_Robot_parameters(std::fstream&, Robot*) method." << std::endl
                  << "void load_SimEnv_parameters(std::fstream&, SimEnv*) method." << std::endl
                  << "Unknown Robot Type end tag: " << word << std::endl;

        exit(1);
      }
    }

    else if(word == "<NumberOfModules>")
    {
      int new_number_of_modules;
      file >> new_number_of_modules;
      robot->set_number_of_modules(new_number_of_modules);

      file >> word;

      if(word != "</NumberOfModules>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_Robot_parameters(std::fstream&, Robot*) method." << std::endl
                  << "Unknown Number Of Modules end tag: " << word << std::endl;

        exit(1);
      }
    }

  }while(word != "</Robot>");
}


void FileHandler::load_SimEnv_parameters(std::fstream& file, SimulationOpenRave *simuEnv)
{
  std::string word;

  do
  {
    file >> word;

    if(word == "<SceneFileName>")
    {
      std::string new_scene_file_name;
      file >> new_scene_file_name;
      simuEnv->set_scene_file_name(new_scene_file_name);

      file >> word;

      if(word != "</SceneFileName>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_SimEnv_parameters(std::fstream&, SimulationOpenRave*) method." << std::endl
                  << "Unknown Scene File name end tag: " << word << std::endl;

        exit(1);
      }
    }

    else if(word == "<SimResolution>")
    {
      double new_sim_resolution;
      file >> new_sim_resolution;
      simuEnv->set_simu_resolution_microseconds(new_sim_resolution);

      file >> word;

      if(word != "</SimResolution>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_SimEnv_parameters(std::fstream&, SimulationOpenRave*) method." << std::endl
                  << "Unknown Simulation Resolution end tag: " << word << std::endl;

        exit(1);
      }
    }
   }while(word != "</SimulationEnvironment>");
}


void FileHandler::load_Controller_parameters(std::fstream& file, Controller *controller)
{
  std::string word;

  do
  {
    file >> word;

    if(word == "<ControllerType>")
    {
      std::string new_controller_type;
      file >> new_controller_type;
      controller->set_controller_type(new_controller_type);

      file >> word;

      if(word != "</ControllerType>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_Controller_parameters(std::fstream&, Controller*) method." << std::endl
                  << "Unknown Controller Type end tag: " << word << std::endl;

        exit(1);
      }
    }

    else if(word == "<FrequencyDomainSize>")
    {
      unsigned int new_frequency_domain_size;
      file >> new_frequency_domain_size;
      controller->set_frequency_domain_size(new_frequency_domain_size);

      file >> word;

      if(word != "</FrequencyDomainSize>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_Controller_parameters(std::fstream&, Controller*) method." << std::endl
                  << "Unknown Frequency Domain Size end tag: " << word << std::endl;

        exit(1);
      }
    }

    else if(word == "<EvaluationPeriod>")
    {
      unsigned int new_evaluation_period;
      file >> new_evaluation_period;
      controller->set_evaluation_period(new_evaluation_period);

      file >> word;

      if(word != "</EvaluationPeriod>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_Controller_parameters(std::fstream&, Controller*) method." << std::endl
                  << "Unknown Evaluation Period end tag: " << word << std::endl;

        exit(1);
      }
    }

    else if(word == "<ServoMax>")
    {
      double new_servo_max;
      file >> new_servo_max;
      controller->set_servo_max(new_servo_max);

      file >> word;

      if(word != "</ServoMax>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_Controller_parameters(std::fstream&, Controller*) method." << std::endl
                  << "Unknown Servo Max end tag: " << word << std::endl;

        exit(1);
      }
    }

    else if(word == "<ServoMin>")
    {
      double new_servo_min;
      file >> new_servo_min;
      controller->set_servo_min(new_servo_min);

      file >> word;

      if(word != "</ServoMin>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_Controller_parameters(std::fstream&, Controller*) method." << std::endl
                  << "Unknown Servo Min end tag: " << word << std::endl;

        exit(1);
      }
    }


    else if(word == "<StartAngleType>")
    {
      std::string new_start_angle_type;
      file >> new_start_angle_type;
      if(new_start_angle_type == "Zero")
      {
        controller->set_start_angle_type("Zero");
      }
      else if(new_start_angle_type == "Random")
      {
        controller->set_start_angle_type("Random");
      }
      else if(new_start_angle_type == "RandomEqual")
      {
        controller->set_start_angle_type("RandomEqual");
      }
      else if(new_start_angle_type == "Predefined")
      {
        controller->set_start_angle_type("Predefined");
      }
      else if(new_start_angle_type == "RunTime")
      {
        controller->set_start_angle_type("RunTime");
      }
      else
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_Controller_parameters(std::fstream&, Controller*) method." << std::endl
                  << "Unknown Start Angle type: " << word << " in <StartAngleType>" << std::endl;

        exit(1);
      }

      file >> word;

      if(word != "</StartAngleType>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_Controller_parameters(std::fstream&, Controller*) method." << std::endl
                  << "Unknown Start Angle end tag: " << word << std::endl;

        exit(1);
      }
    }

    else if(word == "<ServoDeltaThreshold>")
    {
      double new_servo_delta_threshold;
      file >> new_servo_delta_threshold;
      controller->set_servo_delta_threshold(new_servo_delta_threshold);

      file >> word;

      if(word != "</ServoDeltaThreshold>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_Controller_parameters(std::fstream&, Controller*) method." << std::endl
                  << "Unknown Servo Delta Threshold end tag: " << word << std::endl;

        exit(1);
      }
    }

    else if(word == "<ServoDerivativeThreshold>")
    {
      double new_servo_derivative_threshold;
      file >> new_servo_derivative_threshold;
      controller->set_servo_derivative_threshold(new_servo_derivative_threshold);

      file >> word;

      if(word != "</ServoDerivativeThreshold>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_Controller_parameters(std::fstream&, Controller*) method." << std::endl
                  << "Unknown Servo Derivative Threshold end tag: " << word << std::endl;

        exit(1);
      }
    }

    else if(word == "<ServoDerivativeEpsilon>")
    {
      int new_servo_derivative_epsilon;
      file >> new_servo_derivative_epsilon;
      controller->set_servo_derivative_epsilon(new_servo_derivative_epsilon);

      file >> word;

      if(word != "</ServoDerivativeEpsilon>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_Controller_parameters(std::fstream&, Controller*) method." << std::endl
                  << "Unknown Servo Derivative Epsilon end tag: " << word << std::endl;

        exit(1);
      }
    }
  }while(word != "</Controller>");
}


void FileHandler::load_ExtendedKalmanFilter_parameters(std::fstream& file, Controller *controller)
{
  std::string word;

  do
  {
    file >> word;

    if(word == "<dt>")
    {
      double new_dt;
      file >> new_dt;
      controller->set_EKF_dt(new_dt);

      file >> word;

      if(word != "</dt>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_ExtendedKalmanFilter_parameters(std::fstream&, Controller*) method." << std::endl
                  << "Unknown dt end tag: " << word << std::endl;

        exit(1);
      }
    }

    else if(word == "<r>")
    {
      double new_r;
      file >> new_r;
      controller->set_EKF_r(new_r);

      file >> word;

      if(word != "</r>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_ExtendedKalmanFilter_parameters(std::fstream&, Controller*) method." << std::endl
                  << "Unknown dt end tag: " << word << std::endl;

        exit(1);
      }
    }

    else if(word == "<qf>")
    {
      double new_qf;
      file >> new_qf;
      controller->set_EKF_qf(new_qf);

      file >> word;

      if(word != "</qf>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_ExtendedKalmanFilter_parameters(std::fstream&, Controller*) method." << std::endl
                  << "Unknown dt end tag: " << word << std::endl;

        exit(1);
      }
    }
  }while(word != "</ExtendedKalmanFilter>");
}


void FileHandler::load_NN_parameters(std::fstream& file, Flood::MultilayerPerceptron *mlp)
{
  std::string word;
  int nnInput = 0;
  int nnOutput = 0;
  std::vector<int> hiddenLayers;

  do
  {
    file >> word;

    if(word == "<NumberOfNNInputs>")
    {
      file >> word;
      nnInput = atoi(word.c_str());

      file >> word;

      if(word != "</NumberOfNNInputs>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_NN_parameters(std::fstream&, Flood::MultilayerPerceptron*) method." << std::endl
                  << "Unknown Number of NNInputs end tag: " << word << std::endl;

        exit(1);
      }
    }

    else if(word == "<NumberOfNNOutputs>")
    {
      file >> word;
      nnOutput = atoi(word.c_str());

      file >> word;

      if(word != "</NumberOfNNOutputs>")
      {
        std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                  << "void load_NN_parameters(std::fstream&, Flood::MultilayerPerceptron*) method." << std::endl
                  << "Unknown Number of NNOutputs end tag: " << word << std::endl;

        exit(1);
      }
    }

    else if(word == "<HiddenLayers>")
    {
      do
      {
        file >> word;

        if(word == "<Layer>")
        {
          file >> word;
          hiddenLayers.push_back(atoi(word.c_str()));

          file >> word;

          if(word != "</Layer>")
          {
            std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                      << "void load_NN_parameters(std::fstream&, Flood::MultilayerPerceptron*) method." << std::endl
                      << "Unknown NN hidden layers end tag: " << word << std::endl;

            exit(1);
          }
        }
      }while(word != "</HiddenLayers>");
    }
  }while(word != "</NeuralNetwork>");

  Flood::Vector<int> nnHiddenNeurons(hiddenLayers.size());
  for(unsigned int i=0; i<hiddenLayers.size(); i++)
  {
    nnHiddenNeurons[i] = hiddenLayers[i];
  }

  mlp->set_network_architecture (nnInput, nnHiddenNeurons, nnOutput);
}


void FileHandler::load_independent_parameters(std::fstream& file, Flood::MultilayerPerceptron *mlp, Controller *controller)
{
  std::string word;
  unsigned int independentParametersNumber = 0;
  std::vector<std::string> independentParametersName;
  std::vector<double> independentParametersMinimum;
  std::vector<double> independentParametersMaximum;

  do
  {
    file >> word;

    if(word == "<Parameter>")
    {
      unsigned int totalParameters=1;
      independentParametersNumber++;

      do
      {
        file >> word;

        if(word == "<TotalParameters>")
        {
          file >> word;
          totalParameters = atoi(word.c_str());

          if(totalParameters > 1)
          {
            //--Undoing incrementing independentParametersNumber a few lines above.
            independentParametersNumber--;

            if(controller->get_controller_type() == "Fourier_Controller")
            {
              unsigned int length = file.tellg();
              do
              {
                file >> word;
                if(word == "<Name>")
                {
                  file >> word;
                  if(word == "Ak" || word == "Bk")
                  {
                    independentParametersNumber = independentParametersNumber + (totalParameters*controller->get_frequency_domain_size());
                    totalParameters = totalParameters*controller->get_frequency_domain_size();
                  }
                  else
                  {
                    independentParametersNumber = independentParametersNumber + totalParameters;
                  }

                  file >> word;
                }
              }while(word != "</Name>");
              file.seekg(length, file.beg);
            }
            else
            {
              independentParametersNumber = independentParametersNumber + totalParameters;
            }
          }

          file >> word;

          if(word != "</TotalParameters>")
          {
            std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                      << "void load_independent_parameters(std::fstream&, Flood::MultilayerPerceptron*) method." << std::endl
                      << "Unknown Total Parameters end tag: " << word << std::endl;

            exit(1);
          }
        }

        else if(word == "<Name>")
        {
          std::string new_independent_parameter_name;
          file >> new_independent_parameter_name;

          if(totalParameters > 1)
          {
            if(new_independent_parameter_name == "Ak" || new_independent_parameter_name == "Bk")
            {
              for(unsigned int module=0; module<controller->get_robot_primary()->get_number_of_modules(); module++)
              {
                for(unsigned int parameter=0; parameter<controller->get_frequency_domain_size(); parameter++)
                {
                  //std::string module_no;
                  std::string str;
                  ostringstream intToString;
                  intToString << "-module_" << module+1 << "-param_" << parameter+1;
                  str = intToString.str();

                  std::string parameter_name = new_independent_parameter_name + str;
                  independentParametersName.push_back(parameter_name);
                }
              }
            }
            else
            {
              for(unsigned int parameter=0; parameter<totalParameters; parameter++)
              {
                std::string par_no;
                ostringstream intToString;
                intToString << parameter+1;
                par_no = intToString.str();

                std::string parameter_name = new_independent_parameter_name + "-module_" + par_no;
                independentParametersName.push_back(parameter_name);
              }
            }
          }
          else
          {
            independentParametersName.push_back(new_independent_parameter_name);
          }

          file >> word;

          if(word != "</Name>")
          {
            std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                      << "void load_independent_parameters(std::fstream&, Flood::MultilayerPerceptron*) method." << std::endl
                      << "Unknown Independent Parameters Name end tag: " << word << std::endl;

            exit(1);
          }
        }

        else if(word == "<Minimum>")
        {
          file >> word;
          for(unsigned int parameter=0; parameter<totalParameters; parameter++)
          {
            //independentParametersMinimum.push_back(atod(word.c_str())); //TODO: Need to change this to atod().
            independentParametersMinimum.push_back(atof(word.c_str()));
          }

          file >> word;

          if(word != "</Minimum>")
          {
            std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                      << "void load_independent_parameters(std::fstream&, Flood::MultilayerPerceptron*) method." << std::endl
                      << "Unknown Independent Parameters Minimum end tag: " << word << std::endl;

            exit(1);
          }
        }

        else if(word == "<Maximum>")
        {
          file >> word;
          for(unsigned int parameter=0; parameter<totalParameters; parameter++)
          {
            //independentParametersMaximum.push_back(atod(word.c_str()));  //TODO: Need to change this to atod().
            independentParametersMaximum.push_back(atof(word.c_str()));  //TODO: Need to change this to atod().
          }

          file >> word;

          if(word != "</Maximum>")
          {
            std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                      << "void load_independent_parameters(std::fstream&, Flood::MultilayerPerceptron*) method." << std::endl
                      << "Unknown Independent Parameters Maximum end tag: " << word << std::endl;

            exit(1);
          }
        }
      }while(word != "</Parameter>");
    }
  }while(word != "</IndependentParameters>");

  if(independentParametersNumber != independentParametersName.size() || independentParametersNumber != independentParametersMinimum.size() || independentParametersNumber != independentParametersMaximum.size())
  {
    std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
              << "void load_independent_parameters(std::fstream&, Flood::MultilayerPerceptron*) method." << std::endl
              << "Discrepency in Independent Parameters size." << std::endl;
    exit(1);
  }

  Flood::Vector<string> Independent_Parameters_Name(independentParametersNumber);
  Flood::Vector<double> Independent_Parameters_Minimum(independentParametersNumber);
  Flood::Vector<double> Independent_Parameters_Maximum(independentParametersNumber);

  for(unsigned int i=0; i<independentParametersNumber; i++)
  {
    Independent_Parameters_Name[i] = independentParametersName[i];
    Independent_Parameters_Minimum[i] = independentParametersMinimum[i];
    Independent_Parameters_Maximum[i] = independentParametersMaximum[i];
  }

  mlp->set_independent_parameters_number(independentParametersNumber);

  mlp->set_independent_parameters_name(Independent_Parameters_Name);
  mlp->set_independent_parameters_minimum(Independent_Parameters_Minimum);
  mlp->set_independent_parameters_maximum(Independent_Parameters_Maximum);
}


void FileHandler::load_genes(std::fstream& gene_file, Flood::MultilayerPerceptron *mlp, Flood::Matrix<double>* population, std::vector<std::string>* generation_index)
{
   std::string word;
   int parametersNumber = mlp->get_parameters_number();
   Flood::Vector<double> individual;
   individual.set(parametersNumber);
   population->set(1, parametersNumber);


   gene_file >> word;
   if(word != "Generation_0:" && word != "Generation_1:")
   {
      std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                << "void load_genes(std::fstream&, Flood::MultilayerPerceptron*, Flood::Matrix<double>*) method." << std::endl
                << "Wrong Gene Start tag: "<< word  << std::endl;
      exit(1);
   }

   do
   {
      //-- Adding generation_index.
      generation_index->push_back(word);

      for(int j = 0; j < parametersNumber; j++)
      {
         gene_file >> individual[j];
      }
      population->add_row(individual);
      gene_file >> word;
   }while(word != "</Gene>");
}


void FileHandler::load_elite_fitness(std::fstream& fitness_file, std::vector<double>* elite_fitness)
{
  double fitness;

  do
  {
    //--index number (Discarded)
    fitness_file >> fitness;

    //--Best Fitness
    fitness_file >> fitness;
    elite_fitness->push_back(fitness);

    //--Average Fitness (Discarded)
    fitness_file >> fitness;
    //--Worst Fitness (Discarded)
    fitness_file >> fitness;
  }while(!fitness_file.eof());
}


std::string FileHandler::get_file_type()
{
    std::fstream file;

    file.open(myFile_name, std::ios::in);
    if(!file.is_open())
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_file_type(char*)" << std::endl
                << "Cannot open file: "<< myFile_name  << std::endl;
      exit(1);
    }

    std::string line;
    std::string word;
    std::string _file_type;

    getline(file, line);

    if(line != "<Morphomotion: Flood + OpenRave + Y1>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_file_type(char*)" << std::endl
                << "Unknown file declaration: " << line << std::endl;

      exit(1);
    }

    //--File type
    file >> word;

    if(word != "<FileType>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_file_type(char*)" << std::endl
                << "Unknown file type begin tag: " << word << std::endl;

      exit(1);
    }

    file >> _file_type;

    if(_file_type != "GeneFile" && _file_type != "Evolution_Parameters" && _file_type != "EvolutionFile")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_file_type(char*)" << std::endl
                << "Unknown file type: " << word << std::endl;

      exit(1);
    }

    file >> word;

    if(word != "</FileType>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_file_type(char*)" << std::endl
                << "Unknown file type end tag: " << word << std::endl;

      exit(1);
    }

    file.close();
    return _file_type;
}


std::string FileHandler::get_gene_file_name()
{
    std::fstream gene_file;

    gene_file.open(myFile_name, std::ios::in);
    if(!gene_file.is_open())
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_gene_file_name(char*)" << std::endl
                << "Cannot open Parameter file: "<< myFile_name  << std::endl;
      exit(1);
    }

    std::string line;
    std::string word;
    std::string gene_file_name;

    getline(gene_file, line);

    if(line != "<Morphomotion: Flood + OpenRave + Y1>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_gene_file_name(char*)" << std::endl
                << "Unknown file declaration: " << line << std::endl;

      exit(1);
    }

    //--File type
    gene_file >> word;

    if(word != "<FileType>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_gene_file_name(char*)" << std::endl
                << "Unknown file type begin tag: " << word << std::endl;

      exit(1);
    }

    gene_file >> word;

    if(word != "GeneFile")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_gene_file_name(char*)" << std::endl
                << "Unknown file type: " << word << std::endl;

      exit(1);
    }

    gene_file >> word;

    if(word != "</FileType>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_gene_file_name(char*)" << std::endl
                << "Unknown file type end tag: " << word << std::endl;

      exit(1);
    }

    do
    {
      gene_file >> word;

      if(word == "<Gene_File>")
      {
        gene_file >> gene_file_name;
        gene_file >> word;

        if(word != "</Gene_File>")
        {
          std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                    << "get_gene_file_name(char*)" << std::endl
                    << "Unknown Controller Type end tag: " << word << std::endl;

          exit(1);
        }
      }

    }while(word != "</Gene_File>");
    gene_file.close();
    return gene_file_name;
}


std::string FileHandler::get_fitness_file_name()
{
    std::fstream gene_file;

    gene_file.open(myFile_name, std::ios::in);
    if(!gene_file.is_open())
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_fitness_file_name(char*)" << std::endl
                << "Cannot open Parameter file: "<< myFile_name  << std::endl;
      exit(1);
    }

    std::string line;
    std::string word;
    std::string fitness_file_name;

    getline(gene_file, line);

    if(line != "<Morphomotion: Flood + OpenRave + Y1>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_fitness_file_name(char*)" << std::endl
                << "Unknown file declaration: " << line << std::endl;

      exit(1);
    }

    //--File type
    gene_file >> word;

    if(word != "<FileType>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_fitness_file_name(char*)" << std::endl
                << "Unknown file type begin tag: " << word << std::endl;

      exit(1);
    }

    gene_file >> word;

    if(word != "GeneFile")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_fitness_file_name(char*)" << std::endl
                << "Unknown file type: " << word << std::endl;

      exit(1);
    }

    gene_file >> word;

    if(word != "</FileType>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_fitness_file_name(char*)" << std::endl
                << "Unknown file type end tag: " << word << std::endl;

      exit(1);
    }

    do
    {
      gene_file >> word;

      if(word == "<Fitness_File>")
      {
        gene_file >> fitness_file_name;
        gene_file >> word;

        if(word != "</Fitness_File>")
        {
          std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                    << "get_fitness_file_name(char*)" << std::endl
                    << "Unknown Controller Type end tag: " << word << std::endl;

          exit(1);
        }
      }

    }while(word != "</Fitness_File>");
    gene_file.close();
    return fitness_file_name;
}


std::string FileHandler::get_evolution_file_name()
{
    std::fstream gene_file;

    gene_file.open(myFile_name, std::ios::in);
    if(!gene_file.is_open())
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_evolution_file_name(char*)" << std::endl
                << "Cannot open Parameter file: "<< myFile_name << std::endl;
      exit(1);
    }

    std::string line;
    std::string word;
    std::string evolution_file_name;

    getline(gene_file, line);

    if(line != "<Morphomotion: Flood + OpenRave + Y1>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_controller_type(char*)" << std::endl
                << "Unknown file declaration: " << line << std::endl;

      exit(1);
    }

    //--File type
    gene_file >> word;

    if(word != "<FileType>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_evolution_file_name(char*)" << std::endl
                << "Unknown file type begin tag: " << word << std::endl;

      exit(1);
    }

    gene_file >> word;

    if(word != "GeneFile")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_evolution_file_name(char*)" << std::endl
                << "Unknown file type: " << word << std::endl;

      exit(1);
    }

    gene_file >> word;

    if(word != "</FileType>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_evolution_file_name(char*)" << std::endl
                << "Unknown file type end tag: " << word << std::endl;

      exit(1);
    }

    do
    {
      gene_file >> word;

      if(word == "<Evolution_File>")
      {
        gene_file >> evolution_file_name;
        gene_file >> word;

        if(word != "</Evolution_File>")
        {
          std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                    << "get_evolution_file_name(char*)" << std::endl
                    << "Unknown Controller Type end tag: " << word << std::endl;

          exit(1);
        }
      }

    }while(word != "</Evolution_File>");
    gene_file.close();
    return evolution_file_name;
}


std::string FileHandler::get_controller_type(char* gene_filename)
{
    std::fstream gene_file;

    gene_file.open(gene_filename, std::ios::in);
    if(!gene_file.is_open())
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_controller_type(char*)" << std::endl
                << "Cannot open Parameter file: "<< gene_filename  << std::endl;
      exit(1);
    }

    std::string line;
    std::string word;
    std::string controller_type;

    getline(gene_file, line);

    if(line != "<Morphomotion: Flood + OpenRave + Y1>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_controller_type(char*)" << std::endl
                << "Unknown file declaration: " << line << std::endl;

      exit(1);
    }

    //--File type
    gene_file >> word;

    if(word != "<FileType>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_controller_type(char*)" << std::endl
                << "Unknown file type begin tag: " << word << std::endl;

      exit(1);
    }

    gene_file >> word;

    if(word != "GeneFile" && word != "Evolution_Parameters")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_controller_type(char*)" << std::endl
                << "Unknown file type: " << word << std::endl;

      exit(1);
    }

    gene_file >> word;

    if(word != "</FileType>")
    {
      std::cerr << "Morphomotion Error: FileHandler class." << std::endl
                << "get_controller_type(char*)" << std::endl
                << "Unknown file type end tag: " << word << std::endl;

      exit(1);
    }

    do
    {
      gene_file >> word;

      if(word == "<ControllerType>")
      {
        gene_file >> controller_type;
        gene_file >> word;

        if(word != "</ControllerType>")
        {
          std::cerr << "Morphomotion Error: FileHandler Class." << std::endl
                    << "void load_Controller_parameters(std::fstream&, Controller*) method." << std::endl
                    << "Unknown Controller Type end tag: " << word << std::endl;

          exit(1);
        }
      }

    }while(word != "</ControllerType>");
    gene_file.close();
    return controller_type;
}
