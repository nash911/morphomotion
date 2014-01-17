/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   C O N T R O L L E R   C L A S S                                                                            */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

//#include <cmath>
//#include <vector>

#include "Controller.h"


//void changemode(int);
//int  kbhit(void);

using namespace std;

Controller::Controller(void)
{
  //-- Set default parameters.
  set_default();
}


Controller::Controller(Flood::MultilayerPerceptron* mlp_pointer, Robot* pointer_robot_primary)
{
  //-- Set default parameters.
  set_default();

  //-- Multi Layer Perceptron pointer.
  mlp = mlp_pointer;

  //-- Robot pointer.
  robot_primary = pointer_robot_primary;
}


Controller::Controller(Flood::MultilayerPerceptron* mlp_pointer, Robot* pointer_robot_primary, Robot* pointer_robot_secondary)
{
  //-- Set default parameters.
  set_default();

  //-- Multi Layer Perceptron pointer.
  mlp = mlp_pointer;

  //-- Robot pointer.
  robot_primary = pointer_robot_primary;
  robot_secondary= pointer_robot_secondary;
}


// DESTRUCTOR
Controller::~Controller(void)
{
  //-- BUG: This is a problem with the below implementation. Getting a "Invalid pointer" error when freeing ServoFeeback object.
  //-- BUG FIX: Changed 'delete[]' to 'delete'
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    delete servo_feedback[module];
  }
}


void Controller::reset_controller()
{
  for(unsigned int module=0; module<number_of_modules; module++) // Note: this has been moved to void init_local_variables(....)
  {
    servo_feedback[module]->reset_value();
  }
}


void Controller::set_oscillation_analyzer(OscillationAnalyzer_OutputSignal* oscAnlz_pointer)
{
  oscAnlz = oscAnlz_pointer;
}


void Controller::init_controller()
{
  number_of_modules = robot_primary->get_number_of_modules();

  //-- Set the size of the current_servo_angle array based on the number of modules in the configuration
  servo_feedback.resize(number_of_modules);
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    servo_feedback[module] = new ServoFeedback;
  }
}


void Controller::set_default(void)
{
  robot_primary =  NULL;
  robot_secondary = NULL;
  controller_type = Neural_Controller;
  servo_max = 90.0;
  servo_min = -90.0;
  number_of_modules = 2;
  evaluation_period = 50;
  mlp = NULL;
  oscAnlz = NULL;
}


/*void Controller::init_local_variables(Flood::Vector<double> &output,
                                      Flood::Vector<double> &previous_cycle_output,
                                      Flood::Vector<bool> &isFirstStep)
{
  if(start_angle_type == Zero)
  {
    for(unsigned int module=0; module<number_of_modules; module++)
    {
      output[module] = 0;
    }
  }
  else if(start_angle_type == Random)
  {
    for(unsigned int module=0; module<number_of_modules; module++)
    {
      output[module] = calculate_random_uniform(servo_min,servo_max);
    }
  }
  else if(start_angle_type == RandomEqual)
  {
    double random_value = calculate_random_uniform(servo_min,servo_max);
    for(unsigned int module=0; module<number_of_modules; module++)
    {
      output[module] = random_value;
    }
  }
  else if(start_angle_type == Predefined)
  {
    for(unsigned int module=0; module<number_of_modules; module++)
    {
      output[module] = predef_start_angle[module];
    }
  }
  else if(start_angle_type == RunTime)
  {
    for(unsigned int module=0; module<number_of_modules; module++)
    {
       std::cout << std::endl << std::endl << "Enter start angle for module number " << module << ": " << std::endl;
       std::cin >> output[module];
    }
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void init_local_variables() method." << std::endl
              << "Unknown start angle type: " << start_angle_type << "." <<std::endl;
    exit(1);
  }*/

  /*if(controller_type == Simple_Controller || controller_type == Naive_Controller)
  {
    for(unsigned int module=0; module<number_of_modules; module++)
    {
      //-- Setting the motors to a the amplitude value.
      output[module] = oscillator_amplitude + oscillator_offset;
    }
  }

  for(unsigned int module=0; module<number_of_modules; module++)
  {
    previous_cycle_output[module] = 0;
    isFirstStep[module] = true;
  }
}*/


/*bool Controller::run_Controller(const std::string& type, std::stringstream& SS, int memberID, int generation, int eval_no)
{
  // Reset controller.
  reset_controller();

  if(controller_type == Sinusoidal_Controller || controller_type == Sine_Controller)
  {
    load_sine_control_parameters();
  }
  else if(controller_type == Naive_Controller)
  {
    set_oscillator_amplitude(mlp->get_independent_parameter(0));
    set_oscillator_offset(mlp->get_independent_parameter(1));
  }
  else if(controller_type == Simple_Controller)
  {
    set_servo_derivative_threshold(mlp->get_independent_parameter(0));
    set_servo_derivative_epsilon(mlp->get_independent_parameter(1));
    set_oscillator_amplitude(mlp->get_independent_parameter(2));
    set_oscillator_offset(mlp->get_independent_parameter(3));
  }
  else if(controller_type == Neural_Controller)
  {
   // Do Nothing.
  }
  else if(controller_type == Hybrid_Controller)
  {
    set_servo_derivative_threshold(mlp->get_independent_parameter(0));
    set_servo_derivative_epsilon(mlp->get_independent_parameter(1));
  }
  else if(controller_type == Semi_Hybrid_Controller)
  {
    set_servo_derivative_threshold(mlp->get_independent_parameter(0));
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "bool run_Controller(const std::string&, int, int, int) method."
              << "Unknown Controller Type: " << controller_type << std::endl;
    exit(1);
  }

  Flood::Vector<double> output(number_of_modules);
  Flood::Vector<double> previous_cycle_output(number_of_modules);
  Flood::Vector<bool> isFirstStep(number_of_modules);

  // Related to Servo Derivative Resolution -- Based on time dependent Servo Feedback data.
  vector<vector<ServoFeedback*> > servo_feedback_history;
  servo_feedback_history.resize(number_of_modules);


  //-- Initialise local variables.
  init_local_variables(output,
                       previous_cycle_output,
                       isFirstStep);

  double servo_delta = 0;
  double servo_derivative;

  unsigned long previous_read_elapsed_time = 0;
  unsigned long evaluation_elapsed_time = 0;
  unsigned long evaluation_window = (unsigned long)evaluation_period * 1000000; // Converted to microseconds;

//--------------Debugger Relater--------------//
  std::vector<unsigned long> oscillation_time(number_of_modules,0);
  std::vector<int> time_diff_counter(number_of_modules,0);
  //int time_diff_counter[number_of_modules];
  //for(int i=0; i<number_of_modules; i++)
  //{
  //  time_diff_counter[i] = 0;
  //}
//--------------Debugger Relater--------------//

  unsigned int key = 'H';
  changemode(1);

  do
  {
      do
      {
        if(controller_type == Sinusoidal_Controller)
        {
          if(isFirstStep[0])
          {
            // Set Sinusoidal Controller parameters here
            robot_primary->set_sine_controller_parameters(sine_amplitude, sine_offset, sine_phase, sine_frequency);

            isFirstStep[0] = false;
          }

          read_servo_positions_with_time();

          //-- Without the following cleanup of the servo_feedback_history vector, it grows indefinately, and leads to memory-leak.
          for(unsigned int module=0; module<number_of_modules; module++)
          {
            for(unsigned int n=0; n<servo_feedback_history[module].size(); n++)
            {
              delete servo_feedback_history[module][n];
            }
            servo_feedback_history[module].resize(0);
          }
        }
        else if(controller_type == Sine_Controller)
        {
          if(!read_servo_positions_with_time())
          {
              std::cout << "  Communication break down. Redo evaluation." << std::endl;
              robot_primary->reset_comm_link();

              SS << "REDO" << " ";
              return true;
          }

          double t = (double)robot_primary->get_elapsed_evaluation_time()/1000000.0;

          for(unsigned int module=0; module<number_of_modules; module++)
          {
    //---------------------------------------------------------------- Debugger ----------------------------------------------------------/
    #ifdef DEBUGGER
              if(module==0)
              {
                std::cout << "Count: " << evaluation_elapsed_time/1000 << "  Output[" << module << "]: " << output[module] << "  Feedback Self: " << current_servo_angle[module] << "  Feed Back Diff: " << servo_delta << "  Servo Delta: " << servo_derivative;
              }
    #endif
    //---------------------------------------------------------------- Debugger ----------------------------------------------------------/

    //-------------------------------------------------------------- Activity Log --------------------------------------------------------/
    #ifdef ACTIVITY_LOG
              if(type == "evaluation")
              {
                std::cout << generation << " -->  " << evaluation_elapsed_time << ": Previous Output[" << module << "]: " << previous_cycle_output[module] << "  Actual Angle: " << servo_feedback[module]->get_servo_position() << "  Current Output[" << module << "]: " << output[module];
              }
    #endif
    //-------------------------------------------------------------- Activity Log --------------------------------------------------------/

              //-- Storing the output of the previous cycle.
              previous_cycle_output[module] = output[module];

              actuate_with_sine_controller(module, t, output);

              //-- BUG FIX: Fixing the memory-leak bug.
              for(unsigned int n=0; n<servo_feedback_history[module].size(); n++)
              {
                delete servo_feedback_history[module][n];
              }

              servo_feedback_history[module].resize(0);

    #ifdef DEBUGGER
              if(module==0)
              {
                std::cout << "  Next Output " << output[module] << std::endl;
              }
    #endif
    #ifdef ACTIVITY_LOG
              if(type == "evaluation")
              {
                std::cout << "  Next Output[" << module << "]: " << output[module] << std::endl;
              }
    #endif
          }
          actuate_all_modules(output);
        }
        else
        {
          read_servo_positions_with_time();

          for(unsigned int module=0; module<number_of_modules; module++)
          {
            if(isFirstStep[module])
            {
              actuate_module(module, output[module]); // TODO: Not sure if this is needed indeed.
              isFirstStep[module] = false;
            }

            servo_delta = calculate_servo_delta(module, output[module]);
            servo_derivative = calculate_servo_derivative_time(module, servo_feedback_history);

            if(servo_delta < servo_delta_threshold || servo_derivative < servo_derivative_threshold)
            {

    //---------------------------------------------------------------- Debugger ----------------------------------------------------------/
    #ifdef DEBUGGER
              if(module==0)
              {
                std::cout << "Count: " << evaluation_elapsed_time/1000 << "  Output[" << module << "]: " << output[module] << "  Feedback Self: " << current_servo_angle[module] << "  Feed Back Diff: " << servo_delta << "  Servo Delta: " << servo_derivative;
              }
    #endif
    //---------------------------------------------------------------- Debugger ----------------------------------------------------------/

    //-------------------------------------------------------------- Activity Log --------------------------------------------------------/
    #ifdef ACTIVITY_LOG
              if(type == "evaluation")
              {
                std::cout << generation << " -->  " << evaluation_elapsed_time/1000 << ": Previous Output[" << module << "]: " << previous_cycle_output[module] << "  Actual Angle: " << servo_feedback[module]->get_servo_position() << "  Current Output[" << module << "]: " << output[module];
              }
    #endif
    //-------------------------------------------------------------- Activity Log --------------------------------------------------------/

              //-- Storing the output of the previous cycle.
              previous_cycle_output[module] = output[module];

              if(controller_type == Neural_Controller)
              {
                actuate_with_neural_controller(module, output);
              }
              else if(controller_type == Naive_Controller)
              {
                actuate_with_simple_controller(module, output);
              }
              else if(controller_type == Simple_Controller)
              {
                actuate_with_simple_controller(module, output);
              }
              else if(controller_type == Hybrid_Controller)
              {
                actuate_with_hybrid_controller(module, output);
              }
              else if(controller_type == Semi_Hybrid_Controller)
              {
                actuate_with_hybrid_controller(module, output);
              }

              if(oscAnlz)
              {
                // Checking if the previous and the current neural outputs are on opposite directions from value 0.
                if((previous_cycle_output[module] * output[module]) < 0.0) // Here we are assuming that the control signal to each module is always oscillatory. As well as that two subsequent neural outputs are always on opposite directions from the value 0.
                {
                  //oscAnlz->update_oscillation_short_history(module,output[module]); // TODO: Oscillation analyses is performed based on control signal generated by controller output. This should be changed by analysing an oscillation based on the actual feedback from the module.
                  oscAnlz->update_oscillation_short_history(module,servo_feedback[module]->get_servo_position()); // Analysing oscillation based on propreoseptive feedback from the module's actuator.
                }
                else
                {
                }
              }
              else
              {
              }

              //-- BUG FIX: Fixing the memory-leak bug.
              for(unsigned int n=0; n<servo_feedback_history[module].size(); n++)
              {
                delete servo_feedback_history[module][n];
              }

              servo_feedback_history[module].resize(0);

    #ifdef DEBUGGER
              if(module==0)
              {
                std::cout << "  Next Output " << output[module] << "  Time Diff: " << oscillation_time[module]  << "  Counter: " << time_diff_counter[module] << std::endl;
                time_diff_counter[module] = 0;
                oscillation_time[module] = 0
              }
    #endif
    #ifdef ACTIVITY_LOG
              if(type == "evaluation")
              {
                std::cout << "  Next Output[" << module << "]: " << output[module] << "  Time Diff: " << oscillation_time[module] << "  Counter: " << time_diff_counter[module] << std::endl;
                time_diff_counter[module] = 0;
                oscillation_time[module] = 0;
              }
    #endif
            }
            else // This is for Debugger only
            {
              time_diff_counter[module]++;
              oscillation_time[module] = oscillation_time[module] + (evaluation_elapsed_time - previous_read_elapsed_time);
            }
          }
        }

        robot_primary->step(type);
        previous_read_elapsed_time = robot_primary->get_previous_read_evaluation_time();
        evaluation_elapsed_time = robot_primary->get_elapsed_evaluation_time();

        if(robot_secondary != NULL)
        {
          if(robot_secondary->get_robot_environment() == "SimulationOpenRave")
          {
            //-- Stepping through simulation as many time as needed to be in sync with the realtime evaluation of Y1.
            do
            {
              robot_secondary->step(type);
            }while(robot_secondary->get_elapsed_evaluation_time() <= evaluation_elapsed_time);
          }
          else if(robot_secondary->get_robot_type() == "Y1")
          {
            robot_secondary->step(type);
          }
        }

        //-- Record trajectory
        if(oscAnlz && oscAnlz->get_record_trajectory())
        {
          oscAnlz->write_trajectory();
        }
      }while(evaluation_elapsed_time < evaluation_window && !kbhit());

      if(kbhit())
      {
          key = getchar();
      }

      if(key==q || key==Q)
      {
          SS << "CANCEL" << " ";
          return false;
      }
      else if(key==SPACE)
      {
          do
          {
              while(!kbhit());
              key = getchar();

          }while(key!=SPACE);

          key = 'q';

          SS << "REDO" << " ";
          return true;
      }
  }while(evaluation_elapsed_time < evaluation_window && (key != q || key != Q));

  changemode(0);

  if(controller_type == Sinusoidal_Controller)
  {
    //-- Stop Sinusoidal Controller
    robot_primary->stop_sinusoidal_controller();
  }

#ifdef DEBUGGER
  std::cout << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
#endif

  SS << "SUCCESS" << " ";

  return true;
}*/


/*void Controller::actuate_with_sine_controller(const unsigned int module, const double t, Flood::Vector<double>& output)
{
  output[module] = sine_amplitude[module] * sin(2*M_PI*sine_frequency*t + ((sine_phase[module] * M_PI)/180.0)) + sine_offset[module];
}*/


/*void Controller::actuate_with_neural_controller(const unsigned int module, Flood::Vector<double>& output)
{
  Flood::Vector<double> nnOutput(mlp->get_outputs_number());
  Flood::Vector<double> nnInput(mlp->get_inputs_number());

  //-- Following input to the NN is the proprioceptive feedback from the module's actuator, which is scaled down to a value between +/-1.
  nnInput[0] = servo_feedback[module]->get_servo_position()/servo_max;

  //-- Calculate the NN output by passing the inputs
  nnOutput = mlp->calculate_output(nnInput);

  //-- Scale up the output to a value between +/- 90 degrees and save it in "Previous Actuation Value" vector 'output[]'
  output[module] = nnOutput[0]*servo_max;

  //-- Send actuation command to the corrsponding module in the robot configuration.
  actuate_module(module, output[module]);
}*/


/*void Controller::actuate_with_hybrid_controller(const unsigned int module, Flood::Vector<double>& output)
{
  actuate_with_neural_controller(module, output);
}*/


/*void Controller::actuate_with_simple_controller(const unsigned int module, Flood::Vector<double>& output)
{
  output[module] = (-1.0 * (output[module] - oscillator_offset)) + oscillator_offset; // BUG FIX: -(Amplitude) + Offset
  actuate_module(module, output[module]);
}*/


void Controller::actuate_module(const unsigned int module, double output)
{
  //-- Send actuation command to the corrsponding module in the robot configuration.
  robot_primary->set_moduleServo_position(module, output);

  if(robot_secondary != NULL)
  {
    robot_secondary->set_moduleServo_position(module, output);
  }
}


void Controller::actuate_all_modules(const Flood::Vector<double>& output)
{
  std::vector<double> servo_angle;

  for(unsigned int module=0; module<number_of_modules; module++)
  {
      servo_angle.push_back(output[module]);
  }
  //-- Send actuation command to the corrsponding module in the robot configuration.
  robot_primary->set_all_moduleServo_position(servo_angle);

  if(robot_secondary != NULL)
  {
    robot_secondary->set_all_moduleServo_position(servo_angle);
  }
}


bool Controller::read_servo_positions_with_time() // TODO: This should be implemented as a seperate thread.
{
  bool got_position;

  got_position = robot_primary->get_all_moduleServo_position(servo_feedback);
  //robot_primary->get_all_moduleServo_position_with_individual_time(servo_feedback);

/**********************************************************TEMP FIX**************************************************************************/
/*TODO: This is a temprory fix. Need to write servo value along with actual time [X-axis] into graph file.*/

  std::vector<double> servo_positions;
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    servo_positions.push_back(servo_feedback[module]->get_servo_position());
  }
  if(oscAnlz && oscAnlz->get_record_servo())
  {
    oscAnlz->write_servo(servo_positions);
  }
/**********************************************************TEMP FIX**************************************************************************/

  return got_position;
}

double Controller::calculate_servo_delta(const unsigned int module, double last_output)
{
  double servo_delta;

  servo_delta = fabs(last_output-servo_feedback[module]->get_servo_position());
  return(servo_delta);
}


/* Calculating servo derivative based on time info of servo feedback data */
double Controller::calculate_servo_derivative_time(const unsigned int module, vector<vector<ServoFeedback*> > &servo_feedback_history)
{
  double servo_derivative = servo_derivative_threshold + 1.0;

  ServoFeedback *servo_FB_data = NULL;
  servo_FB_data = new ServoFeedback;
  servo_FB_data->set_new_value(servo_feedback[module]->get_servo_position_read_time(), servo_feedback[module]->get_servo_position());
  servo_feedback_history[module].push_back(servo_FB_data);

  unsigned int history_buffer_size = servo_feedback_history[module].size();

  for(unsigned int i=0; i<history_buffer_size; i++)
  {
    if((servo_feedback_history[module][history_buffer_size-1]->get_servo_position_read_time() - servo_feedback_history[module][i]->get_servo_position_read_time()) >= servo_derivative_epsilon)
    {
      servo_derivative = servo_feedback_history[module][history_buffer_size-1]->get_servo_position() - servo_feedback_history[module][i]->get_servo_position();
      servo_derivative = fabs(servo_derivative);
    }
    else
    {
      break; // BUG FIX
    }
  }
  return(servo_derivative);
}


void Controller::set_robot_secondary(Robot* pointer_robot_secondary)
{
  if(pointer_robot_secondary != NULL)
  {
    robot_secondary = pointer_robot_secondary;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_robot_secondary(Robot*) method." << std::endl
              << "Cannot set Robot Secondary to NULL pointer: " << pointer_robot_secondary << "." <<std::endl;
    exit(1);
  }
}


Robot* Controller::get_robot_secondary(void)
{
  return(robot_secondary);
}


void Controller::set_servo_max(double new_servo_max)
{
  if(new_servo_max <= 90.0 && new_servo_max >= -90.0)
  {
    servo_max = new_servo_max;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_servo_max(double) method."
              << "New servo angle is out of range." << std::endl;
    exit(1);
  }
}


double Controller::get_servo_max(void)
{
  return(servo_max);
}


void Controller::set_servo_min(double new_servo_min)
{
  if(new_servo_min <= 90.0 && new_servo_min >= -90.0)
  {
    servo_min = new_servo_min;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_servo_min(double) method."
              << "New servo angle is out of range." << std::endl;
    exit(1);
   }
}


double Controller::get_servo_min(void)
{
  return(servo_min);
}


void Controller::set_evaluation_period(unsigned int new_evaluation_period)
{
  evaluation_period = new_evaluation_period;
}


unsigned int Controller::get_evaluation_period(void)
{
  return (evaluation_period);
}


void Controller::set_start_angle_type(const std::string& new_start_angle_type)
{
  if(new_start_angle_type == "Zero")
  {
    start_angle_type = Zero;
  }
  else if(new_start_angle_type == "Random")
  {
    start_angle_type = Random;
  }
  else if(new_start_angle_type == "RandomEqual")
  {
    start_angle_type = RandomEqual;
  }
  else if(new_start_angle_type == "Predefined")
  {
    start_angle_type = Predefined;
  }
  else if(new_start_angle_type == "RunTime")
  {
    start_angle_type = RunTime;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_start_angle_type(const std::string&) method." << std::endl
              << "Unknown start angle type: " << new_start_angle_type << "." <<std::endl;
    exit(1);
  }
}


std::string Controller::get_start_angle_type(void)
{
  if(start_angle_type == Zero)
  {
    return("Zero");
  }
  else if(start_angle_type == Random)
  {
    return("Random");
  }
  else if(start_angle_type == RandomEqual)
  {
    return("RandomEqual");
  }
  else if(start_angle_type == Predefined)
  {
    return("Predefined");
  }
  else if(start_angle_type == RunTime)
  {
    return("RunTime");
  }
  else
  {
      return("Error");
  }
}


void Controller::set_predef_start_angle_values(Flood::Vector<double> new_predef_start_angle)
{
  if(new_predef_start_angle.get_size() != number_of_modules)
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_predef_start_angle_values(Flood::Vector<double>) method." << std::endl
              << "Size of the vector predef_start_angle " << new_predef_start_angle.get_size() << " does not match the number of modules " << number_of_modules << std::endl;
    exit(1);
  }

  for(unsigned int i=0; i<number_of_modules; i++)
  {
    predef_start_angle[i] = new_predef_start_angle[i];
  }
}


void Controller::set_servo_delta_threshold(double new_servo_delta_threshold)
{
  if(new_servo_delta_threshold > 0)
  {
    servo_delta_threshold = new_servo_delta_threshold;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_servo_delta_threshold(double) method."
              << "Servo Delta Threshold must be greater than zero." << std::endl;
    exit(1);
  }
}


double Controller::get_servo_delta_threshold(void)
{
  return(servo_delta_threshold);
}


void Controller::set_servo_derivative_threshold(double new_servo_derivative_threshold)
{
  if(new_servo_derivative_threshold > 0)
  {
    servo_derivative_threshold = new_servo_derivative_threshold;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_servo_derivative_threshold(double) method."
              << "Servo Derivative Threshold must be greater than zero." << std::endl;
    exit(1);
  }
}


double Controller::get_servo_derivative_threshold(void)
{
  return(servo_derivative_threshold);
}


void Controller::set_servo_derivative_epsilon(unsigned long new_servo_derivative_epsilon)
{
  if(new_servo_derivative_epsilon > 0)
  {
    servo_derivative_epsilon = new_servo_derivative_epsilon;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_servo_derivative_epsilon(unsigned long) method."
              << "Servo Derivative Resolution must be greater than zero: " << new_servo_derivative_epsilon << std::endl;
    exit(1);
  }
}


unsigned long Controller::get_servo_derivative_epsilon(void)
{
  return(servo_derivative_epsilon);
}


void Controller::set_oscillator_amplitude(double new_oscillator_amplitude)
{
  if(new_oscillator_amplitude > 0)
  {
    oscillator_amplitude = new_oscillator_amplitude;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_oscillator_amplitude(double) method."
              << "Oscillator amplitude must be greater than zero. Oscillator amplitude: " << new_oscillator_amplitude << std::endl;
    exit(1);
  }
}


double Controller::get_oscillator_amplitude(void)
{
  return(oscillator_amplitude);
}


void Controller::set_oscillator_offset(double new_oscillator_offset)
{
  if(new_oscillator_offset > -90 && new_oscillator_offset < 90)
  {
    oscillator_offset = new_oscillator_offset;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_oscillator_offset(double) method."
              << "Oscillator offset must be in the range of -90 and +90. Oscillator offset: " << new_oscillator_offset << std::endl;
    exit(1);
   }
}


double Controller::get_oscillator_offset(void)
{
  return(oscillator_offset);
}


void Controller::load_sine_control_parameters()
{
  unsigned int independent_parameter_index = 0;

  //-- Load Amplitude
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    set_sine_amplitude(mlp->get_independent_parameter(independent_parameter_index), module, independent_parameter_index);
    independent_parameter_index++;
  }

  //-- Load Offset
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    set_sine_offset(mlp->get_independent_parameter(independent_parameter_index), module, independent_parameter_index);
    independent_parameter_index++;
  }

  //-- Load Phase
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    set_sine_phase(mlp->get_independent_parameter(independent_parameter_index), module, independent_parameter_index);
    independent_parameter_index++;
  }

  //-- Load Frequency
  set_sine_frequency(mlp->get_independent_parameter(independent_parameter_index), independent_parameter_index);
  independent_parameter_index++;

}

void Controller::set_sine_amplitude(const double new_sine_amplitude, const unsigned int module, const unsigned int independent_parameter_index)
{
  if(new_sine_amplitude > 0 && new_sine_amplitude <= 90)
  {
    sine_amplitude[module] = new_sine_amplitude;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_sine_amplitude(const double, const unsigned int, const unsigned int) method." << std::endl
              << "sine amplitude must be between 0 and 90. Independent parameter Min: " << mlp->get_independent_parameter_minimum(independent_parameter_index) << " Independent parameter Max: " << mlp->get_independent_parameter_maximum(independent_parameter_index) << std::endl
              << "sine amplitude[" << independent_parameter_index%number_of_modules << "]: " << new_sine_amplitude << std::endl;
    exit(1);
  }
}


void Controller::set_sine_offset(const double new_sine_offset, const unsigned int module, const unsigned int independent_parameter_index)
{
  if(new_sine_offset > -90 && new_sine_offset < 90)
  {
    sine_offset[module] = new_sine_offset;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_sine_offset(const double, const unsigned int, const unsigned int) method." << std::endl
              << "sine offset must be between -90 and 90. Independent parameter Min: " << mlp->get_independent_parameter_minimum(independent_parameter_index) << " Independent parameter Max: " << mlp->get_independent_parameter_maximum(independent_parameter_index) << std::endl
              << "sine offset[" << independent_parameter_index%number_of_modules << "]: " << new_sine_offset << std::endl;
    exit(1);
  }
}


void Controller::set_sine_phase(const double new_sine_phase, const unsigned int module, const unsigned int independent_parameter_index)
{
  if(new_sine_phase > mlp->get_independent_parameter_minimum(independent_parameter_index) && new_sine_phase < mlp->get_independent_parameter_maximum(independent_parameter_index))
  {
    sine_phase[module] = new_sine_phase;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_sine_phase(const double, const unsigned int, const unsigned int) method." << std::endl
              << "sine phase must be between Independent parameter Min: " << mlp->get_independent_parameter_minimum(independent_parameter_index) << " and Independent parameter Max: " << mlp->get_independent_parameter_maximum(independent_parameter_index) << std::endl
              << "sine phase[" << independent_parameter_index%number_of_modules << "]: " << new_sine_phase << std::endl;
    exit(1);
  }
}


void Controller::set_sine_frequency(const double new_sine_frequency, const unsigned int independent_parameter_index)
{
  if(new_sine_frequency > mlp->get_independent_parameter_minimum(independent_parameter_index) && new_sine_frequency < mlp->get_independent_parameter_maximum(independent_parameter_index))
  {
    sine_frequency = new_sine_frequency;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_sine_frequency(const double, const unsigned int) method." << std::endl
              << "sine frequency must be between Independent parameter Min: " << mlp->get_independent_parameter_minimum(independent_parameter_index) << " and Independent parameter Max: " << mlp->get_independent_parameter_maximum(independent_parameter_index) << std::endl
              << "sine frequency: " << new_sine_frequency << std::endl;
    exit(1);
   }
}


double Controller::get_sine_amplitude(const unsigned int module)
{
    if(module < number_of_modules)
    {
        return sine_amplitude[module];
    }
    else
    {
        std::cerr << "Morphomotion Error: Controller class." << std::endl
                  << "double get_sine_amplitude(const unsigned int method." << std::endl
                  << "Module: " << module << " should be less than or etual to number of modules: " << number_of_modules << std::endl;
        exit(1);
    }
}


double Controller::get_sine_offset(const unsigned int module)
{
    if(module < number_of_modules)
    {
        return sine_offset[module];
    }
    else
    {
        std::cerr << "Morphomotion Error: Controller class." << std::endl
                  << "double get_sine_offset(const unsigned int method." << std::endl
                  << "Module: " << module << " should be less than or etual to number of modules: " << number_of_modules << std::endl;
        exit(1);
    }
}


double Controller::get_sine_phase(const unsigned int module)
{
    if(module < number_of_modules)
    {
        return sine_phase[module];
    }
    else
    {
        std::cerr << "Morphomotion Error: Controller class." << std::endl
                  << "double get_sine_phase(const unsigned int method." << std::endl
                  << "Module: " << module << " should be less than or etual to number of modules: " << number_of_modules << std::endl;
        exit(1);
    }
}


double Controller::get_sine_frequency()
{
    return sine_frequency;
}


void Controller::set_controller_type(const std::string& new_controller_type)
{
  if(new_controller_type == "Neural_Controller")
  {
    controller_type = Neural_Controller;
  }
  else if(new_controller_type == "Naive_Controller")
  {
    controller_type = Naive_Controller;
  }
  else if(new_controller_type == "Simple_Controller")
  {
    controller_type = Simple_Controller;
  }
  else if(new_controller_type == "Sine_Controller")
  {
    controller_type = Sine_Controller;
  }
  else if(new_controller_type == "Hybrid_Controller")
  {
    controller_type = Hybrid_Controller;
  }
  else if(new_controller_type == "Semi_Hybrid_Controller")
  {
    controller_type = Semi_Hybrid_Controller;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_controller_type(const std::string&) method." << std::endl
              << "Unknown ControllerType: " << new_controller_type << "." <<std::endl;
    exit(1);
  }
}


std::string Controller::get_controller_type(void)
{
  switch(controller_type)
  {
    case Neural_Controller:
    {
      return("Neural_Controller");
    }
    case Naive_Controller:
    {
      return("Naive_Controller");
    }
    case Simple_Controller:
    {
      return("Simple_Controller");
    }
    case Sine_Controller:
    {
      return("Sine_Controller");
    }
    case Hybrid_Controller:
    {
      return("Hybrid_Controller");
    }
    case Semi_Hybrid_Controller:
    {
      return("Semi_Hybrid_Controller");
    }
    default:
    {
      std::cerr << "Morphomotion Error: Controller class." << std::endl
                << "std::string get_controller_type(void) method." << std::endl
                << "Unknown ControllerType: " << std::endl;
      exit(1);
    }
  }
}


double Controller::calculate_random_uniform(double minimum, double maximum)
{
  double random = (double)rand()/(RAND_MAX+1.0);

  double random_uniform = minimum + (maximum-minimum)*random;

  return(random_uniform);
}


double Controller::calculate_random_normal(double mean, double standard_deviation)
{
  double random_normal = 0.0;

  const double pi = 4.0*atan(1.0);

  double random_uniform_1;
  double random_uniform_2;

  do
  {
    random_uniform_1 = (double)rand()/(RAND_MAX+1.0);

  }while(random_uniform_1 == 0.0);

  random_uniform_2 = (double)rand()/(RAND_MAX+1.0);

  // Box-Muller transformation

  random_normal = mean + sqrt(-2.0*log(random_uniform_1))*sin(2.0*pi*random_uniform_2)*standard_deviation;

  return(random_normal);
}


double Controller::scale_to_range(double from_min, double from_max, double to_min, double to_max, double input)
{
  double output;
  double delta_to_max_min = to_max - to_min;
  double delta_from_max_min = from_max - from_min;

  output = (delta_to_max_min/2) + to_min + (input/(delta_from_max_min/delta_to_max_min));

  std::cout << "   Scaled Output: " << output; // Debugger;

  return output;
}


void Controller::changemode(int dir)
{
  static struct termios oldt, newt;

  if ( dir == 1 )
  {
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  }
  else
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}


int Controller::kbhit (void)
{
  struct timeval tv;
  fd_set rdfs;

  tv.tv_sec = 0;
  tv.tv_usec = 0;

  FD_ZERO(&rdfs);
  FD_SET (STDIN_FILENO, &rdfs);

  select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &rdfs);

}
