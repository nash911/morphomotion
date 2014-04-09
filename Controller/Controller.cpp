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

#include "Controller.h"

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
    std::cout << std::endl << "Deleted servo_feedback[" << module << "]" << std::endl;
  }
}


void Controller::reset_controller()
{
  for(unsigned int module=0; module<number_of_modules; module++) // Note: this has been moved to void init_local_variables(....)
  {
    servo_feedback[module]->reset_value();
    servo_feedback[module]->get_ExtKalmanFilter()->reset_parameters();
  }
  //oscAnlz = NULL;
}


void Controller::set_oscillation_analyzer(OscillationAnalyzer_OutputSignal* oscAnlz_pointer)
{
  oscAnlz = oscAnlz_pointer;
}


void Controller::init_controller(const double delta_time)
{
  number_of_modules = robot_primary->get_number_of_modules();

  //-- Set the size of the current_servo_angle array based on the number of modules in the configuration
  servo_feedback.resize(number_of_modules);
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    servo_feedback[module] = new ServoFeedback(delta_time);
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

  EKF_dt = 0.01;
  EKF_r = 0.1;
  EKF_qf = 0.0001;
}


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

/**********************************************************TEMP FIX**************************************************************************/
/*TODO: This is a temprory fix. Need to write servo value along with actual time [X-axis] into graph file.*/

  if(oscAnlz && oscAnlz->get_record_servo())
  {
    std::vector<double> servo_positions;
    for(unsigned int module=0; module<number_of_modules; module++)
    {
      servo_positions.push_back(servo_feedback[module]->get_servo_position());
    }
    oscAnlz->write_servo(servo_positions);
  }
/**********************************************************TEMP FIX**************************************************************************/

  return got_position;
}


void Controller::read_servo_positions_with_time_THREAD()
{
  robot_primary->get_all_moduleServo_position(servo_feedback);
  //std::cout << "Returning from read_servo_positions_with_time_THREAD()." << std::endl; //--TODO: Debugger to be removed.
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


double Controller::sine_wave(double amplitude, double offset, double frequency, double phase, double time)
{
    return(amplitude * sin(2*M_PI*frequency*time + ((phase * M_PI)/180.0)) + offset);
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


void Controller::set_oscillator_frequency(double new_oscillator_frequency)
{
  if(new_oscillator_frequency > 0)
  {
    oscillator_frequency = new_oscillator_frequency;
  }
  else
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_oscillator_frequency(double) method."
              << "Oscillator frequency must be greater than 0. Oscillator frequency: " << new_oscillator_frequency << std::endl;
    exit(1);
   }
}


double Controller::get_oscillator_frequency(void)
{
  return(oscillator_frequency);
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
  if(new_sine_amplitude >= 0 && new_sine_amplitude <= 90)
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
  else if(new_controller_type == "InverseSine_Controller")
  {
    controller_type = InverseSine_Controller;
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
    case InverseSine_Controller:
    {
      return("InverseSine_Controller");
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


void Controller::set_EKF_dt(const double delta_time)
{
    if(delta_time <= 0)
    {
        std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void set_EKF_dt(const double) method." << std::endl
              << "Invalide delta_time: " << delta_time << std::endl;
        exit(1);
    }
    else
    {
        EKF_dt = delta_time;
    }
}


double Controller::get_EKF_dt(void)
{
  return(EKF_dt);
}


void Controller::set_EKF_r(const double new_r)
{
    EKF_r = new_r;
}


double Controller::get_EKF_r(void)
{
  return(EKF_r);
}


void Controller::set_EKF_qf(const double new_qf)
{
    EKF_qf = new_qf;
}


double Controller::get_EKF_qf(void)
{
  return(EKF_qf);
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
