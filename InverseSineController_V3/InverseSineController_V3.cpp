/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   I N V E R S E S I N E   C O N T R O L L E R   V 3   C L A S S                                              */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#include "InverseSineController_V3.h"

InverseSineController_V3::InverseSineController_V3(void):Controller()
{
}


InverseSineController_V3::InverseSineController_V3(Flood::MultilayerPerceptron* mlp_pointer, Robot* pointer_robot_primary):Controller(mlp_pointer, pointer_robot_primary)
{
}


InverseSineController_V3::InverseSineController_V3(Flood::MultilayerPerceptron* mlp_pointer, Robot* pointer_robot_primary, Robot* pointer_robot_secondary):Controller(mlp_pointer, pointer_robot_primary, pointer_robot_secondary)
{
}


void InverseSineController_V3::init_controller(const double delta_time)
{
  Controller::init_controller(delta_time);

  tPrime.resize(number_of_modules);
  iteration_start_time.resize(number_of_modules);
  servo_delta.reserve(number_of_modules);
}


void InverseSineController_V3::set_default(void)
{
  Controller::set_default();

  oscillator_amplitude = 0;
  oscillator_offset = 0;

  for(unsigned int module=0; module<number_of_modules; module++)
  {
    servo_delta[module] = 0.0;
  }
}


void InverseSineController_V3::init_local_variables(Flood::Vector<double> &output,
                                      Flood::Vector<double> &previous_cycle_output)
{
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    tPrime[module] = 0;
    iteration_start_time[module] = robot_primary->get_elapsed_evaluation_time();
    output[module] = 0;
    previous_cycle_output[module] = 0;
  }
}


void InverseSineController_V3::start_Controller(const std::string& type, std::stringstream& SS, int generation)
{
  //--Reset controller.
  reset_controller();

  robot_primary->set_receive_broadcast(true);

  std::thread ctrl(&InverseSineController_V3::run_Controller, this, type, std::ref(SS), generation);
  std::thread read_broadcast(&InverseSineController_V3::read_servo_positions_with_time_THREAD, this);

  ctrl.join();
  read_broadcast.join();
}


void InverseSineController_V3::run_Controller(const std::string& type, std::stringstream& SS, int generation)
{
  load_InverseSineController_V3_control_parameters();

  if(validate_amplitude_plus_offset(get_oscillator_amplitude(), get_oscillator_offset()))
  {
    SS << "INVALID_AMP+OFF" << " ";

    robot_primary->set_processing_flag(false);
    robot_primary->set_receive_broadcast(false);
    while(robot_primary->get_broadcast_thread());
    return;
  }

  for(unsigned int module=0; module<number_of_modules; module++)
  {
    if(servo_feedback[module]->get_ExtKalmanFilter() != NULL)
    {
      servo_feedback[module]->get_ExtKalmanFilter()->set_omega(oscillator_frequency);
      if(robot_primary->get_robot_environment() == "Y1")
      {
        servo_feedback[module]->get_ExtKalmanFilter()->set_dt(get_EKF_dt());
      }
      else
      {
        //--The 'dt` parameter for EKF is as set in the main function [In either EvolveController.cpp or EvaluateController.cpp]
      }
      servo_feedback[module]->get_ExtKalmanFilter()->set_r(get_EKF_r());
      servo_feedback[module]->get_ExtKalmanFilter()->set_qf(get_EKF_qf());
    }
  }

  Flood::Vector<double> output(number_of_modules);
  Flood::Vector<double> previous_cycle_output(number_of_modules);

  // Related to Servo Derivative Resolution -- Based on time dependent Servo Feedback data.
  vector<vector<ServoFeedback*> > servo_feedback_history;
  servo_feedback_history.resize(number_of_modules);

  //-- Initialise local variables.
  init_local_variables(output, previous_cycle_output);

  double actual_diff;
  double current_servo_angle;
  //double t;

  unsigned long previous_read_elapsed_time = 0;
  unsigned long evaluation_elapsed_time = 0;
  unsigned long evaluation_window = (unsigned long)evaluation_period * 1000000; // Converted to microseconds;

//--------------Debugger Relater--------------//
  std::vector<unsigned long> oscillation_time(number_of_modules,0);
  std::vector<int> time_diff_counter(number_of_modules,0);
//--------------Debugger Relater--------------//

  unsigned int key = 'H';
  changemode(1);

  do
  {
      do
      {
        if(oscAnlz)
        {
            //--Record reference position.
            if(oscAnlz->get_record_ref())
            {
              oscAnlz->write_ref(output);
            }

            //--Record current Filtered position.
            if(oscAnlz->get_record_servo())
            {
              std::vector<double> servo_positions;
              for(unsigned int module=0; module<number_of_modules; module++)
              {
                servo_positions.push_back(servo_feedback[module]->get_servo_position());
              }
              oscAnlz->write_servo(servo_positions);
            }

            //--Record current Raw position.
            if(oscAnlz->get_record_servo_raw())
            {
              std::vector<double> servo_raw_positions;
              for(unsigned int module=0; module<number_of_modules; module++)
              {
                servo_raw_positions.push_back(servo_feedback[module]->get_servo_raw_position());
              }
              oscAnlz->write_servo_raw(servo_raw_positions);
            }
        }

        for(unsigned int module=0; module<number_of_modules; module++)
        {
          current_servo_angle = servo_feedback[module]->get_servo_position();

          actual_diff = fabs(previous_cycle_output[module] - current_servo_angle);

          if(actual_diff > servo_delta[module])
          {

    //---------------------------------------------------------------- Debugger ----------------------------------------------------------/
    #ifdef DEBUGGER
            if(module==0)
            {
                std::cout << "Count: " << (double)evaluation_elapsed_time/1000000 << "  Module[" << module << "]: y(t'):" previous_cycle_output[module] << "  θ: " << current_servo_angle << "  y(t')-θ: " << actual_diff;
            }
    #endif
    //---------------------------------------------------------------- Debugger ----------------------------------------------------------/

    //-------------------------------------------------------------- Activity Log --------------------------------------------------------/
    #ifdef ACTIVITY_LOG
            if(type == "evaluation")
            {
              std::cout << std::endl << generation << " -->  " << (double)evaluation_elapsed_time/1000000.0 << ": Module[" << module << "]: y(t'):" << previous_cycle_output[module] << "  θ: " << current_servo_angle;
            }
    #endif
    //-------------------------------------------------------------- Activity Log --------------------------------------------------------/

            update_tPrime(module);

            /*if(oscAnlz)
            {
              //-- Checking if the previous and the current neural outputs are on opposite directions from value 0.
              if((previous_cycle_output[module] * Yi[module]) < 0.0) // Here we are assuming that the control signal to each module is always oscillatory. As well as that two subsequent neural outputs are always on opposite directions from the value 0.
              {
                oscAnlz->update_oscillation_short_history(module,servo_feedback[module]->get_servo_position()); // Analysing oscillation based on propreoseptive feedback from the module's actuator.
              }
            }*/

            for(unsigned int n=0; n<servo_feedback_history[module].size(); n++)
            {
              delete servo_feedback_history[module][n];
            }
            servo_feedback_history[module].resize(0);

    #ifdef DEBUGGER
            if(module==0)
            {
                std::cout << "  Next Output " << sin_wave(oscillator_amplitude, oscillator_offset, oscillator_frequency, 0, get_t(module))
                          << "  Time Diff: " << oscillation_time[module]  << "  Counter: "
                          << time_diff_counter[module] << std::endl;
              time_diff_counter[module] = 0;
              oscillation_time[module] = 0;
            }
    #endif
    #ifdef ACTIVITY_LOG
            if(type == "evaluation")
            {
              std::cout << "  Next Output[" << module << "]: " << sin_wave(oscillator_amplitude, oscillator_offset, oscillator_frequency, 0, get_t(module))
                        << "  Time Diff: " << oscillation_time[module] << "  Counter: "
                        << time_diff_counter[module] << std::endl;

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

          actuate_with_inverse_sine_controller(module, get_t(module), output);

          //-- Storing the output of the previous cycle.
          previous_cycle_output[module] = output[module];
        }

        actuate_all_modules(output);

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
      }while(evaluation_elapsed_time < evaluation_window && !kbhit() && robot_primary->get_receive_broadcast());

      if(kbhit())
      {
          key = getchar();
      }

      if(key==q || key==Q)
      {
          SS << "CANCEL" << " ";

          robot_primary->set_processing_flag(false);
          robot_primary->set_receive_broadcast(false);
          while(robot_primary->get_broadcast_thread());
          return;
      }
      else if(key==p || key==P)
      {
        SS << "PREVIOUS" << " ";

        robot_primary->set_processing_flag(false);
        robot_primary->set_receive_broadcast(false);
        while(robot_primary->get_broadcast_thread());
        return;
      }
      else if(key==SPACE)
      {
          robot_primary->set_processing_flag(false);
          robot_primary->set_receive_broadcast(false);
          while(robot_primary->get_broadcast_thread());

          std::cout << std::endl << "             Paused: Waiting for SPACE key...." << std::endl;
          do
          {
              while(!kbhit());
              key = getchar();

          }while(key!=SPACE);

          key = 'q';
          SS << "REDO" << " ";

          std::cout << std::endl << "                       Continuing...." << std::endl;
          return;
      }
  }while(evaluation_elapsed_time < evaluation_window && (key != q || key != Q)  && robot_primary->get_receive_broadcast());
  changemode(0);

#ifdef DEBUGGER
  std::cout << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
#endif

  SS << "SUCCESS" << " ";

  robot_primary->set_processing_flag(false);
  robot_primary->set_receive_broadcast(false);
  while(robot_primary->get_broadcast_thread());
  return;
}


void InverseSineController_V3::actuate_with_inverse_sine_controller(const unsigned int module, const double tPrime, Flood::Vector<double>& output)
{
  output[module] = sin_wave(oscillator_amplitude, oscillator_offset, oscillator_frequency, 0, tPrime);

  //--Make sure that the next control signal is a value between the servo range.
  if(output[module] > get_servo_max())
  {
    std::cerr << "Morphomotion Error: InverseSineController class." << std::endl
              << "void actuate_with_inverse_sine_controller(const unsigned int, const double, Flood::Vector<double>&) method."
              << "y(t'): " << output[module] << " > Servo_Max: " << get_servo_max() << std::endl;
    exit(1);
  }
  else if(output[module] < get_servo_min())
  {
    std::cerr << "Morphomotion Error: InverseSineController class." << std::endl
              << "void actuate_with_inverse_sine_controller(const unsigned int, const double, Flood::Vector<double>&) method."
              << "y(t'): " << output[module] << " < Servo_Min: " << get_servo_min() << std::endl;
    exit(1);
  }
}


void InverseSineController_V3::update_tPrime(unsigned int module)
{
  double current_servo_angle = servo_feedback[module]->get_servo_position();
  double Theta;
  double V;
  double tHat;

  //--Make sure that Theta is between the range of Amplitude+Offset > Theta > -Amplitude+Offset.
  if(current_servo_angle <= oscillator_amplitude+oscillator_offset && current_servo_angle >= ((-1.0)*oscillator_amplitude)+oscillator_offset)
  {
    Theta = current_servo_angle;
  }
  else if(current_servo_angle >= oscillator_amplitude+oscillator_offset)
  {
    Theta = oscillator_amplitude+oscillator_offset;
  }
  else if(current_servo_angle <= ((-1.0)*oscillator_amplitude)+oscillator_offset)
  {
    Theta = ((-1.0)*oscillator_amplitude)+oscillator_offset;
  }
  else
  {
    std::cerr << "Morphomotion Error: InverseSineController class." << std::endl
              << "void update_tPrime_and_Yi(unsigned int) method."
              << "Theta uninitialized. current_servo_angle = " << current_servo_angle << std::endl;
    exit(1);
  }

  V = getCurrentVelocityDirection(current_servo_angle); //TODO: A very primitive way of doing this. This needs to be changed.

  //--Calculate the Time Shift factor.
  tPrime[module] = calculate_tPrime(Theta, V);

  //--Reset iteration start time.
  iteration_start_time[module] = robot_primary->get_elapsed_evaluation_time();

  return;
}


double InverseSineController_V3::calculate_tPrime(double Theta, double V)
{
  //--Eq: t'(V,θ) = (1/2f) + [1/2f * (1-(V/|V|))] - y^-1(θ,V)

  //--               1    |‾  1    |‾ 1 - V   ‾| ‾|
  //--Eq: t'(V,θ) = --- + |  --- x |     ---  |  | - y^-1(θ,V)
  //--              2f    |  2f    |_    |V| _|  |
  //--                    |_                    _|

  double tPrime;
  tPrime = (1.0/(2.0*oscillator_frequency)) + ((1.0/(2.0*oscillator_frequency)) * (1 - (V/fabs(V)))) - inverse_sinewave_function(Theta, V);

  return tPrime;
}


double InverseSineController_V3::inverse_sinewave_function(const double Theta, const double V)
{
  //--               |‾                                                              ‾|
  //--               |1-(V/|V|)   1-(θ/|θ|) * 1+(V/|V|)   |‾         Sin^-1(θ-o/A) ‾| |     1
  //--Eg: y^-1(θ,V) =|--------- + --------------------- + |(V/|V|) x -------------  | |  x ---
  //--               |    4                 4             |_               2π      _| |     f
  //--               |_                                                              _|

  //invSineWave = ((1-(V/fabs(V)))/4 + ((1-(theta/fabs(theta))) * (1+(V/fabs(V))))/4 + ((V/fabs(V)) * (asin((theta-o[actuator])/A[actuator])/(2*Pi)))) * (1/f[actuator]);

  double invSineWave;
  double X;
  double Y;
  double Z;

  X = (1-(V/fabs(V)))/4;
  Y = ((1-(Theta/fabs(Theta))) * (1+(V/fabs(V))))/4;
  Z = ((V/fabs(V)) * (asin(float((Theta-oscillator_offset)/oscillator_amplitude))/(2*M_PI))); //-- BUGFIX: asin(long double) method converts double to long double. Fixed it by type casting the passed parameter to (float).

  invSineWave = (X + Y + Z) * (1/oscillator_frequency);

  return invSineWave;
}


double InverseSineController_V3::getCurrentVelocityDirection(double current_servo_position)
{
  if(current_servo_position > 0)
  {
    return(1.0);
  }
  else if(current_servo_position <= 0)
  {
    return(-1.0);
  }
}


double InverseSineController_V3::get_t(const unsigned int module)
{
  double t;

  t = (double)(robot_primary->get_elapsed_evaluation_time() - iteration_start_time[module])/1000000.0;
  t = t + tPrime[module];

  return t;
}

void InverseSineController_V3::load_InverseSineController_V3_control_parameters()
{
  if(mlp->get_independent_parameters_number() != (number_of_modules + 3))
  {
    std::cerr << "Morphomotion Error: Controller class." << std::endl
              << "void load_InverseSineController_V3_control_parameters(void) method." << std::endl
              << "No. of MLP Independent Parameters != Total No. of Controller Parameters." << std::endl
              << "MLP Independent Parameters: " << mlp->get_independent_parameters_number()
              << "  Controller Parameters: " << (number_of_modules + 3) <<  std::endl;
    exit(1);
  }

  unsigned int independent_parameter_index = 0;

  //-- Load Servo_Delta
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    set_servo_delta(mlp->get_independent_parameter(independent_parameter_index), module);
    independent_parameter_index++;
  }

  //-- Load Amplitude
  set_oscillator_amplitude(mlp->get_independent_parameter(independent_parameter_index));
  independent_parameter_index++;

  //-- Load Offset
  set_oscillator_offset(mlp->get_independent_parameter(independent_parameter_index));
  independent_parameter_index++;

  //-- Load Frequency
  set_oscillator_frequency(mlp->get_independent_parameter(independent_parameter_index));
  independent_parameter_index++;
}
