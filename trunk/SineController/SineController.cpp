/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   S I N E   C O N T R O L L E R   C L A S S                                                                  */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#include "SineController.h"

SineController::SineController(void):Controller()
{
}


SineController::SineController(Flood::MultilayerPerceptron* mlp_pointer, Robot* pointer_robot_primary):Controller(mlp_pointer, pointer_robot_primary)
{
}


SineController::SineController(Flood::MultilayerPerceptron* mlp_pointer, Robot* pointer_robot_primary, Robot* pointer_robot_secondary):Controller(mlp_pointer, pointer_robot_primary, pointer_robot_secondary)
{
}


void SineController::init_controller(const double delta_time)
{
  Controller::init_controller(delta_time);

  //-- Set the size of Sinusoidal Controller parameters vector
  sine_amplitude.resize(number_of_modules);
  sine_offset.resize(number_of_modules);
  sine_phase.resize(number_of_modules);
}


void SineController::set_default(void)
{
  Controller::set_default();
}


//void SineController::init_local_variables(Flood::Vector<double> &previous_cycle_output, Flood::Vector<bool> &isFirstStep)
void SineController::init_local_variables(Flood::Vector<double> &previous_cycle_output)
{
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    previous_cycle_output[module] = 0;
    //isFirstStep[module] = true;
  }
}


void SineController::start_Controller(const std::string& type, std::stringstream& SS, int generation) //--Thread Change
{
  //--Reset controller.
  reset_controller();

  robot_primary->set_receive_broadcast(true);

  std::thread ctrl(&SineController::run_Controller, this, type, std::ref(SS), generation);
  std::thread read_broadcast(&SineController::read_servo_positions_with_time_THREAD, this);

  ctrl.join();
  read_broadcast.join();
}



void SineController::run_Controller(const std::string& type, std::stringstream& SS, int generation)
{
  //Reset controller.
  //reset_controller(); //--Thread Change

  load_sine_control_parameters();

  for(unsigned int module=0; module<number_of_modules; module++)
  {
    if(servo_feedback[module]->get_ExtKalmanFilter() != NULL)
    {
      servo_feedback[module]->get_ExtKalmanFilter()->set_omega(sine_frequency);
      if(robot_primary->get_robot_environment() == "Y1")
      {
        servo_feedback[module]->get_ExtKalmanFilter()->set_dt(get_EKF_dt());
      }
      else
      {
        //--The 'dt` parameter for EKF is as set in the main function [In either EvolveController.cpp or EvaluateController.cpp]
      }
      servo_feedback[module]->get_ExtKalmanFilter()->set_r(get_EKF_r());
      servo_feedback[module]->get_ExtKalmanFilter()->set_qf(0.000001); //--For estimating ICF.
      //servo_feedback[module]->get_ExtKalmanFilter()->set_qf(get_EKF_qf());
    }
  }

  Flood::Vector<double> output(number_of_modules);
  Flood::Vector<double> previous_cycle_output(number_of_modules);

  // Related to Servo Derivative Resolution -- Based on time dependent Servo Feedback data.
  vector<vector<ServoFeedback*> > servo_feedback_history;
  servo_feedback_history.resize(number_of_modules);


  //-- Initialise local variables.
  init_local_variables(previous_cycle_output);

  double t=0;

  unsigned long previous_read_elapsed_time = 0;
  unsigned long evaluation_elapsed_time = 0;
  unsigned long evaluation_window = (unsigned long)evaluation_period * 1000000; // Converted to microseconds;

  unsigned int key = 'H';
  changemode(1);

  do
  {
      do
      {
        /*if(!read_servo_positions_with_time())
        {
            std::cout << "  Communication break down. Redo evaluation." << std::endl;
            robot_primary->reset_comm_link();

            SS << "REDO" << " ";
            return;
        }*/ //--Thread Change

        if(oscAnlz)
        {
            //--Record reference position.
            if(oscAnlz->get_record_ref())
            {
              oscAnlz->write_ref(output);
            }

            //--Record current Filtered position.
            if(oscAnlz->get_record_servo()) //--Thread Change
            {
              std::vector<double> servo_positions;
              for(unsigned int module=0; module<number_of_modules; module++)
              {
                servo_positions.push_back(servo_feedback[module]->get_servo_position());
              }
              oscAnlz->write_servo(servo_positions);
            }

            //--Record current Raw position.
            if(oscAnlz->get_record_servo_raw()) //--Thread Change
            {
              std::vector<double> servo_raw_positions;
              for(unsigned int module=0; module<number_of_modules; module++)
              {
                servo_raw_positions.push_back(servo_feedback[module]->get_servo_raw_position());
              }
              oscAnlz->write_servo_raw(servo_raw_positions);
            }
        }

        t = (double)robot_primary->get_elapsed_evaluation_time()/1000000.0;

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
            //if(type == "evaluation" && module == 0)
            {
              std::cout << generation << " -->  " << evaluation_elapsed_time << " : Previous Output[" << module << "]: " << previous_cycle_output[module] << "  Actual Angle: " << servo_feedback[module]->get_servo_position() << "  Current Output[" << module << "]: " << output[module];
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
            //if(type == "evaluation" && module == 0)
            {
              std::cout << "  Next Output[" << module << "]: " << output[module] << std::endl;
            }
#endif
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

        //std::cout << "Exiting the NESTED While() Loop." << std::endl; //--TODO: Debugger to be removed.
      }while(evaluation_elapsed_time < evaluation_window && !kbhit() && robot_primary->get_receive_broadcast());  //--Thread Change

      if(kbhit())
      {
        key = getchar();
      }

      if(key==q || key==Q)
      {
        //SS.flush(); //--Thread Change
        SS << "CANCEL" << " ";

        robot_primary->set_processing_flag(false); //--Thread Change
        robot_primary->set_receive_broadcast(false); //--Thread Change
        while(robot_primary->get_broadcast_thread());
        return;
      }
      else if(key==SPACE)
      {
          robot_primary->set_processing_flag(false); //--Thread Change
          robot_primary->set_receive_broadcast(false); //--Thread Change
          while(robot_primary->get_broadcast_thread());

          do
          {
              while(!kbhit());
              key = getchar();
              //std::cout << "Waiting for SPACE key." << std::endl;
          }while(key!=SPACE);

          key = 'q';

          //SS.flush(); //--Thread Change
          SS << "REDO" << " ";

          return;
      }
      //std::cout << "Exiting the MAIN While()." << std::endl; //--TODO: Debugger to be removed.
  }while(evaluation_elapsed_time < evaluation_window && (key != q || key != Q)  && robot_primary->get_receive_broadcast());  //--Thread Change
  changemode(0);

#ifdef DEBUGGER
  std::cout << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
#endif

  //SS.flush(); //--Thread Change
  SS << "SUCCESS" << " ";

  robot_primary->set_processing_flag(false); //--Thread Change
  robot_primary->set_receive_broadcast(false); //--Thread Change
  while(robot_primary->get_broadcast_thread()); //--Thread Change
  return;
}


void SineController::actuate_with_sine_controller(const unsigned int module, const double t, Flood::Vector<double>& output)
{
  output[module] = sin_wave(sine_amplitude[module], sine_offset[module], sine_frequency, sine_phase[module], t);

  //--Make sure that the next control signal is a value between the servo range.
  if(output[module] > get_servo_max())
  {
    output[module] = get_servo_max();
  }
  else if(output[module] < get_servo_min())
  {
    output[module] = get_servo_min();
  }
}
