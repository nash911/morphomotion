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


void SineController::init_controller()
{
  Controller::init_controller();

  //-- Set the size of Sinusoidal Controller parameters vector
  sine_amplitude.resize(number_of_modules);
  sine_offset.resize(number_of_modules);
  sine_phase.resize(number_of_modules);
}


void SineController::set_default(void)
{
  Controller::set_default();
}


void SineController::init_local_variables(Flood::Vector<double> &previous_cycle_output, Flood::Vector<bool> &isFirstStep)
{
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    previous_cycle_output[module] = 0;
    isFirstStep[module] = true;
  }
}


bool SineController::run_Controller(const std::string& type, std::stringstream& SS, int memberID, int generation, int eval_no)
{
  // Reset controller.
  reset_controller();

  load_sine_control_parameters();

  Flood::Vector<double> output(number_of_modules);
  Flood::Vector<double> previous_cycle_output(number_of_modules);
  Flood::Vector<bool> isFirstStep(number_of_modules);

  // Related to Servo Derivative Resolution -- Based on time dependent Servo Feedback data.
  vector<vector<ServoFeedback*> > servo_feedback_history;
  servo_feedback_history.resize(number_of_modules);


  //-- Initialise local variables.
  init_local_variables(previous_cycle_output, isFirstStep);

  double t;

  unsigned long previous_read_elapsed_time = 0;
  unsigned long evaluation_elapsed_time = 0;
  unsigned long evaluation_window = (unsigned long)evaluation_period * 1000000; // Converted to microseconds;

  unsigned int key = 'H';
  changemode(1);

  do
  {
      do
      {
        if(!read_servo_positions_with_time())
        {
            std::cout << "  Communication break down. Redo evaluation." << std::endl;
            robot_primary->reset_comm_link();

            SS << "REDO" << " ";
            return true;
        }

        //--Record the reference position.
        if(oscAnlz && oscAnlz->get_record_ref())
        {
          oscAnlz->write_ref(output);
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

#ifdef DEBUGGER
  std::cout << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
#endif

  SS << "SUCCESS" << " ";

  return true;
}


void SineController::actuate_with_sine_controller(const unsigned int module, const double t, Flood::Vector<double>& output)
{
  //output[module] = sine_amplitude[module] * sin(2*M_PI*sine_frequency*t + ((sine_phase[module] * M_PI)/180.0)) + sine_offset[module];
  output[module] = sine_wave(sine_amplitude[module], sine_offset[module], sine_frequency, sine_phase[module], t);
}
