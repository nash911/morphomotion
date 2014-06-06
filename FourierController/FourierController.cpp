/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   F O U R I E R   C O N T R O L L E R   C L A S S                                                            */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#include "FourierController.h"

FourierController::FourierController(void):Controller()
{
}


FourierController::FourierController(Flood::MultilayerPerceptron* mlp_pointer, Robot* pointer_robot_primary):Controller(mlp_pointer, pointer_robot_primary)
{
}


FourierController::FourierController(Flood::MultilayerPerceptron* mlp_pointer, Robot* pointer_robot_primary, Robot* pointer_robot_secondary):Controller(mlp_pointer, pointer_robot_primary, pointer_robot_secondary)
{
}


void FourierController::init_controller(const double delta_time)
{
  Controller::init_controller(0);

  //-- Set the size of the Controller parameters vector
  Ak.resize(number_of_modules);
  Bk.resize(number_of_modules);
  sine_amplitude.resize(number_of_modules);
  sine_offset.resize(number_of_modules);
  normalizing_factor.resize(number_of_modules);

  for(unsigned int module=0; module<number_of_modules; module++)
  {
    normalizing_factor[module] = 1;
  }
}


void FourierController::set_default(void)
{
  Controller::set_default();
}


void FourierController::init_local_variables(Flood::Vector<double> &previous_cycle_output)
{
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    previous_cycle_output[module] = 0;
  }
}


/*void FourierController::estimate_fourier_control_normalizing_factor()
{
  vec Xt[number_of_modules];
  double x;
  double T = 1.0/get_sine_frequency();
  double delta_t = 0.01;
  double max, min;

  for(unsigned int module=0; module<number_of_modules; module++)
  {
    for(double t=0; t<T; t=t+delta_t)
    {
      x=0;
      for(unsigned int k=0; k<frequency_domain_size; k++)
      {
        x += (Ak[module][k] * sin(2*M_PI*(k/T)*t)) + (Bk[module][k] * cos(2*M_PI*(k/T)*t));
      }
      Xt[module] << x << endr;
    }
  }

  for(double t=0; t<T; t=t+delta_t)
  {
    //std::cout << "Xt[" << t << "]: " << Xt[0][]
  }

  for(unsigned int module=0; module<number_of_modules; module++)
  {
    max = Xt[module].max();
    min = Xt[module].min();

    if(fabs(max) >= fabs(min))
    {
      normalizing_factor[module] = max;
    }
    else
    {
      normalizing_factor[module] = min;
    }

    std::cout << "normalizing_factor[" << module+1 << "]: " << normalizing_factor[module] << std::endl; // TODO: Debugger to be removed.
  }
}*/


void FourierController::estimate_fourier_control_normalizing_factor()
{
  double x;
  double T = 1.0/get_sine_frequency();
  double delta_t = 0.0001;
  double max, min;

  for(unsigned int module=0; module<number_of_modules; module++)
  {
    vec Xt((T/delta_t)+1);
    unsigned int i=0;
    for(double t=0; t<T; t=t+delta_t, i++)
    {
      x=0;
      for(unsigned int k=0; k<frequency_domain_size; k++)
      {
        x += (Ak[module][k] * sin(2*M_PI*(k/T)*t)) + (Bk[module][k] * cos(2*M_PI*(k/T)*t));
      }
      Xt(i) = x;
      //std::cout << "Xt[" << i << "]: " << x << std::endl; //TODO: Debugger to be removed.
    }

    max = Xt.max();
    min = Xt.min();

    if(fabs(max) >= fabs(min))
    {
      normalizing_factor[module] = fabs(max);
    }
    else
    {
      normalizing_factor[module] = fabs(min);
    }
    //std::cout << "max[" << module+1 << "]: " << max << std::endl; //TODO: Debugger to be removed.
    //std::cout << "min[" << module+1 << "]: " << min << std::endl; //TODO: Debugger to be removed.
    //std::cout << "normalizing_factor[" << module+1 << "]: " << normalizing_factor[module] << std::endl; //TODO: Debugger to be removed.
  }
}


void FourierController::start_Controller(const std::string& type, std::stringstream& SS, int generation) //--Thread Change
{
  //--Reset controller.
  reset_controller();

  robot_primary->set_receive_broadcast(true);

  std::thread ctrl(&FourierController::run_Controller, this, type, std::ref(SS), generation);
  std::thread read_broadcast(&FourierController::read_servo_positions_with_time_THREAD, this);

  ctrl.join();
  read_broadcast.join();
}


void FourierController::run_Controller(const std::string& type, std::stringstream& SS, int generation)
{
  load_fourier_control_parameters();
  estimate_fourier_control_normalizing_factor();

  Flood::Vector<double> output(number_of_modules);
  Flood::Vector<double> previous_cycle_output(number_of_modules);

  // Related to Servo Derivative Resolution -- Based on time dependent Servo Feedback data.
  vector<vector<ServoFeedback*> > servo_feedback_history;
  servo_feedback_history.resize(number_of_modules);

  //-- Initialise local variables.
  init_local_variables(previous_cycle_output);

  double t=0;

  //unsigned long previous_read_elapsed_time = 0;
  unsigned long evaluation_elapsed_time = 0;
  unsigned long evaluation_window = (unsigned long)evaluation_period * 1000000; // Converted to microseconds;

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
              std::cout << "Count: " << evaluation_elapsed_time/1000 << "  Output[" << module << "]: " << output[module] << "  Feedback Self: " << servo_feedback[module]->get_servo_position();
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

            actuate_with_fourier_controller(module, t, output);

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
        //previous_read_elapsed_time = robot_primary->get_previous_read_evaluation_time();
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


void FourierController::actuate_with_fourier_controller(const unsigned int module, const double t, Flood::Vector<double>& output)
{
  double Xt = 0;
  double T = 1.0/get_sine_frequency();

  for(unsigned int k=0; k<frequency_domain_size; k++)
  {
    Xt += (Ak[module][k] * sin(2*M_PI*(k/T)*t)) + (Bk[module][k] * cos(2*M_PI*(k/T)*t));
  }

  output[module] = sine_amplitude[module] * (Xt/normalizing_factor[module]);
}
