/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   S I M P L E   C O N T R O L L E R   C L A S S                                                              */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#include "SimpleController.h"

SimpleController::SimpleController(void):Controller()
{
}


SimpleController::SimpleController(Flood::MultilayerPerceptron* mlp_pointer, Robot* pointer_robot_primary):Controller(mlp_pointer, pointer_robot_primary)
{
}


SimpleController::SimpleController(Flood::MultilayerPerceptron* mlp_pointer, Robot* pointer_robot_primary, Robot* pointer_robot_secondary):Controller(mlp_pointer, pointer_robot_primary, pointer_robot_secondary)
{
}


void SimpleController::init_controller(const double delta_time)
{
  Controller::init_controller(delta_time);
}


void SimpleController::set_default(void)
{
  Controller::set_default();

  oscillator_amplitude = 0;
  oscillator_offset = 0;
}


void SimpleController::init_local_variables(Flood::Vector<double> &output,
                                            Flood::Vector<double> &previous_cycle_output,
                                            Flood::Vector<bool> &isFirstStep)
{
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    //-- Setting the motors to a the amplitude value.
    output[module] = oscillator_amplitude + oscillator_offset;
  }

  for(unsigned int module=0; module<number_of_modules; module++)
  {
    previous_cycle_output[module] = 0;
    isFirstStep[module] = true;
  }
}


void SimpleController::start_Controller(const std::string& type, std::stringstream& SS, int generation)
{
  //--Reset controller.
  reset_controller();

  robot_primary->set_receive_broadcast(true);

  std::thread ctrl(&SimpleController::run_Controller, this, type, std::ref(SS), generation);
  std::thread read_broadcast(&SimpleController::read_servo_positions_with_time_THREAD, this);

  ctrl.join();
  read_broadcast.join();
}


void SimpleController::run_Controller(const std::string& type, std::stringstream& SS, int generation)
{
    set_servo_derivative_threshold(mlp->get_independent_parameter(0));
    set_servo_derivative_epsilon(mlp->get_independent_parameter(1));
    set_oscillator_amplitude(mlp->get_independent_parameter(2));
    set_oscillator_offset(mlp->get_independent_parameter(3));

    Flood::Vector<double> output(number_of_modules);
    Flood::Vector<double> previous_cycle_output(number_of_modules);
    Flood::Vector<bool> isFirstStep(number_of_modules);

    // Related to Servo Derivative Resolution -- Based on time dependent Servo Feedback data.
    vector<vector<ServoFeedback*> > servo_feedback_history;
    servo_feedback_history.resize(number_of_modules);


    //-- Initialise local variables.
    init_local_variables(output, previous_cycle_output, isFirstStep);

    double servo_delta = 0;
    double servo_derivative;

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
          //--Record reference position.
          if(oscAnlz && oscAnlz->get_record_ref())
          {
            oscAnlz->write_ref(output);
          }

          //--Record current position.
          if(oscAnlz && oscAnlz->get_record_servo() && robot_primary->get_robot_environment() == "Y1")
          {
            std::vector<double> servo_positions;
            for(unsigned int module=0; module<number_of_modules; module++)
            {
              servo_positions.push_back(servo_feedback[module]->get_servo_position());
            }
            oscAnlz->write_servo(servo_positions);
          }

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
              std::cout << "Count: " << evaluation_elapsed_time/1000 << "  Output[" << module << "]: " << output[module] << "  Feedback Self: " << servo_feedback[module]->get_servo_position() << "  Feed Back Diff: " << servo_delta << "  Servo Delta: " << servo_derivative;
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

            actuate_with_simple_controller(module, output);

            if(oscAnlz)
            {
              // Checking if the previous and the current neural outputs are on opposite directions from value 0.
              if((previous_cycle_output[module] * output[module]) < 0.0) // Here we are assuming that the control signal to each module is always oscillatory. As well as that two subsequent neural outputs are always on opposite directions from the value 0.
              {
                //oscAnlz->update_oscillation_short_history(module,output[module]); // TODO: Oscillation analyses is performed based on control signal generated by controller output. This should be changed by analysing an oscillation based on the actual feedback from the module.
                oscAnlz->update_oscillation_short_history(module,servo_feedback[module]->get_servo_position()); // Analysing oscillation based on propreoseptive feedback from the module's actuator.
              }
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
              oscillation_time[module] = 0;
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

        robot_primary->set_receive_broadcast(false); //--Thread Change
        while(robot_primary->get_broadcast_thread());
        return;
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

        robot_primary->set_receive_broadcast(false);
        while(robot_primary->get_broadcast_thread());
        return;
      }
  }while(evaluation_elapsed_time < evaluation_window && (key != q || key != Q));
  changemode(0);

  #ifdef DEBUGGER
    std::cout << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
  #endif

    SS << "SUCCESS" << " ";

    robot_primary->set_receive_broadcast(false);
    while(robot_primary->get_broadcast_thread());
    return;
}


void SimpleController::actuate_with_simple_controller(const unsigned int module, Flood::Vector<double>& output)
{
  output[module] = (-1.0 * (output[module] - oscillator_offset)) + oscillator_offset; // BUG FIX: -(Amplitude) + Offset
  actuate_module(module, output[module]);
}
