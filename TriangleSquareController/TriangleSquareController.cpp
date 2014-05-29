/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   T R I A N G L E S Q U A R E   C O N T R O L L E R   C L A S S                                              */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#include "TriangleSquareController.h"

TriangleSquareController::TriangleSquareController(void):Controller()
{
}


TriangleSquareController::TriangleSquareController(Flood::MultilayerPerceptron* mlp_pointer, Robot* pointer_robot_primary):Controller(mlp_pointer, pointer_robot_primary)
{
}


TriangleSquareController::TriangleSquareController(Flood::MultilayerPerceptron* mlp_pointer, Robot* pointer_robot_primary, Robot* pointer_robot_secondary):Controller(mlp_pointer, pointer_robot_primary, pointer_robot_secondary)
{
}


void TriangleSquareController::init_controller(const double delta_time)
{
  Controller::init_controller(0);

  //-- Set the size of TriangleSquare Controller parameters vector
  A.resize(number_of_modules);
  for(unsigned int module = 0; module < number_of_modules; module++)
  {
      A[module].resize(2);
  }

  s.resize(number_of_modules);
  for(unsigned int module = 0; module < number_of_modules; module++)
  {
      s[module].resize(2);
  }

  a.resize(number_of_modules);
  for(unsigned int module = 0; module < number_of_modules; module++)
  {
      a[module].resize(2);
  }

  offset.resize(number_of_modules);
  phase.resize(number_of_modules);
}


void TriangleSquareController::set_default(void)
{
  Controller::set_default();
}


void TriangleSquareController::init_local_variables(Flood::Vector<double> &previous_cycle_output)
{
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    previous_cycle_output[module] = 0;
  }
}


void TriangleSquareController::start_Controller(const std::string& type, std::stringstream& SS, int generation) //--Thread Change
{
  //--Reset controller.
  reset_controller();

  robot_primary->set_receive_broadcast(true);

  std::thread ctrl(&TriangleSquareController::run_Controller, this, type, std::ref(SS), generation);
  std::thread read_broadcast(&TriangleSquareController::read_servo_positions_with_time_THREAD, this);

  ctrl.join();
  read_broadcast.join();
}



void TriangleSquareController::run_Controller(const std::string& type, std::stringstream& SS, int generation)
{
  load_trianglesquare_control_parameters();

  Flood::Vector<double> output(number_of_modules);
  Flood::Vector<double> previous_cycle_output(number_of_modules);

  //-- Related to Servo Derivative Resolution -- Based on time dependent Servo Feedback data.
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
              std::cout << generation << " -->  " << evaluation_elapsed_time << " : Previous Output[" << module << "]: " << previous_cycle_output[module] << "  Actual Angle: " << servo_feedback[module]->get_servo_position() << "  Current Output[" << module << "]: " << output[module];
            }
#endif
//-------------------------------------------------------------- Activity Log --------------------------------------------------------/

            //-- Storing the output of the previous cycle.
            previous_cycle_output[module] = output[module];

            actuate_with_trianglesquare_controller(module, t, output);

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
  }while(evaluation_elapsed_time < evaluation_window && (key != q || key != Q)  && robot_primary->get_receive_broadcast());  //--Thread Change
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


double TriangleSquareController::sawtooth(const double t, const unsigned int module)
{
  double saw;
  double Period = 1.0/frequency;

  //--            |‾                                       ‾|
  //--            |  t    sgn(t)Φ   | 1     t    sgn(t)Φ |  |                                   --- (1)
  //-- Saw(t) = 2 | --- + ------- - |--- + --- + ------- |  |
  //--            |  P      2π      | 2     P       2π   |  |
  //--            |_                |_                  _| _|

  //saw = 2.0 * ((t+((sgn(t)*phase[module])/(2*M_PI)))/P - floor((t+((sgn(t)*phase[module])/(2*M_PI)))/P + 0.5)); //-- BUG: Had a bug in Phase part of the equation.
  saw = 2.0 * ((t/Period) + ((sgn(t)*phase[module])/(2*M_PI)) - floor((t/Period) + ((sgn(t)*phase[module])/(2*M_PI)) + 0.5));
  return saw;
}


double TriangleSquareController::triangle(const double t, const unsigned int module)
{
  return((2.0 / M_PI ) * asin(sin((2 * M_PI * frequency * t) + phase[module])));
}


double TriangleSquareController::triangle_asymmetric(const double t, const double a, const unsigned int module)
{
  double tri_asym = 0;
  double saw = sawtooth(t, module);
  double saw_rev = sawtooth(-t, module);

  //--                |‾                                                                                                                    ‾|
  //--                | |‾        ‾|  |‾                      ‾|        |‾                                                                 ‾| |
  //--             1  | |  Saw(t)  |  |                       |    1   |                                                                   | |
  //-- Tri(t,a) = --- | | -------- |  | sgn(a - |Saw()t|) + 1 | + ---  | (Saw(-t)+1) (sgn(-a-Saw(-t))+1) + (Saw(-t)-1) (sgn(-a+Saw(-t)+1)) | | --- (2)
  //--             2  | |     a    |  |                       |   1-a  |                                                                   | |
  //--                | |_        _|  |_                     _|        |_                                                                 _| |
  //--                |_                                                                                                                    _|

  tri_asym = 0.5 * ((saw/a)*(sgn(a - fabs(saw)) + 1.0) + ((1.0/(1.0-a)) * (((saw_rev + 1.0) * (sgn(-a - saw_rev) + 1.0)) + ((saw_rev - 1.0) * (sgn(-a + saw_rev) + 1.0)))));

  return tri_asym;
}


void TriangleSquareController::actuate_with_trianglesquare_controller(const unsigned int module, const double t, Flood::Vector<double>& output)
{
  double tri_sqr;
  //double Tri = triangle(t, module);
  std::vector<double> Tri(2);
  double polarity;
  double sqr;
  double tri;

  Tri[0] = triangle_asymmetric(t, a[module][0], module);
  Tri[1] = triangle_asymmetric(t, a[module][1], module);

  //--                           |                         | |‾                                                                                                   ‾|
  //--                    1      | sgn(Tri(t,a_i))+(-1)^i  | |         |  |             |       |                    |‾ |‾                   ‾|     ‾|             |
  //-- Tri_Sqr(t,a) = O + Σ  A'i | ----------------------- | | sgn(s_i)|  | Tri(t, a_i) | + s_i | (1 - s_i) (-1)^i - |  | |Tri(t, a_i)| + s_i | - 2  | Tri(t, a_i)|  ------ (4)
  //--                   i=0     |             2           | |         |_ |             |      _|                    |_ |                     |     _|            |
  //--                           |                         | |_                                                                                                  _|

  tri_sqr = offset[module];
  for(unsigned int i=0; i<=1; i++)
  {
    polarity = fabs((sgn(Tri[i]) + pow(-1,i)) / 2.0);
    sqr = sgn(s[module][i]) * floor(fabs(Tri[i]) + s[module][i]) * (1-s[module][i]) * pow(-1,i);
    tri = (ceil(fabs(Tri[i]) + s[module][i]) - 2) * Tri[i];

    tri_sqr += A[module][i] * polarity * (sqr - tri);
  }

  output[module] = tri_sqr;

  //--Make sure that the next control signal is a value between the servo range.
  if(output[module] > get_servo_max())
  {
    output[module] = get_servo_max();
  }
  else if(output[module] < get_servo_min())
  {
    output[module] = get_servo_min();
  }

  //-- TODO: This is only for a test and should be removed.
  /*if(module == 0 || module == 1 || module == 2)
  {
    output[module] = 0;
  }*/
}
