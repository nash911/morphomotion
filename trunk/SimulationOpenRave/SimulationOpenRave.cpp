/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   S I M U L A T I O N O P E N R A V E   C L A S S                                                            */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#include "SimulationOpenRave.h"

using namespace OpenRAVE;

ViewerBasePtr viewer;

void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
  viewer = RaveCreateViewer(penv,viewername);
  BOOST_ASSERT(!!viewer);

  // attach it to the environment:
  penv->AttachViewer(viewer);

  // finally you call the viewer's infinite loop (this is why you need a separate thread):
  bool showgui = true;
  viewer->main(showgui);
}


// DEFAULT CONSTRUCTOR
SimulationOpenRave::SimulationOpenRave(void):Robot()
{
  //-- Set default parameters.
  set_default_parameters();
}


// DEFAULT CONSTRUCTOR
SimulationOpenRave::SimulationOpenRave(const Robot* source_object):Robot()
{
  //-- Set default parameters.
  set_default_parameters();

  this->set_robot_type(source_object->get_robot_type());
  this->set_evaluation_method(source_object->get_evaluation_method());
  this->set_number_of_modules(source_object->get_number_of_modules());
}


// CONSTRUCTOR WITH SCENE FILE NAME
SimulationOpenRave::SimulationOpenRave(std::string& scenefilename):Robot()
{
  //-------------------------------//
  //-- OPENRAVE INITIZALIZATION -- //
  //-------------------------------//
  string viewername = "qtcoin";

  RaveInitialize(true); // start openrave core.
  penv = RaveCreateEnvironment(); // create the main environment.
  RaveSetDebugLevel(Level_Debug);

  boost::thread thviewer(boost::bind(SetViewer,penv,viewername));
  usleep(100000); //-- Wait for the viewer to setup.
  penv->Load(scenefilename); // load the scene.
  SetCamera(0.427, 0.285, 0.47, 0.718, 0.59, 0.078, 0.263);
  penv->StopSimulation();
  std::cout << "Thread Completed" << std::endl;

  //-- Get the robot.
  std::vector<RobotBasePtr> robots;
  penv->GetRobots(robots);
  probot = robots[0];
  cout << "Robot: " << probot->GetName() << endl;

  // create the controllers, make sure to lock environment!
  EnvironmentMutex::scoped_lock EnvLock(penv->GetMutex()); // lock environment

  // Get number of modules in the configuration
  number_of_modules = probot->GetDOF();

  //-- Load the controller.
  pcontroller=RaveCreateController(penv,"servocontroller");
  //pcontroller=RaveCreateController(penv,"sinoscontroller");
  vector<int> dofindices(probot->GetDOF());
  for(int i = 0; i < probot->GetDOF(); ++i)
  {
    dofindices[i] = i;
  }
  probot->SetController(pcontroller,dofindices,1);

  //-- Record the initial position of the robot.
  t0=probot->GetTransform();

  // Capture initial position of the robot
  //robot_pos_initial = get_robot_XY(); // TODO: May be not needed here.

  //-- Set default parameters.
  set_default_parameters();
}


// DESTRUCTOR
SimulationOpenRave::~SimulationOpenRave(void)
{
}


void SimulationOpenRave::init_simu_env(std::string controller)
{
  //-------------------------------//
  //-- OPENRAVE INITIZALIZATION -- //
  //-------------------------------//
  string viewername = "qtcoin";

  RaveInitialize(true); // start openrave core.
  penv = RaveCreateEnvironment(); // create the main environment.
  RaveSetDebugLevel(Level_Debug);

  boost::thread thviewer(boost::bind(SetViewer,penv,viewername));
  usleep(100000); //-- Wait for the viewer to setup.
  penv->Load(scene_file_name); // load the scene.
  SetCamera(0.427, 0.285, 0.47, 0.718, 0.59, 0.078, 0.263);
  penv->StopSimulation();
  std::cout << "Thread Completed" << std::endl;

  //-- Get the robot.
  std::vector<RobotBasePtr> robots;
  penv->GetRobots(robots);
  probot = robots[0];
  cout << "Robot: " << probot->GetName() << endl;

  // create the controllers, make sure to lock environment!
  EnvironmentMutex::scoped_lock EvnLock(penv->GetMutex()); // lock environment

  // Get number of modules in the configuration
  number_of_modules = probot->GetDOF();

  //-- Load the controller.
  if(controller == "Sinusoidal_Controller")
  {
    pcontroller=RaveCreateController(penv,"sinoscontroller");
  }
  else
  {
    pcontroller=RaveCreateController(penv,"servocontroller");
  }
  vector<int> dofindices(probot->GetDOF());
  for(int i = 0; i < probot->GetDOF(); ++i)
  {
    dofindices[i] = i;
  }
  probot->SetController(pcontroller,dofindices,1);
  //load_controller("servocontroller");

  //-- Record the initial position of the robot.
  t0=probot->GetTransform();

  // Capture initial position of the robot
  //robot_pos_initial = get_robot_XY(); // TODO: May be not needed here.
}


void SimulationOpenRave::set_default_parameters(void)
{
  scene_file_name = "/home/nash/Dropbox/PhD/modularRobotics/morphoMotion/models/Minicube-I.env.xml";
  //simu_resolution_microseconds = 0.005;  // Bug
  simu_resolution_microseconds = 0.0025; // Bug (fix?): When this parameter is 0.005, while evaluating Simulated_Cube2 configuration with controller evolved using a value of '0.0025', the single phase between two modules starts to oscillate, making the robot move forward and backward, just as observed while evaluating Simulated_Robot evolved controller on the Real_Robot.
  //unit_second = (double)1/simu_resolution_microseconds; // TODO: To be removed.
  number_of_modules = 2;
}


void SimulationOpenRave::SetCamera(dReal q0, dReal q1, dReal q2, dReal q3, dReal tx, dReal ty, dReal tz)
{
  RaveVector<float> rotation(q0,q1,q2,q3);
  RaveVector<float> translation(tx,ty,tz);
  RaveTransform<float> T(rotation,translation);
  viewer->SetCamera(T);
}


void SimulationOpenRave::load_controller(std::string controller)
{
  EnvironmentMutex::scoped_lock EvnLock(penv->GetMutex()); // lock environment
  pcontroller=RaveCreateController(penv,controller);
  vector<int> dofindices(probot->GetDOF());
  for(int i = 0; i < probot->GetDOF(); ++i)
  {
    dofindices[i] = i;
  }
  probot->SetController(pcontroller,dofindices,1);
}

void SimulationOpenRave::set_scene_file_name(const std::string new_scene_file_name)
{
  scene_file_name = new_scene_file_name;
}


std::string SimulationOpenRave::get_scene_file_name(void)
{
  return(scene_file_name);
}


void SimulationOpenRave::set_simu_resolution_microseconds(double new_simu_resolution_microseconds)
{
  simu_resolution_microseconds = new_simu_resolution_microseconds;
  //unit_second = (double)1/simu_resolution_microseconds; // TODO: To be removed.
}


double SimulationOpenRave::get_simu_resolution_microseconds(void)
{
  return(simu_resolution_microseconds);
}


/*void SimulationOpenRave::set_unit_second(int new_unit_second)  // TODO: To be removed
{
  if(new_unit_second > 0)
  {
    unit_second = new_unit_second;
  }
  else
  {
    std::cerr << "cubeRevolution Error: SimEnv class." << std::endl
              << "void set_new_unit_second(int) method."
              << std::endl
              << "Unit second must be greater than zero." << std::endl;
    exit(1);
  }
}*/


/*int SimulationOpenRave::get_unit_second(void)  // TODO: To be removed
{
  return(unit_second);
}*/


Vector SimulationOpenRave::get_robot_XY()
{
  return(probot->GetCenterOfMass());
}


void SimulationOpenRave::reset_robot(void)
{
  //-- Stop Simulation
  penv->StopSimulation();

  //-- Set the servo of each module to zero.
  for(int i=0; i<number_of_modules; i++)
  {
    set_moduleServo_position(i, 0);
  }

  // Wait for two seconds.
  int two_seconds = (1/simu_resolution_microseconds)*2;
  for (int ss=0; ss<two_seconds; ss++)
  {
    penv->StepSimulation(simu_resolution_microseconds);
  }

  //-- Set the translation of the robot to the initial position.
  probot->SetTransform(t0);

  //-- Capture initial position of the robot
  //robot_pos_initial = get_robot_XY();
  robot_pos_previous = get_robot_XY();

  //-- Initialize the controller evaluation time counter to 0;
  init_elapsed_evaluation_time();

  distance_travelled = 0;
}


/*void SimulationOpenRave::set_sinusoidal_controller_parameters(const vector<double>& sinusoidal_amplitude, const vector<double>& sinusoidal_offset, const vector<double>& sinusoidal_phase, const double sinusoidal_frequency)
{
  stringstream os,is;
  is << "setamplitude 60 60 60 60 ";
  pcontroller->SendCommand(os,is);

  //is << "setinitialphase 0 120 ";
  is << "setinitialphase 0 0 120 240 ";
  pcontroller->SendCommand(os,is);

  is << "setoffset 0 0 0 0 ";
  pcontroller->SendCommand(os,is);

  is << "setperiod 1.0 ";
  pcontroller->SendCommand(os,is);

  is << "oscillation on ";
  pcontroller->SendCommand(os,is);
}*/


void SimulationOpenRave::set_sinusoidal_controller_parameters(const vector<double>& sinusoidal_amplitude, const vector<double>& sinusoidal_offset, const vector<double>& sinusoidal_phase, const double sinusoidal_frequency)
{
  stringstream os,is;

  is << "setamplitude ";
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    is << sinusoidal_amplitude[module] << " ";
  }
  pcontroller->SendCommand(os,is);

  is << "setinitialphase ";
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    is << sinusoidal_phase[module] << " ";
  }
  pcontroller->SendCommand(os,is);

  is << "setoffset ";
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    is << sinusoidal_offset[module] << " ";
  }
  pcontroller->SendCommand(os,is);

  is << "setperiod " << sinusoidal_frequency << " ";
  pcontroller->SendCommand(os,is);

  is << "oscillation on ";
  pcontroller->SendCommand(os,is);
}


void SimulationOpenRave::stop_sinusoidal_controller(void)
{
  stringstream os,is;

  is << "reset_controller ";
  pcontroller->SendCommand(os,is);

  is << "oscillation off ";
  pcontroller->SendCommand(os,is);

  //-- Set the servo of each module to zero.
  /*for(int i=0; i<number_of_modules; i++)
  {
    set_moduleServo_position(i, 80);
  }

  // Wait for two seconds.
  int two_seconds = (1/simu_resolution_microseconds)*10;
  for (int ss=0; ss<two_seconds; ss++)
  {
    penv->StepSimulation(simu_resolution_microseconds);
  }*/
}


void SimulationOpenRave::set_moduleServo_position(unsigned int module, double servo_angle)
{
  stringstream os,is;
  is << "setpos1" << " " << module << " " << servo_angle << " ";
  pcontroller->SendCommand(os,is);
}


double SimulationOpenRave::get_moduleServo_position(unsigned int module)
{
  stringstream os,is;
  double servo_angle;

  is << "getpos1" << " " << module << " ";
  pcontroller->SendCommand(os,is);
  os >> servo_angle;

 return servo_angle;
}


std::vector<double> SimulationOpenRave::get_all_moduleServo_position() // TODO: This should be removed after implementing get_all_moduleServo_position_with_time().
{
  stringstream os,is;
  std::vector<double> servo_angle;
  double angle;

  for(int module=0; module<number_of_modules; module++)
  {
    is << "getpos1" << " " << module << " ";
    pcontroller->SendCommand(os,is);
    os >> angle;
    servo_angle.push_back(angle);
  }
  return servo_angle;
}


void SimulationOpenRave::get_all_moduleServo_position_with_time(vector<ServoFeedback*>& servo_feedback)
{
  stringstream os,is;
  double angle = 21.0; // This value set to 21.0 as a way of detecting when a failuer to read module position occurs.

  for(int module=0; module<number_of_modules; module++)
  {
    is << "getpos1" << " " << module << " ";
    pcontroller->SendCommand(os,is);
    os >> angle;
    servo_feedback[module]->set_new_value(elapsed_evaluation_time, angle);
    //std::cout << "Inside for(;;) --> Module: " << module << std::endl; // TODO: Debugger to be removed.
  }
}


void SimulationOpenRave::init_elapsed_evaluation_time(void)
{
  elapsed_evaluation_time = 0;
}


void SimulationOpenRave::update_elapsed_evaluation_time(void)
{
  elapsed_evaluation_time = elapsed_evaluation_time + (simu_resolution_microseconds * 1000000);
}

unsigned long SimulationOpenRave::get_elapsed_evaluation_time(void)
{
  return(elapsed_evaluation_time);
}


double SimulationOpenRave::calculate_distance_travelled_euclidean(void)
{
  Vector robot_pos_current = probot->GetCenterOfMass();  // TODO: To be changed to get_robot_XY();
  Vector distance3D = robot_pos_current - robot_pos_previous;
  double distanceTravelled = sqrt((distance3D.x*distance3D.x)+(distance3D.y*distance3D.y));
  robot_pos_previous = robot_pos_current;
  return(distanceTravelled);
}


void SimulationOpenRave::measure_cumulative_distance(void)
{
  distance_travelled = distance_travelled + calculate_distance_travelled_euclidean();
  //std::cout << std::endl << "Measured Cumulative Distance: " << distance_travelled << std::endl;  // Debugger
}


double SimulationOpenRave::get_robot_X(void)
{
  Vector robot_pos_current = probot->GetCenterOfMass();  // TODO: To be changed to get_robot_XY();
  double robot_X = robot_pos_current.x;
  return(robot_X);
}


double SimulationOpenRave::get_robot_Y(void)
{
  Vector robot_pos_current = probot->GetCenterOfMass();  // TODO: To be changed to get_robot_XY();
  double robot_Y = robot_pos_current.y;
  return(robot_Y);
}


unsigned long SimulationOpenRave::step(const std::string& type)
{
  penv->StepSimulation(simu_resolution_microseconds);

#ifndef Y1_CONFIGURATION
  if(type == "evaluation")
  {
    usleep(get_simu_resolution_microseconds() * 1000000);  // Real-Time
    //usleep(get_simu_resolution_microseconds() * 500000);  // Real-Time * 1/2
  }

  //elapsed_evaluation_time = elapsed_evaluation_time + (simu_resolution_microseconds * 1000000);
  update_elapsed_evaluation_time();

  if(evaluation_method == Euclidean_Distance_Cumulative)
  {
    if(elapsed_evaluation_time % (CUMULATIVE_DISTENCE_MEASUREMENT_RESOLUTION * 1000000) == 0)
    {
      measure_cumulative_distance();
    }
  }
#endif

  return(elapsed_evaluation_time);  // TODO: A return value of elapsed_evaluation_time may not be needed. Check and change strategy.
}
