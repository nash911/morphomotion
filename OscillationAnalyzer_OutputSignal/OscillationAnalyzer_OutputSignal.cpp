/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   O U T P U T S I G N A L   O S C I L L A T I O N A N A L Y Z E R   C L A S S                                */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#include "OscillationAnalyzer_OutputSignal.h"

OscillationAnalyzer_OutputSignal::OscillationAnalyzer_OutputSignal(Robot* robot_pointer)
{
  robot = robot_pointer;
  unsigned int number_of_modules = robot->get_number_of_modules();

  oscillation_short_history.set(number_of_modules, 3);
  time_at_previous_cycle.set(number_of_modules);
  amplitude_degrees.set(number_of_modules);
  offset_degrees.set(number_of_modules);
  frequency_hertz.set(number_of_modules);
  phase_degrees.set(number_of_modules,number_of_modules);

  oscillation_short_history.initialize(0);
  time_at_previous_cycle.initialize(0);
  amplitude_degrees.initialize(0);
  offset_degrees.initialize(0);
  frequency_hertz.initialize(0);
  phase_degrees.initialize(0);

  phase_graph_size = 0;
  for(unsigned int i=0; i<number_of_modules; i++)
  {
    phase_graph_size = phase_graph_size + i;
  }

  estimate_amplitude_offset = false;
  estimate_frequency = false;
  estimate_phase = false;

  record_servo = false;
  record_ref = false;
  record_amplitude = false;
  record_offset = false;
  record_frequency = false;
  record_phase = false;
  record_trajectory = false;

  servo_graph_file = NULL;
  ref_graph_file = NULL;
  amplitude_graph_file = NULL;
  offset_graph_file = NULL;
  frequency_graph_file = NULL;
  phase_180_graph_file = NULL;
  phase_360_graph_file = NULL;
  trajectory_graph_file = NULL;
}


// DESTRUCTOR
OscillationAnalyzer_OutputSignal::~OscillationAnalyzer_OutputSignal(void)
{
    if(servo_graph_file != NULL)
    {
        delete servo_graph_file;
        //std::cout << std::endl << "servo_graph_file Deleted." << std::endl;
    }

    if(ref_graph_file != NULL)
    {
        delete ref_graph_file;
        //std::cout << std::endl << "ref_graph_file Deleted." << std::endl;
    }

    if(amplitude_graph_file != NULL)
    {
        delete amplitude_graph_file;
        //std::cout << std::endl << "amplitude_graph_file Deleted." << std::endl;
    }

    if(offset_graph_file != NULL)
    {
        delete offset_graph_file;
        //std::cout << std::endl << "offset_graph_file Deleted." << std::endl;
    }

    if(frequency_graph_file != NULL)
    {
        delete frequency_graph_file;
        //std::cout << std::endl << "frequency_graph_file Deleted." << std::endl;
    }

    if(phase_180_graph_file != NULL)
    {
        delete phase_180_graph_file;
        //std::cout << std::endl << "phase_180_graph_file Deleted." << std::endl;
    }

    if(phase_360_graph_file != NULL)
    {
        delete phase_360_graph_file;
        //std::cout << std::endl << "phase_360_graph_file Deleted." << std::endl;
    }

    if(trajectory_graph_file != NULL)
    {
        delete trajectory_graph_file;
        //std::cout << std::endl << "trajectory_graph_file Deleted." << std::endl;
    }
}

void OscillationAnalyzer_OutputSignal::update_oscillation_short_history(unsigned int module, double new_oscillation_value)
{
  oscillation_short_history[module][0] = oscillation_short_history[module][1];
  oscillation_short_history[module][1] = oscillation_short_history[module][2];
  oscillation_short_history[module][2] = new_oscillation_value;

  if(estimate_amplitude_offset)
  {
    calculate_amplitude_offset(module);
  }

  if(estimate_phase)
  {
    calculate_phase(module);
  }

  if(estimate_frequency)
  {
    calculate_frequency(module);
  }
}


void OscillationAnalyzer_OutputSignal::calculate_amplitude_offset(unsigned int module)
{
  if(oscillation_short_history[module][0] < oscillation_short_history[module][1] && oscillation_short_history[module][1] > oscillation_short_history[module][2])
  {
    amplitude_degrees[module] = abs(oscillation_short_history[module][1] - oscillation_short_history[module][2])/2;
    offset_degrees[module] = oscillation_short_history[module][1] -amplitude_degrees[module];

    if(record_amplitude)
    {
      write_amplitude();
    }

    if(record_offset)
    {
      write_offset();
    }
  }
}


void OscillationAnalyzer_OutputSignal::calculate_frequency(unsigned int module)
{
  if(oscillation_short_history[module][0] < oscillation_short_history[module][1] && oscillation_short_history[module][1] > oscillation_short_history[module][2])
  {
    double frequency_time = robot->get_elapsed_evaluation_time() - time_at_previous_cycle[module];

    if(frequency_time > 0.0)
    {
      frequency_hertz[module] = (double)1000000/frequency_time;  // Frequency of oscillation calculated as Hz/sec.
    }
    else
    {
      frequency_hertz[module] = 0;
    }

    time_at_previous_cycle[module] = robot->get_elapsed_evaluation_time();

    if(record_frequency)
    {
      write_frequency();
    }
  }
}


void OscillationAnalyzer_OutputSignal::calculate_phase(unsigned int module)
{
  if(oscillation_short_history[module][0] < oscillation_short_history[module][1] && oscillation_short_history[module][1] > oscillation_short_history[module][2])
  {
    unsigned long elapsed_evaluation_time = robot->get_elapsed_evaluation_time();
    unsigned long frequency_time_this_module = elapsed_evaluation_time - time_at_previous_cycle[module];

    if(frequency_time_this_module > 0.0)
    {
      unsigned int number_of_modules = robot->get_number_of_modules();

      for(unsigned int j=module+1; j<number_of_modules; j++)
      {
        unsigned long frequency_time_jTH_module = elapsed_evaluation_time - time_at_previous_cycle[j];
        phase_degrees[module][j] = 360 * ((double)(frequency_time_this_module - (frequency_time_jTH_module % frequency_time_this_module)) / (double)frequency_time_this_module);
      }

      if(record_phase)
      {
        write_phase();
      }
    }
  }
}


bool OscillationAnalyzer_OutputSignal::get_record_servo(void)
{
  return(record_servo);
}


bool OscillationAnalyzer_OutputSignal::get_record_ref(void)
{
  return(record_ref);
}


bool OscillationAnalyzer_OutputSignal::get_record_trajectory(void)
{
  return(record_trajectory);
}


void OscillationAnalyzer_OutputSignal::set_estimate_amplitude_offset(const bool estimate_amplitude_offset_bool_value)
{
  estimate_amplitude_offset = estimate_amplitude_offset_bool_value;
}


void OscillationAnalyzer_OutputSignal::set_estimate_frequency(const bool estimate_frequency_bool_value)
{
  estimate_frequency = estimate_frequency_bool_value;
}


void OscillationAnalyzer_OutputSignal::set_estimate_phase(const bool estimate_phase_bool_value)
{
  estimate_phase = estimate_phase_bool_value;
}


void OscillationAnalyzer_OutputSignal::set_record_servo(const bool record_servo_bool_value)
{
  record_servo = record_servo_bool_value;

  if(record_servo_bool_value)
  {
    remove("../Evaluation_Files/servo.dat");
    servo_graph_file = new GraphFile("../Evaluation_Files/servo.dat");
  }
}


void OscillationAnalyzer_OutputSignal::set_record_ref(const bool record_ref_bool_value)
{
  record_ref = record_ref_bool_value;

  if(record_ref_bool_value)
  {
    remove("../Evaluation_Files/ref.dat");
    ref_graph_file = new GraphFile("../Evaluation_Files/ref.dat");
  }
}


void OscillationAnalyzer_OutputSignal::set_record_amplitude(const bool record_amplitude_bool_value)
{
  record_amplitude = record_amplitude_bool_value;

  if(record_amplitude_bool_value)
  {
    remove("../Evaluation_Files/amplitude.dat");
    amplitude_graph_file = new GraphFile("../Evaluation_Files/amplitude.dat");

    set_estimate_amplitude_offset(true);
  }
}


void OscillationAnalyzer_OutputSignal::set_record_offset(const bool record_offset_bool_value)
{
  record_offset = record_offset_bool_value;

  if(record_offset_bool_value)
  {
    remove("../Evaluation_Files/offset.dat");
    offset_graph_file = new GraphFile("../Evaluation_Files/offset.dat");

    set_estimate_amplitude_offset(true);
  }
}


void OscillationAnalyzer_OutputSignal::set_record_frequency(const bool record_frequency_bool_value)
{
  record_frequency = record_frequency_bool_value;

  if(record_frequency_bool_value)
  {
    remove("../Evaluation_Files/frequency.dat");
    frequency_graph_file = new GraphFile("../Evaluation_Files/frequency.dat");

    set_estimate_frequency(true);
  }
}


void OscillationAnalyzer_OutputSignal::set_record_phase(const bool record_phase_bool_value)
{
  record_phase = record_phase_bool_value;

  if(record_phase_bool_value)
  {
    remove("../Evaluation_Files/phase180.dat");
    remove("../Evaluation_Files/phase360.dat");

    phase_180_graph_file = new GraphFile("../Evaluation_Files/phase180.dat");
    phase_360_graph_file = new GraphFile("../Evaluation_Files/phase360.dat");

    set_estimate_phase(true);
  }
}


void OscillationAnalyzer_OutputSignal::set_record_trajectory(const bool record_trajectory_bool_value)
{
  record_trajectory = record_trajectory_bool_value;

  if(record_trajectory_bool_value)
  {
    remove("../Evaluation_Files/trajectory.dat");
    trajectory_graph_file = new GraphFile("../Evaluation_Files/trajectory.dat");
  }
}


unsigned int OscillationAnalyzer_OutputSignal::get_phase_graph_size(void)
{
   return phase_graph_size;
}


void OscillationAnalyzer_OutputSignal::write_servo(std::vector<double>& servo_positions)
{
  std::stringstream ss;
  unsigned int number_of_modules = robot->get_number_of_modules();

  ss << number_of_modules+1 << " ";
  ss << robot->get_elapsed_evaluation_time()/1000 << " "; // X-axis in micro seconds.

  for(unsigned int module=0; module<number_of_modules; module++)
  {
    ss << servo_positions[module] << " ";
  }
  servo_graph_file->write(ss);
}


void OscillationAnalyzer_OutputSignal::write_ref(Flood::Vector<double>& ref_positions)
{
  std::stringstream ss;
  unsigned int number_of_modules = robot->get_number_of_modules();

  ss << number_of_modules+1 << " ";
  ss << robot->get_elapsed_evaluation_time()/1000 << " "; // X-axis in micro seconds.

  for(unsigned int module=0; module<number_of_modules; module++)
  {
    ss << ref_positions[module] << " ";
  }
  ref_graph_file->write(ss);
}


void OscillationAnalyzer_OutputSignal::write_amplitude(void)
{
  std::stringstream ss;
  unsigned int number_of_modules = robot->get_number_of_modules();

  ss << number_of_modules+1 << " ";
  ss << robot->get_elapsed_evaluation_time()/1000 << " "; // X-axis in micro seconds.

  for(unsigned int module=0; module<number_of_modules; module++)
  {
    ss << amplitude_degrees[module] << " ";
  }
  amplitude_graph_file->write(ss);
}


void OscillationAnalyzer_OutputSignal::write_offset(void)
{
  std::stringstream ss;
  unsigned int number_of_modules = robot->get_number_of_modules();

  ss << number_of_modules+1 << " ";
  ss << robot->get_elapsed_evaluation_time()/1000 << " "; // X-axis in micro seconds.

  for(unsigned int module=0; module<number_of_modules; module++)
  {
    ss << offset_degrees[module] << " ";
  }
  offset_graph_file->write(ss);
}


void OscillationAnalyzer_OutputSignal::write_frequency(void)
{
  std::stringstream ss;
  unsigned int number_of_modules = robot->get_number_of_modules();

  ss << number_of_modules+1 << " ";
  ss << robot->get_elapsed_evaluation_time()/1000 << " "; // X-axis in micro seconds.

  for(unsigned int module=0; module<number_of_modules; module++)
  {
    ss << frequency_hertz[module] << " ";
  }
  frequency_graph_file->write(ss);
}


void OscillationAnalyzer_OutputSignal::write_phase(void)
{
  std::stringstream ss;
  unsigned int number_of_modules = robot->get_number_of_modules();
  double phase_value;

  ss << get_phase_graph_size()+1 << " ";
  ss << robot->get_elapsed_evaluation_time()/1000 << " "; // X-axis in micro seconds.

  for(unsigned int i=0; i<number_of_modules; i++)
  {
    for(unsigned int j=i+1; j<number_of_modules; j++)
    {
      phase_value = phase_degrees[i][j];
      if(phase_value >= 180)
      {
        phase_value = phase_value - 360;
      }
      ss << phase_value << " ";
    }
  }
  phase_180_graph_file->write(ss);

  ss.flush();
  ss << get_phase_graph_size()+1 << " ";
  ss << robot->get_elapsed_evaluation_time()/1000 << " "; //-- X-axis in micro seconds.

  for(unsigned int i=0; i<number_of_modules; i++)
  {
    for(unsigned int j=i+1; j<number_of_modules; j++)
    {
      phase_value = phase_degrees[i][j];
      ss << phase_value << " ";
    }
  }
  phase_360_graph_file->write(ss);
}


void OscillationAnalyzer_OutputSignal::write_trajectory()
{
  std::stringstream ss;

  ss << 2 << " ";
  ss << robot->get_robot_Y() << " " << robot->get_robot_X() << " ";

  trajectory_graph_file->write(ss);
}
