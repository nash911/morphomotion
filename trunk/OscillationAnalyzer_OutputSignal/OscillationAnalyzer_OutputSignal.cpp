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
  frequency_hertz.set(number_of_modules);
  phase_degrees.set(number_of_modules,number_of_modules);

  oscillation_short_history.initialize(0);
  time_at_previous_cycle.initialize(0);
  frequency_hertz.initialize(0);
  phase_degrees.initialize(0);

  phase_graph_size = 0;
  for(int i=0; i<number_of_modules; i++)
  {
    phase_graph_size = phase_graph_size + i;
  }

  estimate_frequency = false;
  estimate_phase = false;

  record_servo = false;
  record_frequency = false;
  record_phase = false;

  servo_graph_file = NULL;
  frequency_graph_file = NULL;
  phase_180_graph_file = NULL;
  phase_360_graph_file = NULL;
}


// DESTRUCTOR
OscillationAnalyzer_OutputSignal::~OscillationAnalyzer_OutputSignal(void)
{
}

void OscillationAnalyzer_OutputSignal::update_oscillation_short_history(unsigned int module, double new_oscillation_value)
{
  oscillation_short_history[module][0] = oscillation_short_history[module][1];
  oscillation_short_history[module][1] = oscillation_short_history[module][2];
  oscillation_short_history[module][2] = new_oscillation_value;

  if(estimate_phase)
  {
    calculate_phase(module);
  }

  if(estimate_frequency)
  {
    calculate_frequency(module);
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
  ss << robot->get_elapsed_evaluation_time()/1000 << " "; // X-axis in micro seconds.

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
