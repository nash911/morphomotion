/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   O U T P U T S I G N A L   O S C I L L A T I O N A N A L Y Z E R   C L A S S   H E A D E R                  */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/


#ifndef OSCILLATIONANALYZER_OUTPUTSIGNAL_H
#define OSCILLATIONANALYZER_OUTPUTSIGNAL_H

#include <vector>
#include <sstream>

#include "Robot.h"
#include "GraphFile.h"
#include "Vector.h"
#include "Matrix.h"

class OscillationAnalyzer_OutputSignal
{
private:
  Robot *robot;
  unsigned int phase_graph_size;

  Flood::Matrix<double> oscillation_short_history; // Size(number_of_modules, 3)
  Flood::Vector<unsigned long> time_at_previous_cycle; // Size(number_of_modules);
  Flood::Vector<double> amplitude_degrees; // Size(number_of_modules);
  Flood::Vector<double> offset_degrees; // Size(number_of_modules);
  Flood::Vector<double> frequency_hertz; // Size(number_of_modules);
  Flood::Matrix<double> phase_degrees; // Size(number_of_modules, number_of_modules);

  bool estimate_amplitude_offset;
  bool estimate_frequency;
  bool estimate_phase;

  bool record_servo;
  bool record_servo_raw;
  bool record_ref;
  bool record_amplitude;
  bool record_offset;
  bool record_frequency;
  bool record_phase;
  bool record_trajectory;

  GraphFile *servo_graph_file;
  GraphFile *servo_raw_graph_file;
  GraphFile *ref_graph_file;
  GraphFile *amplitude_graph_file;
  GraphFile *offset_graph_file;
  GraphFile *frequency_graph_file;
  GraphFile *phase_180_graph_file;
  GraphFile *phase_360_graph_file;
  GraphFile *trajectory_graph_file;

public:
  OscillationAnalyzer_OutputSignal(Robot*);
  ~OscillationAnalyzer_OutputSignal(void);

  void update_oscillation_short_history(unsigned int, double);
  void calculate_amplitude_offset(unsigned int);
  void calculate_phase(unsigned int);
  void calculate_frequency(unsigned int);

  bool get_record_servo(void);
  bool get_record_servo_raw(void);
  bool get_record_ref(void);
  bool get_record_trajectory(void);

  void set_record_servo(const bool);
  void set_record_servo_raw(const bool);
  void set_record_ref(const bool);
  void set_estimate_amplitude_offset(const bool);
  void set_record_amplitude(const bool);
  void set_record_offset(const bool);
  void set_estimate_frequency(const bool);
  void set_record_frequency(const bool);
  void set_estimate_phase(const bool);
  void set_record_phase(const bool);
  void set_record_trajectory(const bool);

  unsigned int get_phase_graph_size(void);

  void write_servo(std::vector<double>&);
  void write_servo_raw(std::vector<double>&);
  void write_ref(Flood::Vector<double>&);
  void write_amplitude(void);
  void write_offset(void);
  void write_frequency(void);
  void write_phase(void);
  void write_trajectory(void);
};

#endif
