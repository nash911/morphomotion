/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   S E R V O F E E D B A C K   C L A S S   H E A D E R                                                        */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#ifndef SERVOFEEDBACK_H
#define SERVOFEEDBACK_H

#include <iostream>
#include <cstdlib>

class ServoFeedback
{

private:
  unsigned long servo_position_read_time;
  double servo_position;

public:
  ServoFeedback(void);
  ~ServoFeedback(void);

  void reset_value(void);

  void set_new_value(unsigned long, double);
  void set_servo_position_read_time(unsigned long);
  void set_servo_position(double);

  unsigned long get_servo_position_read_time(void);
  double get_servo_position(void);

};

#endif
