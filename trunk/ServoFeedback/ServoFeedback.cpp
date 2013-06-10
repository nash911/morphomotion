/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   S E R V O F E E D B A C K   C L A S S                                                                      */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#include "ServoFeedback.h"

ServoFeedback::ServoFeedback()
{
  servo_position_read_time = 0;
  servo_position = 0;
}


ServoFeedback::~ServoFeedback(void)
{
}


void ServoFeedback::reset_value()
{
  servo_position_read_time = 0;
  servo_position = 0;
}

void ServoFeedback::set_new_value(unsigned long current_time_value, double current_servo_position)
{
  if(current_time_value < servo_position_read_time)
  {
    std::cerr << "Morphomotion Error: ServoFeedback class." << std::endl
              << "set_new_value(unsigned long, double)" << std::endl
              << "New time value: " << current_time_value << " is less than previous time value: " << servo_position_read_time << std::endl;

    exit(1);
  }
  else
  {
    this->set_servo_position_read_time(current_time_value);
    this->set_servo_position(current_servo_position);
  }

}

void ServoFeedback::set_servo_position_read_time(unsigned long current_time_value)
{
  servo_position_read_time = current_time_value;
}


void ServoFeedback::set_servo_position(double current_servo_position)
{
  servo_position = current_servo_position;
}


unsigned long ServoFeedback::get_servo_position_read_time(void)
{
  return(servo_position_read_time);
}


double ServoFeedback::get_servo_position(void)
{
  return(servo_position);
}
