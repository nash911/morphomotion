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
  servo_raw_position = 0;

  EKF = NULL;
}


ServoFeedback::ServoFeedback(const double delta_time)
{
  servo_position_read_time = 0;
  servo_position = 0;
  servo_raw_position = 0;

  if(delta_time)
  {
    EKF = new ExtKalmanFilter_SinusModel(delta_time);
  }
  else
  {
    EKF = NULL;
  }
}


ServoFeedback::~ServoFeedback(void)
{
  if(EKF != NULL)
  {
    delete EKF;
  }
}


void ServoFeedback::reset_value()
{
  servo_position_read_time = 0;
  servo_position = 0;
  servo_raw_position = 0;

  //read_time_history.clear();
  //position_history.clear();
}


/*void ServoFeedback::add_to_history(const unsigned long current_time_value, const double current_servo_position)
{
  if(current_time_value < servo_position_read_time)
  {
    std::cerr << "Morphomotion Error: ServoFeedback class." << std::endl
              << "add_to_history(unsigned long, double)" << std::endl
              << "New time value: " << current_time_value << " is less than previous time value: " << servo_position_read_time << std::endl;
    exit(1);
  }
  else
  {
    if(read_time_history.size() > HISTORY_SIZE)
    {
      std::cerr << "Morphomotion Error: ServoFeedback class." << std::endl
                << "add_to_history(unsigned long, double)" << std::endl
                << "Size of vector read_time_history: " << read_time_history.size() << " is greated than HISTORY_SIZE: " << HISTORY_SIZE << std::endl;
      exit(1);
    }
    else if(read_time_history.size() == HISTORY_SIZE)
    {
      //double velocity = (fabs(current_servo_position - position_history.back())/(double)(current_time_value - read_time_history.back())) * 1000.0;
      //if(velocity <= VELOCITY_MAX)
      {
        read_time_history.erase(read_time_history.begin());
        position_history.erase(position_history.begin());

        read_time_history.push_back(current_time_value);
        position_history.push_back(current_servo_position);

        //std::cout << "Pass: " << velocity << std::endl; // TODO: Debugger
      }
      //else
      {
        //std::cout << "Fail: " << velocity << std::endl; // TODO: Debugger
      }
    }
    else
    {
        read_time_history.push_back(current_time_value);
        position_history.push_back(current_servo_position);
    }

    this->set_new_value(read_time_history.back(),position_history.back());
  }
}*/


void ServoFeedback::set_new_value(const unsigned long current_time_value, const double current_servo_position)
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


void ServoFeedback::set_servo_position_read_time(const unsigned long current_time_value)
{
  servo_position_read_time = current_time_value;
}


void ServoFeedback::set_servo_position(const double current_servo_position)
{
  if(EKF == NULL)
  {
    servo_raw_position = current_servo_position;
    servo_position = current_servo_position;
  }
  else
  {
    servo_raw_position = current_servo_position;
    servo_position = EKF->predict_and_update(current_servo_position, (double)servo_position_read_time/1000000.0);
    //servo_position = current_servo_position;

    //std::cout << servo_raw_position << " " << servo_position << " "; //TODO: Debugger to be removed.
  }

}


unsigned long ServoFeedback::get_servo_position_read_time(void)
{
  return(servo_position_read_time);
}


double ServoFeedback::get_servo_position(void)
{
  return(servo_position);
}


double ServoFeedback::get_servo_raw_position(void)
{
  return(servo_raw_position);
}


ExtKalmanFilter_SinusModel* ServoFeedback::get_ExtKalmanFilter()
{
  return EKF;
}
