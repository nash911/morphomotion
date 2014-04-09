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
#include <cmath>
#include <vector>

#include"ExtKalmanFilter_SinusModel.h"

//#define HISTORY_SIZE 3
//#define VELOCITY_MAX 0.26087 //-- www.es.co.th/schemetic/pdf/et-servo-s3003.pdf‎

class ServoFeedback
{

private:
  unsigned long servo_position_read_time;
  double servo_position;
  double servo_raw_position;

  //std::vector<unsigned long> read_time_history;
  //std::vector<double> position_history;

  ExtKalmanFilter_SinusModel *EKF;

public:
  ServoFeedback(void);
  ServoFeedback(const double);

  ~ServoFeedback(void);

  void reset_value(void);

  //void add_to_history(const unsigned long, const double);
  void set_new_value(const unsigned long, const double);
  void set_servo_position_read_time(const unsigned long);
  void set_servo_position(const double);

  unsigned long get_servo_position_read_time(void);
  double get_servo_position(void);
  double get_servo_raw_position(void);

  ExtKalmanFilter_SinusModel* get_ExtKalmanFilter();

};

#endif
