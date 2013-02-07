/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   Y 1   M O D U L A R   R O B O T    C L A S S   H E A D E R                                                 */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/


#ifndef Y1MODULARROBOT_H
#define Y1MODULARROBOT_H

//#include <Robot.h>
#include "Robot.h"
#include "VisualTracker.h"
#include "SerialCommunication.h"

#define BAUD_RATE 115200

using namespace std;

class Y1ModularRobot: public Robot
{

public:

  // DEFAULT CONSTRUCTOR
  Y1ModularRobot();
  
  // COPY CONSTRUCTOR
  Y1ModularRobot(const Robot*);

  // CONSTRUCTOR WITH SERIAL PORT
  Y1ModularRobot(const std::string&);

  // DESTRUCTOR
  virtual ~Y1ModularRobot(void);

  // METHODS
  void set_default_parameters(void);
  bool set_serial_port(const std::string&, int);
  bool get_message(char*);
  bool decode_message(const char[], vector<double>&);

  // INHERITED METHODS
  std::vector<double> get_robot_XY();
  void reset_robot(void);
  void set_moduleServo_position(unsigned int, double);
  double get_moduleServo_position(unsigned int);
  std::vector<double> get_all_moduleServo_position(void); // TODO: This should be removed after implementing get_all_moduleServo_position_with_time().
  void get_all_moduleServo_position_with_time(vector<ServoFeedback*>&);
  void init_elapsed_evaluation_time(void);
  unsigned long get_elapsed_evaluation_time(void);
  double calculate_distance_travelled_euclidean(void);
  void measure_cumulative_distance(void);
  unsigned long step(const std::string&);

private:
  std::vector<double> robot_pos_initial;
  VisualTracker *vis_track;
  unsigned int serial_port;
};

#endif
