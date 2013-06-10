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

//#define MAX_POLL_TRIALS 10  //-- For Wired-Communication between PC and Skymega
#define MAX_POLL_TRIALS 14  //-- For XBEE-Communication between PC and Skymega

using namespace std;

class Y1ModularRobot: public Robot
{

public:

  enum MessageType{Request_ServoTime, Request_Time, None};

  //-- DEFAULT CONSTRUCTOR
  Y1ModularRobot();
  
  //-- COPY CONSTRUCTOR
  Y1ModularRobot(const Robot*);

  //-- CONSTRUCTOR WITH SERIAL PORT
  Y1ModularRobot(const std::string&);

  //-- DESTRUCTOR
  virtual ~Y1ModularRobot(void);

  //-- METHODS
  void set_default_parameters(void);
  bool set_serial_port(const std::string&, int);
  void set_elapsed_evaluation_time(unsigned long);
  void init_elapsed_evaluation_time(unsigned long);
  bool get_message(char*);
  bool get_message_with_time(char*);
  bool decode_message(const char[], vector<double>&);
  bool decode_message_with_time(const char[], vector<double>&);
  void get_current_time(void);
  std::vector<double> get_robot_XY();

  //-- INHERITED METHODS
  void copy(const Robot*);
  void reset_robot(void);
  void set_sinusoidal_controller_parameters(const vector<double>&, const vector<double>&, const vector<double>&, const double);
  void stop_sinusoidal_controller(void);
  void set_moduleServo_position(unsigned int, double);
  double get_moduleServo_position(unsigned int);
  std::vector<double> get_all_moduleServo_position(void); //-- TODO: This should be removed after implementing get_all_moduleServo_position_with_time().
  void get_all_moduleServo_position_with_time(vector<ServoFeedback*>&);
  unsigned long get_elapsed_evaluation_time(void);
  unsigned long get_previous_read_evaluation_time(void);
  double calculate_distance_travelled_euclidean(void);
  void measure_cumulative_distance(void);
  double get_robot_X(void);
  double get_robot_Y(void);
  void step(const std::string&);

private:
  std::vector<double> robot_pos_initial;
  VisualTracker *vis_track;
  unsigned int serial_port;
};

#endif
