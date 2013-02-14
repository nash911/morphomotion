/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   Y 1   M O D U L A R   R O B O T    C L A S S                                                               */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#include "Y1ModularRobot.h"


Y1ModularRobot::Y1ModularRobot():Robot()
{
  //-- Set default parameters.
  set_default_parameters();
  
  //-- Create Visual Tracker
  vis_track = new VisualTracker;
}


// DEFAULT CONSTRUCTOR
Y1ModularRobot::Y1ModularRobot(const Robot* source_object):Robot()
{
  //-- Set default parameters.
  set_default_parameters();
  
  //-- Create Visual Tracker
  vis_track = new VisualTracker;
  
  this->set_robot_type(source_object->get_robot_type());
  this->set_evaluation_method(source_object->get_evaluation_method());
  this->set_number_of_modules(source_object->get_number_of_modules());
}


Y1ModularRobot::Y1ModularRobot(const std::string& new_serial_port):Robot()
{
  //-- Set default parameters.
  set_default_parameters();

  vis_track = new VisualTracker;

  set_serial_port(new_serial_port, BAUD_RATE);
}



// DESTRUCTOR
Y1ModularRobot::~Y1ModularRobot(void)
{
}


void Y1ModularRobot::set_default_parameters(void)
{
  vis_track = NULL;
  serial_port = 0;
}


bool Y1ModularRobot::set_serial_port(const std::string& new_serial_port, int baud_rate)
{
  if(new_serial_port == "/dev/ttyUSB0")
  {
    serial_port = 16;
    if(OpenComport(serial_port, baud_rate))
    {
      serial_port = 0;
      exit(1);
    }
    else
    {
      return true;
    }
  }
  else if(new_serial_port == "/dev/ttyUSB1")
  {
    serial_port = 17;
    if(OpenComport(serial_port, baud_rate))
    {
      serial_port = 0;
      exit(1);
    }
    else
    {
      return true;
    }
  }
  else if(new_serial_port == "/dev/ttyUSB2")
  {
    serial_port = 18;
    if(OpenComport(serial_port, baud_rate))
    {
      serial_port = 0;
      exit(1);
    }
    else
    {
      return true;
    }
  }
  else if(new_serial_port == "/dev/ttyUSB3")
  {
    serial_port = 19;
    if(OpenComport(serial_port, baud_rate))
    {
      serial_port = 0;
      exit(1);
    }
    else
    {
      return true;
    }
  }
  else
  {
    std::cerr << "MorphoMotion Error: Y1ModularRobot class." << std::endl
              << "void set_serial_port(const std::string&) method." << std::endl
              << "Unknown Serial Port: " << new_serial_port << "." <<std::endl;

    exit(1);
  }
}


bool Y1ModularRobot::get_message(char* inString)
{
  unsigned char inBuf[4096];
  unsigned int inString_size = 0;
  bool string_captured = false;
  int i, n=0;

  do
  {
    n = PollComport(serial_port, inBuf, 4095);
    //usleep(10000);

    if(n == 0)
    {
      return(false);
    }
    else
    {
      if(inBuf[n-1] != '$')
      {
        //std::cout << inBuf[n-1] << std::endl;
        for(i=0;i<n;i++)
        {
          inString[inString_size] = inBuf[i];
          inString_size++;
        }
        string_captured = false;
      }
      else
      {
        for(i=0;i<n;i++)
        {
          inString[inString_size] = inBuf[i];
          inString_size++;
        }
        string_captured = true;
      }
    }
  }while(!string_captured);

  inString[inString_size] = 0;   // always put a "null" at the end of a string!

  //std::cout << "received bytes: " << inString << std::endl; // Debugger
  return(true);
}


bool Y1ModularRobot::decode_message(const char inString[], vector<double>& servo_feedback_angle)
{
  char ch = 'X';
  unsigned int n = 0;
  unsigned int module = 0;

  int servo_address = 0;
  double data[4][2];

  ch = inString[n++];

  if(ch == '#')
  {
    do
    {
      ch = inString[n++];

      if(ch == '%')
      {
        do
        {
          ch = inString[n++];
          if(ch != '%')
          {
            switch(ch)
            {
              case '2': break;
              default: {  std::cerr << std::endl
                                    << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                                    << "void decode_message(const char[], vector<double>&) method." << std::endl
                                    << "Unknown Message Type: " << ch << std::endl;
                          return(false);  } //exit(1);  }
            }
          }
        }while(ch != '%');
      }
      else if(ch == '&')
      {
        do
        {
          ch = inString[n++];
          if(ch != '&')
          {
            switch(ch)
            {
              case '0': { servo_address = 0; break; }
              case '1': { servo_address = 1; break; }
              case '2': { servo_address = 2; break; }
              case '3': { servo_address = 3; break; }
              case '4': { servo_address = 4; break; }
              default:  { std::cerr << std::endl
                                    << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                                    << "void decode_message(const char[], vector<double>&) method." << std::endl
                                    << "Invalid Servo Address: " << ch << std::endl;
                          return(false);  } //exit(1);  }
             }
          }
        }while(ch != '&');
      }
      else if(ch == '*')
      {
        do
        {
          ch = inString[n++];
          if(ch == '<')
          {
            if(ch != '*')
            {
              stringstream SS;
              do
              {
                ch = inString[n++];
                if(ch == '/')
                {
                  SS << " ";
                }
                else if(ch == '>');
                else
                {
                  SS << ch;
                }
              }while(ch != '>');

              SS >> data[module][0];
              SS >> data[module][1];
              //std::cout << "Data Stream: " << SS.str() << std::endl;
              module++;
            }
          }
        }while(ch != '*');
      }
    }while(ch != '$');
  }
  else
  {
    std::cerr << std::endl
              << "MorphoMotion Error: Y1ModularRobot class." << std::endl
              << "void decode_message(const char[], vector<double>&) method." << std::endl
              << "Unknown Message Frame Start Tag: " << ch << std::endl;
    return(false); //exit(1);
  }

  for(module = 0; module < number_of_modules; module++)
  {
    servo_feedback_angle[module] = data[module][0]/pow(10, data[module][1]);
  }

  return(true);
}


std::vector<double> Y1ModularRobot::get_robot_XY()
{
	// TODO: Dirty implementation. Clean it.
	std::vector<double> robot_XY(2);
	std::vector<int> XY(2);
  XY[0] = 0;
  XY[1] = 0;
	
  vis_track->get_realRobot_XY(XY);
  
  robot_XY[0] = XY[0];
  robot_XY[1] = XY[1];
  
   std::cout << std::endl << "X= " << XY[0] << "    Y= " << XY[1] << std::endl; // Debugger: To be removed.
  
  return(robot_XY);
}


void Y1ModularRobot::reset_robot(void)
{
  //-- Set the translation of the robot to the initial position.
  //TODO: Reset the robot to (0,0) according to image coordinate.

  //-- Set the servo of each module to zero.
  std::cout << "Number of modules: " << number_of_modules << std::endl;  // Debugger
  for(unsigned int module=0; module<number_of_modules; module++)
  {
    set_moduleServo_position(module, 0);
    std::cout << "Set module: " << module << " position to 0. " << std::endl;  // Debugger
  }

  // Capture initial position of the robot
  robot_pos_initial = get_robot_XY();

  // Initialize the controller evaluation time counter to 0;
  init_elapsed_evaluation_time();

  distance_travelled = 0;
}


void Y1ModularRobot::set_sinusoidal_controller_parameters(const vector<double>& sinusoidal_amplitude, const vector<double>& sinusoidal_offset, const vector<double>& sinusoidal_phase, const double sinusoidal_frequency)
{
  // TODO: May need to be implemented here as well.
}


void Y1ModularRobot::stop_sinusoidal_controller(void)
{
  // TODO: May need to be implemented here as well.
}


void Y1ModularRobot::set_moduleServo_position(unsigned int module, double servo_angle)
{
  stringstream outSS;
  unsigned char outBuf[100];
  unsigned int outBufSize = 0;

  //outSS << "#%0%&" << module << "&*<" << servo_angle << ">*$";
  
/***********************************************Temp Solution *******************************************************/
  // TODO: This format is a pain in the ass, and needs to be fixed to the line above, both here and in Ardino code.
  outSS << "#%0%&" << module << "&*<";
  for(unsigned int n=0; n<module; n++)
  {
		outSS << "><";
	}
	outSS << servo_angle << ">*$";
/***********************************************Temp Solution *******************************************************/

  for(int i=0;i<outSS.str().size();i++)
  {
    outBuf[i] = outSS.str()[i];
  }

  if(serial_port)
  {
    SendBuf(serial_port, outBuf, outSS.str().size());
  }
  else
  {
    std::cerr << "MorphoMotion Error: Y1ModularRobot class." << std::endl
              << "void set_moduleServo_position(int, double) method." << std::endl
              << "No Serial Port: " << serial_port << "." <<std::endl;

    exit(1);
  }
}


double Y1ModularRobot::get_moduleServo_position(unsigned int module)
{
  return 0;
}


std::vector<double> Y1ModularRobot::get_all_moduleServo_position() // TODO: This should be removed after implementing get_all_moduleServo_position_with_time().
{
  std::vector<double> servo_feedback_angle;

  if(serial_port)
  {
    unsigned char outBuf[20] = {'#','%','1','%','&','4','&','*','4','5','.','0','*','$'};
    bool message_got = false;
    bool message_decoded = false;

    do
    {
      char recvString[50];
      do
      {
        cprintf(serial_port, (char *)outBuf);

        char inString[50]; // Bug?: char inString[20]; Is this a bug? The message frame seems to be a size of approx. 42 characters long.
        message_got = get_message(inString);
        if(message_got)
        {
          //std::cout << "inString: " << inString << std::endl; // Debugger
          strcpy(recvString,inString);
          //std::cout << "recvString: " << recvString << std::endl << std::endl; // Debugger
        }
        else
        {
          flush_cport(serial_port, 1);
        }
      }while(!message_got);

      message_decoded = decode_message(recvString, servo_feedback_angle);
    }while(!message_decoded);

    //std::cout << "received bytes: " << inString << std::endl; // Debugger
  }
  else
  {
    std::cerr << std::endl
              << "Flood Error: SimEnv application." << std::endl
              << "void read_servo_position_real(void) method." << std::endl
              << "Serial port not open." << std::endl;
    exit(1);
  }

  return servo_feedback_angle;
}


void Y1ModularRobot::get_all_moduleServo_position_with_time(vector<ServoFeedback*>& servo_feedback)
{
  std::vector<double> servo_feedback_angle;
  double angle;

  for(int module=0; module<number_of_modules; module++)
  {
    // TODO:
  }
}


void Y1ModularRobot::init_elapsed_evaluation_time(void)
{
  elapsed_evaluation_time = 0;
}


unsigned long Y1ModularRobot::get_elapsed_evaluation_time(void)
{
  return(elapsed_evaluation_time);
}


double Y1ModularRobot::calculate_distance_travelled_euclidean(void)
{
  double distanceTravelled;
  return(distanceTravelled);
}


void Y1ModularRobot::measure_cumulative_distance(void)
{
}


unsigned long Y1ModularRobot::step(const std::string& type)
{
  //TODO:
  return(0);
}
