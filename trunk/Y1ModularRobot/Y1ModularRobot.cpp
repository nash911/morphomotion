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
  //vis_track = new VisualTracker;
  
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


bool Y1ModularRobot::get_message_with_time(char* inString)
{
  //unsigned char inBuf[4096];
  unsigned int inString_size = 0;
  bool string_captured = false;
  int i, n=0;
  unsigned int poll_trials = 0;

  do
  {
    unsigned char inBuf[70] = {'x','x','x','x','x','x','x','x','x','x',
                               'x','x','x','x','x','x','x','x','x','x',
                               'x','x','x','x','x','x','x','x','x','x',
                               'x','x','x','x','x','x','x','x','x','x',
                               'x','x','x','x','x','x','x','x','x','x',
                               'x','x','x','x','x','x','x','x','x','x',
                               'x','x','x','x','x','x','x','x','x','x'};

    n = PollComport(serial_port, inBuf, 4095);

    if(n > 0)
    {
      for(i=0;i<n;i++)
      {
        inString[inString_size] = inBuf[i];
        inString_size++;
      }

      if(inBuf[n-1] == '$')
      {
        string_captured = true;
      }
      else
      {
        string_captured = false;
      }
    }
    poll_trials++;
  }while(!string_captured && poll_trials < MAX_POLL_TRIALS);

  inString[inString_size] = '\0'; // always put a "null" at the end of a string!

  //std::cout << " --> Poll Counter = " << poll_trials << " --> " << inString << std::endl; // TODO: Debuggerto be removed

  return(string_captured);
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


bool Y1ModularRobot::decode_message_with_time(const char inString[], vector<double>& servo_feedback_angle)
{
  char ch = 'X';
  unsigned int n = 0;
  unsigned int module = 0;

  Y1ModularRobot::MessageType mType = None;
  int servo_address = 0;
  double data[4][2];
  unsigned long time_value=0;

  ch = inString[n++];

  // Message frame begin byte
  if(ch == '#')
  {
    do
    {
      ch = inString[n++];

      // Message-Type
      if(ch == '%')
      {
        do
        {
          ch = inString[n++];
          if(ch != '%')
          {
            switch(ch)
            {
              case '2': { mType = Request_ServoTime; break;}
              case '4': { mType = Request_Time; break;}
              default: {  std::cerr << std::endl
                                    << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                                    << "void decode_message(const char[], vector<double>&) method." << std::endl
                                    << "Unknown Message Type: " << ch << std::endl;
                          return(false);  }
            }
          }
        }while(ch != '%'); // Message-Type end byte
      }

      // Servo address
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
                          return(false);  }
             }
          }
        }while(ch != '&'); //Servo address end byte
      }

      // Time
      else if(ch == '[')
      {
        std::vector<unsigned int> time_data;
        do
        {
          ch = inString[n++];
          if(ch != ']')
          {
            switch(ch)
            {
              case '0': { time_data.push_back(0); break; }
              case '1': { time_data.push_back(1); break; }
              case '2': { time_data.push_back(2); break; }
              case '3': { time_data.push_back(3); break; }
              case '4': { time_data.push_back(4); break; }
              case '5': { time_data.push_back(5); break; }
              case '6': { time_data.push_back(6); break; }
              case '7': { time_data.push_back(7); break; }
              case '8': { time_data.push_back(8); break; }
              case '9': { time_data.push_back(9); break; }
              default:  { std::cerr << std::endl
                                    << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                                    << "void decode_message(const char[], vector<double>&) method." << std::endl
                                    << "Invalid Time value: " << ch << std::endl;
                          return(false); }
            }
          }
        }while(ch != ']'); // Time end byte

        for(int i=0; i<time_data.size(); i++)
        {
            time_value = time_value + (time_data[i]*pow(10,(time_data.size()-(i+1))));
            //std::cout << "time_data[" << i << "] = " << time_data[i] << "   time_value = " << time_value << std::endl; // TODO: Debugger to be removed.
        }
      }

      // Actual data begin byte
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
        }while(ch != '*'); // Actual data end byte
      }

    }while(ch != '$'); // End of message frame byte
  }
  else
  {
    std::cerr << std::endl
              << "MorphoMotion Error: Y1ModularRobot class." << std::endl
              << "void decode_message(const char[], vector<double>&) method." << std::endl
              << "Unknown Message Frame Start Tag: " << ch << std::endl;
    return(false);
  }

  if(mType == Request_Time)
  {
    init_elapsed_evaluation_time(time_value);
  }
  else if(mType == Request_ServoTime)
  {
    set_elapsed_evaluation_time(time_value);

    for(module = 0; module < number_of_modules; module++)
    {
      servo_feedback_angle[module] = data[module][0]/pow(10, data[module][1]);
    }
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
	
  if(vis_track!=NULL)
  {
    vis_track->get_realRobot_XY(XY);
  }
  
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

  // Initialize the controller evaluation time counter to the current time of Skymega;
#ifdef Y1_CONFIGURATION
  get_current_time();
#endif


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
    unsigned char outBuf[20] = {'#','%','1','%','&','4','&','*','4','5','.','0','*','$'};  // To request current servo position of all modules [PC to Skymega].
    bool message_got = false;
    bool message_decoded = false;

    do
    {
      char recvString[70];
      do
      {
        cprintf(serial_port, (char *)outBuf); // Send message requesting currnt servo position of all modules to Skymega.

        char inString[70]; // Bug?: char inString[20]; Is this a bug? The message frame seems to be a size of approx. 42 characters long.
        message_got = get_message(inString);
        //message_got = get_message_with_time(inString);
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
    servo_feedback_angle.resize(number_of_modules);

    if(serial_port)
    {
      unsigned char outBuf[20] = {'#','%','1','%','&','4','&','*','4','5','.','0','*','$'};  // To request current servo position of all modules [PC to Skymega].
      bool message_got = false;
      bool message_decoded = false;

      do
      {
        char recvString[70];
        do
        {
          cprintf(serial_port, (char *)outBuf); // Send message requesting currnt servo position of all modules to Skymega.

          char inString[70]; // Bug?: char inString[20]; Is this a bug? The message frame seems to be a size of approx. 42 characters long.
          message_got = get_message_with_time(inString);
          if(message_got)
          {
            strcpy(recvString,inString);
          }
          else
          {
            flush_cport(serial_port, 1);
          }
        }while(!message_got);

        message_decoded = decode_message_with_time(recvString, servo_feedback_angle);
      }while(!message_decoded);

      //std::cout << "                                             "; // TODO: Debugger to be removed
      for(int module=0; module<number_of_modules; module++)
      {
        servo_feedback[module]->set_new_value(elapsed_evaluation_time, servo_feedback_angle[module]);
        //std::cout << servo_feedback_angle[module] << "   "; // TODO: Debugger to be removed
      }
      //std::cout << std::endl; // TODO: Debugger to be removed
    }
    else
    {
      std::cerr << std::endl
                << "Flood Error: SimEnv application." << std::endl
                << "void read_servo_position_real(void) method." << std::endl
                << "Serial port not open." << std::endl;
      exit(1);
    }
}


void Y1ModularRobot::get_current_time(void)
{
    std::vector<double> servo_feedback_angle;

    if(serial_port)
    {
      unsigned char outBuf[20] = {'#','%','3','%','&','4','&','$'};  // To request current time [PC to Skymega]. // TODO: 'To' address [&4&] should not be manditory. This should be changed in Arduino code as well.
      bool message_got = false;
      bool message_decoded = false;

      do
      {
        char recvString[40];
        do
        {
          cprintf(serial_port, (char *)outBuf); // Send message requesting currnt servo position of all modules to Skymega.

          char inString[40];
          message_got = get_message_with_time(inString);
          if(message_got)
          {
            strcpy(recvString,inString);
          }
          else
          {
            flush_cport(serial_port, 1);
          }
        }while(!message_got);

        message_decoded = decode_message_with_time(recvString, servo_feedback_angle);
      }while(!message_decoded);
    }
    else
    {
      std::cerr << std::endl
                << "Flood Error: SimEnv application." << std::endl
                << "void get_current_time(void) method." << std::endl
                << "Serial port not open." << std::endl;
      exit(1);
    }
}


void Y1ModularRobot::init_elapsed_evaluation_time(unsigned long new_initial_evaluation_time)
{
  initial_evaluation_time = new_initial_evaluation_time;
}


void Y1ModularRobot::set_elapsed_evaluation_time(unsigned long new_elapsed_evaluation_time)
{
  elapsed_evaluation_time = new_elapsed_evaluation_time - initial_evaluation_time;
  //elapsed_evaluation_time = new_elapsed_evaluation_time;

  //std::cout << "new_elapsed_evaluation_time: -->                 " << new_elapsed_evaluation_time << std::endl; // TODO: Debugger to be removed.
  //std::cout << "initial_evaluation_time: -->                 " << initial_evaluation_time << std::endl; // TODO: Debugger to be removed.
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


double Y1ModularRobot::get_robot_X(void)
{
  //-- To be implemnted
  return(0.0);
}


double Y1ModularRobot::get_robot_Y(void)
{
    //-- To be implemnted
    return(0.0);
}

unsigned long Y1ModularRobot::step(const std::string& type)
{
  //TODO:
  return(0);
}
