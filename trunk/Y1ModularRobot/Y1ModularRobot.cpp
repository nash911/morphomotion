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

void changemode(int);
int  kbhit(void);
bool operator==(const std::vector<double>&, const std::vector<double>&);


// DEFAULT CONSTRUCTOR
Y1ModularRobot::Y1ModularRobot():Robot()
{
  //-- Set default parameters.
  set_default_parameters();

  //-- Create Visual Tracker
  //vis_track = new VisualTracker;
}


// COPY CONSTRUCTOR
Y1ModularRobot::Y1ModularRobot(const Robot* source_object):Robot()
{
  if(source_object == NULL)
  {
    std::cerr << "MorphoMotion Error: Y1ModularRobot class." << std::endl
              << "Y1ModularRobot(const Robot*) method." << std::endl
              << "Source object pointer = NULL" <<std::endl;

    exit(1);
  }

  //-- Set default parameters.
  set_default_parameters();

  //-- Create Visual Tracker
  vis_track = new VisualTracker;

  this->set_robot_type(source_object->get_robot_type());
  this->set_evaluation_method(source_object->get_evaluation_method());
  this->set_number_of_modules(source_object->get_number_of_modules());

  //-- Set robot priority
  this->set_robot_priority("Robot_Secondary");
}


Y1ModularRobot::Y1ModularRobot(const std::string& new_serial_port):Robot()
{
  //-- Set default parameters.
  set_default_parameters();

  //-- Create Visual Tracker
  vis_track = new VisualTracker;

  set_serial_port(new_serial_port, BAUD_RATE);
}



// DESTRUCTOR
Y1ModularRobot::~Y1ModularRobot(void)
{
    if(vis_track != NULL)
    {
        delete vis_track;
    }
}


void Y1ModularRobot::copy(const Robot* source_object)
{
  if(source_object == NULL)
  {
    std::cerr << "MorphoMotion Error: Y1ModularRobot class." << std::endl
              << "void copy(const Robot*) method." << std::endl
              << "Source object pointer = NULL" <<std::endl;

    exit(1);
  }

  this->set_robot_type(source_object->get_robot_type());
  this->set_evaluation_method(source_object->get_evaluation_method());
  this->set_number_of_modules(source_object->get_number_of_modules());

  //-- Set robot priority
  this->set_robot_priority("Robot_Secondary");
}


/*bool contains(std::vector<char> V, char c)
{
  for(unsigned int i=0; i<V.size(); i++)
  {
    if(V[i] == c)
    {
      return true;
    }
  }

  return false;
}*/


unsigned int contains(std::vector<char> V, char c)
{
  unsigned int position = 0;

  for(unsigned int i=0; i<V.size(); i++)
  {
    if(V[i] == c)
    {
      return i+1;
    }
  }

  return position;
}


std::string toString(std::vector<char> V)
{
  std::stringstream ss;

  for(unsigned int i=0; i<V.size(); i++)
  {
      ss << V[i];
  }
  return ss.str();
}


long long timeval_diff(struct timeval *difference,
                       struct timeval *end_time,
                       struct timeval *start_time
                      )
{
  struct timeval temp_diff;

  if(difference==NULL)
  {
    difference=&temp_diff;
  }

  difference->tv_sec =end_time->tv_sec -start_time->tv_sec ;
  difference->tv_usec=end_time->tv_usec-start_time->tv_usec;

  /* Using while instead of if below makes the code slightly more robust. */

  while(difference->tv_usec<0)
  {
    difference->tv_usec+=1000000;
    difference->tv_sec -=1;
  }

  return 1000000LL*difference->tv_sec+
                   difference->tv_usec;
}


void Y1ModularRobot::set_default_parameters(void)
{
  vis_track = NULL;
  serial_port = 0;
  this->set_robot_environment("Y1");
  previous_elapsed_evaluation_time = 0;
  updating_elapsed_evaluation_time = false;
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


//-- Compatible with Arduino code --> servo_controller_charArray_V6.pde
bool Y1ModularRobot::get_message_with_time(char* inString)
{
  //unsigned char inBuf[4096];
  unsigned int inString_size = 0;
  bool string_captured = false;
  int i, n=0;
  unsigned int poll_trials = 0;

  do
  {
    unsigned char inBuf[100] = {'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x'};

    n = PollComport(serial_port, inBuf, 100);

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

  //std::cout << " Time --> " << inString << std::endl; // TODO: Debuggerto be removed

  return(string_captured);
}


//-- Compatible with Arduino code --> servo_controller_charArray_V10.pde
bool Y1ModularRobot::get_message_with_time_THREAD(std::vector<char>& inString)
{
  unsigned int inString_size = 0;
  bool string_captured = false;
  int i, n=0;
  unsigned int poll_trials = 0;

  do
  {
    unsigned char inBuf[200] = {'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x',
                                'x','x','x','x','x','x','x','x','x','x'};
    n = PollComport(serial_port, inBuf, 200);

    if(n > 0)
    {
      for(unsigned int j=0; j<n; j++)
      {
        inString.push_back(inBuf[j]);
      }

      if(contains(inString,'#') && contains(inString,'$'))
      {
        string_captured = true;
        break;
      }
    }
    poll_trials++;
  }while(!string_captured && poll_trials < MAX_POLL_TRIALS);

  if(poll_trials >= MAX_POLL_TRIALS)
  {
      std::cout << "Reached MAX_POLL_TRIALS in get_message_with_time_THREAD(std::vector<char>&)." << std::endl; // TODO: Debuggerto be removed
  }

  //std::cout << " --> Poll Counter = " << poll_trials << " --> " << toString(inString) << std::endl; // TODO: Debuggerto be removed

  /*std::cout << " ";
  for(unsigned int i=0; i<inString.size(); i++)
  {
      if(inString[i] == '[')
      {
          for(unsigned int j=i+1; j<inString.size(); j++)
          {
              if(inString[j] == ']')
              {
                  break;
              }
              std::cout << inString[j];
          }
      }
      else if(inString[i] == ']')
      {
          break;
      }
  }
  std::cout << " " << std::endl;*/

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


//-- Compatible with Arduino code --> servo_controller_charArray_V6.pde
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
              case '2': { mType = Requested_ServoTime; break;}
              case '4': { mType = Requested_Time; break;}
              default: {  std::cerr << std::endl
                                    << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                                    << "bool decode_message_with_time(const char[], vector<double>&) method." << std::endl
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
                                    << "bool decode_message_with_time(const char[], vector<double>&) method." << std::endl
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
                                    << "bool decode_message_with_time(const char[], vector<double>&) method." << std::endl
                                    << "Invalid Time value: " << ch << std::endl;
                          return(false); }
            }
          }
        }while(ch != ']'); // Time end byte

        for(unsigned int i=0; i<time_data.size(); i++)
        {
            time_value = time_value + (time_data[i]*pow(10,(time_data.size()-(i+1))));
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
              << "bool decode_message_with_time(const char[], vector<double>&) method." << std::endl
              << "Unknown Message Frame Start Tag: " << ch << std::endl;
    return(false);
  }

  if(mType == Requested_Time)
  {
    init_elapsed_evaluation_time(time_value);
  }
  else if(mType == Requested_ServoTime)
  {
    set_elapsed_evaluation_time(time_value);

    for(module = 0; module < number_of_modules; module++)
    {
      servo_feedback_angle[module] = data[module][0]/pow(10, data[module][1]);
    }
  }

  return(true);
}


//-- Compatible with Arduino code --> servo_controller_charArray_V6.pde
bool Y1ModularRobot::decode_message_with_individual_time(const char inString[], unsigned long& time, vector<double>& servo_feedback_angle, vector<long>& servo_read_time)
{
  char ch = 'X';
  unsigned int n = 0;
  unsigned int module = 0;

  Y1ModularRobot::MessageType mType = None;
  int servo_address = 0;
  double data[4][2];
  int base10Exp = 2; //-- This parameter has to be the same both here and in Arduino code [int OutputStringStream_All(int, unsigned char*)].
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
              case '2': { mType = Requested_ServoTime; break;}
              case '4': { mType = Requested_Time; break;}
              default: {
#ifdef COMMUNICATION_DEBUGGER
                          std::cerr << std::endl
                                    << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                                    << "bool decode_message_with_individual_time(const char[], vector<double>&) method." << std::endl
                                    << "Unknown Message Type: " << ch << std::endl;
#endif
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
              default:  {
#ifdef COMMUNICATION_DEBUGGER
                          std::cerr << std::endl
                                    << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                                    << "bool decode_message_with_individual_time(const char[], vector<double>&) method." << std::endl
                                    << "Invalid Servo Address: " << ch << std::endl;
#endif
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
              default:  {
#ifdef COMMUNICATION_DEBUGGER
                          std::cerr << std::endl
                                    << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                                    << "bool decode_message_with_individual_time(const char[], vector<double>&) method." << std::endl
                                    << "Invalid Time value: " << ch << std::endl;
#endif
                          return(false); }
            }
          }
        }while(ch != ']'); // Time end byte

        for(unsigned int i=0; i<time_data.size(); i++)
        {
            time_value = time_value + (time_data[i]*pow(10,(time_data.size()-(i+1))));
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
#ifdef COMMUNICATION_DEBUGGER
    std::cerr << std::endl
              << "MorphoMotion Error: Y1ModularRobot class." << std::endl
              << "bool decode_message_with_individual_time(const char[], vector<double>&) method." << std::endl
              << "Unknown Message Frame Start Tag: " << ch << std::endl;
#endif
    return(false);
  }

  if(mType == Requested_Time)
  {
    if(!init_elapsed_evaluation_time(time_value))
    {
      return false;
    }
  }
  else if(mType == Requested_ServoTime)
  {
    //set_elapsed_evaluation_time(time_value);
    time = time_value;

    for(module = 0; module < number_of_modules; module++)
    {
      servo_feedback_angle[module] = data[module][0]/pow(10, base10Exp);
      servo_read_time[module] = data[module][1];
    }
  }

  return(true);
}


//-- Compatible with Arduino code --> servo_controller_charArray_V11.pde
Y1ModularRobot::MessageType Y1ModularRobot::decode_message_with_ackn(const char inString[])
{
  char ch = 'X';
  unsigned int n = 0;
  Y1ModularRobot::MessageType mType = None;

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
              case '2': { mType = Requested_ServoTime; break;}
              case '4': { mType = Requested_Time; break;}
              case '7': { mType = Turn_On_Broadcast_Ackn; break;}
              case '8': { mType = Turn_Off_Broadcast_Ackn; break;}
              default: {
#ifdef COMMUNICATION_DEBUGGER
                          std::cout << inString << std::endl;
                          std::cerr << std::endl
                                    << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                                    << "Y1ModularRobot::MessageType Y1ModularRobot::decode_message_with_ackn(const char[]) method." << std::endl
                                    << "Unknown Message Type: " << ch << std::endl;
#endif
                          return(None);  }
            }
          }
        }while(ch != '%' && ch != '$'); // Message-Type end byte
      }
    }while(ch != '$'); // End of message frame byte
  }
  else
  {
#ifdef COMMUNICATION_DEBUGGER
    std::cout << inString << std::endl;
    std::cerr << std::endl
              << "MorphoMotion Error: Y1ModularRobot class." << std::endl
              << "Y1ModularRobot::MessageType Y1ModularRobot::decode_message_with_ackn(const char[]) method." << std::endl
              << "Unknown Message Frame Start Tag: " << ch << std::endl;
#endif
    return(None);
  }

  return(mType);
}


//-- Compatible with Arduino code --> servo_controller_charArray_V10.pde
bool Y1ModularRobot::decode_message_with_individual_time_V2(const char inString[], unsigned long& time, vector<double>& servo_feedback_angle, vector<long>& servo_read_time)
{
  char ch = 'X';
  unsigned int n = 0;
  unsigned int module = 0;

  Y1ModularRobot::MessageType mType = None;
  int servo_address = -1;
  std::vector<double> data;
  std::vector<unsigned int> data_time;
  std::vector<bool> data_available;
  std::vector<bool> data_time_available;
  int base10Exp = 2; //-- This parameter has to be the same both here and in Arduino code [int OutputStringStream_All(int, unsigned char*)].
  unsigned long time_value=0;
  bool time_available = false;

  ch = inString[n++];
  double D;
  unsigned int UI;

  //std::cout << inString << std::endl;

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
              case '2': { mType = Requested_ServoTime; break;}
              case '4': { mType = Requested_Time; break;}
              default: {
#ifdef COMMUNICATION_DEBUGGER
                          std::cout << inString << std::endl;
                          std::cerr << std::endl
                                    << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                                    << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                                    << "Unknown Message Type: " << ch << std::endl;
#endif
                          return(false);
                        }
            }
          }
        }while(ch != '%' && ch != '$'); // Message-Type end byte OR End of message frame byte
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
              default:  {
#ifdef COMMUNICATION_DEBUGGER
                          std::cout << inString << std::endl;
                          std::cerr << std::endl
                                    << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                                    << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                                    << "Invalid Servo Address: " << ch << std::endl;
#endif
                          return(false);  }
             }
          }
        }while(ch != '&' && ch != '$'); //Servo address end byte OR End of message frame byte
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
              default:  {
#ifdef COMMUNICATION_DEBUGGER
                          std::cout << inString << std::endl;
                          std::cerr << std::endl
                                    << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                                    << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                                    << "Invalid Time value: " << ch << std::endl;
#endif
                          return(false); }
            }
          }
          else if(ch == ']')
          {
            if(time_data.size() > 0)
            {
              time_available = true;
            }
          }
        }while(ch != ']'  && ch != '$'); // Time end byte OR End of message frame byte

        if(time_available)
        {
          for(unsigned int i=0; i<time_data.size(); i++)
          {
            time_value = time_value + (time_data[i]*pow(10,(time_data.size()-(i+1))));
          }
        }
        else
        {
#ifdef COMMUNICATION_DEBUGGER
            std::cout << inString << std::endl;
            std::cerr << std::endl
                    << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                    << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                    << "Time not available." << std::endl;
#endif
          return(false);
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
            data_time_available.push_back(false);
            data_available.push_back(false);

            stringstream SS;
            do
            {
              ch = inString[n++];
              if(ch == '/')
              {
                if(!data_time_available[module])
                {
                  SS << " ";
                  data_time_available[module] = true;
                }
                else
                {
#ifdef COMMUNICATION_DEBUGGER
                  std::cout << inString << std::endl;
                  std::cerr << std::endl
                            << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                            << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                            << "Double '/' found in Module: " << module+1 << " data." << std::endl;
#endif
                  return(false);
                }
              }
              else if(ch == '>')
              {
                data_available[module] = true;
              }
              else if(ch == '-' || (ch >= '0' && ch <= '9'))
              {
                SS << ch;
              }
              else
              {
#ifdef COMMUNICATION_DEBUGGER
                std::cout << inString << std::endl;
                std::cerr << std::endl
                          << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                          << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                          << "Unknown data Byte: " << ch << " in Module: " << module+1 << std::endl;
#endif
                return(false);
              }
            }while(ch != '>'  && ch != '$'); // Single data end byte OR End of message frame byte

            if(data_available[module])
            {
              SS >> D;
              data.push_back(D);
            }
            else
            {
              data.push_back(0);
#ifdef COMMUNICATION_DEBUGGER
              std::cout << inString << std::endl;
              std::cerr << std::endl
                        << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                        << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                        << "Data not available for Module: " << module+1 << std::endl;
#endif
              return(false);
            }

            if(data_time_available[module])
            {
              SS >> UI;
              data_time.push_back(UI);
            }
            else
            {
              data_time.push_back(0);
            }

            module++;
            if(module >= 5)
            {
#ifdef COMMUNICATION_DEBUGGER
              std::cout << inString << std::endl;
              std::cerr << std::endl
                        << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                        << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                        << "Data frame overflow." << std::endl;
#endif
              return(false);
            }
          }

          else if(ch != '*' || ch != '$'); // Do nothing

          else
          {
#ifdef COMMUNICATION_DEBUGGER
            std::cout << inString << std::endl;
            std::cerr << std::endl
                      << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                      << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                      << "Unknown Data start tag in Module: " << module+1 << std::endl;
#endif
            return(false);
          }
        }while(ch != '*'  && ch != '$'); // Data frame end byte OR End of message frame byte
      }

      else if(ch == '$');

      else
      {
#ifdef COMMUNICATION_DEBUGGER
        std::cout << inString << std::endl;
        std::cerr << std::endl
                  << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                  << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                  << "Unknown Byte: " << ch << std::endl;
#endif
        return(false);
      }

    }while(ch != '$'); // End of message frame byte
  }
  else
  {
#ifdef COMMUNICATION_DEBUGGER
    std::cout << inString << std::endl;
    std::cerr << std::endl
              << "MorphoMotion Error: Y1ModularRobot class." << std::endl
              << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
              << "Unknown Message Frame Start Tag: " << ch << std::endl;
#endif
    return(false);
  }

  if(mType == Requested_Time)
  {
    if(time_available)
    {
      init_elapsed_evaluation_time(time_value);
    }
  }
  else if(mType == Requested_ServoTime)
  {
    if(servo_address == -1)
    {
#ifdef COMMUNICATION_DEBUGGER
      std::cout << inString << std::endl;
      std::cerr << std::endl
                << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                << "Servo Address not available." << std::endl;
#endif
      return(false);
    }
    else if(servo_address == 4)
    {
      if(data.size() != 4 || data_time.size() != 4)
      {
#ifdef COMMUNICATION_DEBUGGER
        std::cout << inString << std::endl;
        std::cerr << std::endl
                  << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                  << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                  << "Invalid number of data. Data: " << data.size() << "    Data_Time: " << data_time.size() << std::endl;
#endif
        return(false);
      }
    }

    if(time_available)
    {
      if(time_value - get_initial_evaluation_time() <= get_elapsed_evaluation_time())
      {
#ifdef COMMUNICATION_DEBUGGER
        std::cout << inString << std::endl;
        std::cerr << std::endl
                  << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                  << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                  << "New Time value: " << time_value << " is LESS than or EQUAL to previous time value: " << get_initial_evaluation_time() + get_elapsed_evaluation_time() << std::endl;
#endif
        return(false);
      }
      /*else if(time_value - get_initial_evaluation_time() - get_elapsed_evaluation_time() >= 100000)
      {
#ifdef COMMUNICATION_DEBUGGER
        std::cout << inString << std::endl;
        std::cerr << std::endl
                  << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                  << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                  << "New Time value: " << time_value << " is Much GREATER that the elapsed evaluation time." << std::endl;
#endif
        return(false);
      }*/
      else
      {
        time = time_value;
      }
    }
    else
    {
#ifdef COMMUNICATION_DEBUGGER
      std::cout << inString << std::endl;
      std::cerr << std::endl
                << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                << "Time value NOT available." << std::endl;
#endif
      return(false);
    }

    for(module = 1; module < data_time.size() ; module++)
    {
      if(data_time[module-1] > data_time[module])
      {
#ifdef COMMUNICATION_DEBUGGER
        std::cout << inString << std::endl;
        std::cerr << std::endl
                  << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                  << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
                  << "Data Time of Module: " << module << " is greater than Data Time of Module: " << module+1 << std::endl;
#endif
        return(false);
      }
    }

    for(module = 0; module < number_of_modules; module++)
    {
      servo_feedback_angle[module] = 0;
      servo_read_time[module] = 0;
    }

    for(module = 0; module < number_of_modules && module < data.size(); module++)
    {
      servo_feedback_angle[module] = data[module]/pow(10, base10Exp);
      servo_read_time[module] = data_time[module];
    }
  }
  else
  {
#ifdef COMMUNICATION_DEBUGGER
    std::cout << inString << std::endl;
    std::cerr << std::endl
              << "MorphoMotion Error: Y1ModularRobot class." << std::endl
              << "bool decode_message_with_individual_time_V2(const char[], vector<double>&) method." << std::endl
              << "Message Type not available." << std::endl;
#endif
    return(false);
  }

  return(true);
}


//-- Compatible with Arduino code --> servo_controller_charArray_V6.pde
/*bool Y1ModularRobot::decode_message_with_individual_time_THREAD(const char inString[], unsigned long& time, vector<double>& servo_feedback_angle, vector<long>& servo_read_time)
{
  char ch = 'X';
  unsigned int n = 0;
  unsigned int module = 0;

  Y1ModularRobot::MessageType mType = None;
  int servo_address = 0;
  double data[4][2];
  int base10Exp = 2; //-- This parameter has to be the same both here and in Arduino code [int OutputStringStream_All(int, unsigned char*)].
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
              case '2': { mType = Requested_ServoTime; break;}
              case '4': { mType = Requested_Time; break;}
              default: {  std::cerr << std::endl
                                    << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                                    << "bool decode_message_with_individual_time_THREAD(const char[], vector<double>&) method." << std::endl
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
                                    << "bool decode_message_with_individual_time_THREAD(const char[], vector<double>&) method." << std::endl
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
                                    << "bool decode_message_with_individual_time_THREAD(const char[], vector<double>&) method." << std::endl
                                    << "Invalid Time value: " << ch << std::endl;
                          return(false); }
            }
          }
        }while(ch != ']'); // Time end byte

        for(unsigned int i=0; i<time_data.size(); i++)
        {
            time_value = time_value + (time_data[i]*pow(10,(time_data.size()-(i+1))));
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
              << "bool decode_message_with_individual_time_THREAD(const char[], vector<double>&) method." << std::endl
              << "Unknown Message Frame Start Tag: " << ch << std::endl;
    return(false);
  }

  if(mType == Requested_Time)
  {
    init_elapsed_evaluation_time(time_value);
  }
  else if(mType == Requested_ServoTime)
  {
    time = time_value;

    for(module = 0; module < number_of_modules; module++)
    {
      servo_feedback_angle[module] = data[module][0]/pow(10, base10Exp);
      servo_read_time[module] = data[module][1];
    }
  }

  return(true);
}*/


std::vector<double> Y1ModularRobot::get_robot_XY(const std::string& phase)
{
  std::vector<double> robot_XY(2);

  double x = 1.1;
  double y = 2.2;
  double z = 3.3;

  bool got_robot_position_success = false;

  unsigned int key = 'H';
  changemode(1);

  if(vis_track!=NULL)
  {
    unsigned int vtCounter = 0;

    /*do
    {
        do
        {
          //std::cout << std::endl << vtCounter++ << std::endl; // TODO: Debugger to be removed
          got_robot_position_success = vis_track->get_robot_3D_position_rectfied(phase, x, y, z);
          vtCounter++;
        }while(!got_robot_position_success); //&& vtCounter < 1);

        std::cout << "Approve? " << std::endl;
        do
        {
            while(!kbhit());
            key = getchar();
        }while(!(key==ENTER || key==ESC)); //--Do until either ENTER or ESC is pressed.

        if(key == ENTER)
        {
            std::cout << "    Yes" << std::endl;
        }
        else if(key == ESC)
        {
            std::cout << "    No" << std::endl;
        }
    }while(key == ESC); //--DO if ESC is pressed.
    */

    do
    {
      //std::cout << std::endl << vtCounter++ << std::endl; // TODO: Debugger to be removed
      got_robot_position_success = vis_track->get_robot_3D_position_rectfied(phase, x, y, z);

      if(vtCounter>=VTRACKER_LIMIT && !got_robot_position_success && phase == "Reset")
      {
        vtCounter = 0;
        displace_robot();
        reset_modules();
      }
      else
      {
        vtCounter++;
      }
    }while(!got_robot_position_success); //&& vtCounter < 1);
  }

  robot_XY[0] = x;
  robot_XY[1] = y;

  changemode(0);
  return(robot_XY);
}


void Y1ModularRobot::reset_robot(void)
{
  //-- Set the translation of the robot to the initial position.
  //TODO: Reset the robot to (0,0) according to image coordinate.

  turn_off_broadcast(10000);

  previous_control_signal.resize(number_of_modules,0);

  reset_modules();

  if(this->robot_priority == Robot_Primary)
  {
    // Capture initial position of the robot
    robot_pos_initial = get_robot_XY("Reset");
  }

  if(this->robot_priority == Robot_Primary)
  {
    get_current_time();
  }

  distance_travelled = 0;
  comm_fail_counter = 0;
}


void Y1ModularRobot::reset_modules(void)
{
    //-- Set the servo of each module to zero.
    for(unsigned int module=0; module<number_of_modules; module++)
    {
      set_moduleServo_position(module, 0);
    }

    //--Sleep for 1 seconds.
    usleep(1000000);
}


void Y1ModularRobot::reset_comm_link(void)
{
    unsigned int comm_trial_counter=0;
     usleep(1000000); //--Sleep for 1 second.

     do
     {
         if(get_current_time() == 1)
         {
             comm_trial_counter++;
         }
         else
         {
             comm_trial_counter = 0;
         }
     }while(comm_trial_counter < 3);

     //get_current_time();

     usleep(1000000); //--Sleep for 1 second.
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

/***********************************************Temp Solution *******************************************************/
  // TODO: This format is a pain in the ass, and needs to be fixed to the line above, both here and in Ardino code.
  outSS << "#%0%&" << module << "&*<";
  for(unsigned int n=0; n<module; n++)
  {
        outSS << "><";
    }
    outSS << servo_angle << ">*$";
/***********************************************Temp Solution *******************************************************/

  for(unsigned int i=0;i<outSS.str().size();i++)
  {
    outBuf[i] = outSS.str()[i];
  }

  if(serial_port)
  {
    if(!SendBuf(serial_port, outBuf, outSS.str().size()))
    {
      std::cerr << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                << "void set_moduleServo_position(int, double) method." << std::endl
                << "SendBuf Failed!" << std::endl;
         exit(1);
    }
  }
  else
  {
    std::cerr << "MorphoMotion Error: Y1ModularRobot class." << std::endl
              << "void set_moduleServo_position(int, double) method." << std::endl
              << "No Serial Port: " << serial_port << "." <<std::endl;

    exit(1);
  }
}


void Y1ModularRobot::set_all_moduleServo_position(const vector<double>& servo_angle)
{
  if(!(previous_control_signal == servo_angle))
  {
    previous_control_signal = servo_angle;

    stringstream outSS;
    unsigned char outBuf[100];

    outSS << "#%0%&" << 4 << "&*";
    for(unsigned int module=0; module<number_of_modules; module++)
    {
      outSS << "<" << servo_angle[module] << ">";
    }
    outSS << "*$";

    for(unsigned int i=0;i<outSS.str().size();i++)
    {
      outBuf[i] = outSS.str()[i];
    }

    if(serial_port)
    {
      if(!SendBuf(serial_port, outBuf, outSS.str().size()))
      {
        std::cerr << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                  << "void set_all_moduleServo_position(const vector<double>&) method." << std::endl
                  << "SendBuf Failed!" << std::endl;

        exit(1);
      }
      else
      {
        //std::cout << " Message Sent! " << std::endl; // TODO: Debugger to be removed.
      }
    }
    else
    {
      std::cerr << "MorphoMotion Error: Y1ModularRobot class." << std::endl
                << "void set_all_moduleServo_position(const vector<double>&) method." << std::endl
                << "No Serial Port: " << serial_port << "." <<std::endl;

      exit(1);
    }
  }
}


double Y1ModularRobot::get_moduleServo_position(unsigned int module)
{
  return 0;
}


bool Y1ModularRobot::get_all_moduleServo_position(vector<ServoFeedback*>& servo_feedback)
{
  get_all_moduleServo_position_with_individual_time_THREAD(servo_feedback);
  return true;
}


//-- To get servo position via serial communication with the controller board [Skymega], along with a single time value at which all servo position were read.
//-- Compatible with Arduino code --> servo_controller_charArray_V6.pde
bool Y1ModularRobot::get_all_moduleServo_position_with_time(vector<ServoFeedback*>& servo_feedback)
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
        char recvString[90];
        do
        {
          cprintf(serial_port, (char *)outBuf); // Send message requesting currnt servo position of all modules to Skymega.

          char inString[90]; // Bug?: char inString[20]; Is this a bug? The message frame seems to be a size of approx. 42 characters long.
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

      for(unsigned int module=0; module<number_of_modules; module++)
      {
        servo_feedback[module]->set_new_value(elapsed_evaluation_time, servo_feedback_angle[module]);
      }
    }
    else
    {
      std::cerr << std::endl
                << "Flood Error: SimEnv application." << std::endl
                << "void read_servo_position_real(void) method." << std::endl
                << "Serial port not open." << std::endl;
      exit(1);
    }

    return true;
}


//-- To get servo position via serial communication with the controller board [Skymega], along with individual time value at which each servo position was read.
//-- Compatible with Arduino code --> servo_controller_charArray_V7.pde
bool Y1ModularRobot::get_all_moduleServo_position_with_individual_time(vector<ServoFeedback*>& servo_feedback)
{
    unsigned long time;

    std::vector<double> servo_feedback_angle;
    servo_feedback_angle.resize(number_of_modules);

    std::vector<long> servo_read_time;
    servo_read_time.resize(number_of_modules);

    unsigned int comm_fail_consec_counter=0;

    if(serial_port)
    {
      unsigned char outBuf[20] = {'#','%','1','%','&','4','&','*','4','5','.','0','*','$'};  // To request current servo position of all modules [PC to Skymega].
      bool message_got = false;
      bool message_decoded = false;

      do
      {
        char recvString[90];
        do
        {
          cprintf(serial_port, (char *)outBuf); // Send message requesting currnt servo position of all modules to Skymega.

          char inString[90]; // Bug?: char inString[20]; Is this a bug? The message frame seems to be a size of approx. 63 characters long.
          message_got = get_message_with_time(inString);
          if(message_got)
          {
            strcpy(recvString,inString);
          }
          else
          {
            flush_cport(serial_port, 1);
            comm_fail_counter++;
            comm_fail_consec_counter++;
          }

          if(comm_fail_counter>=MAX_COMM_FAIL || comm_fail_consec_counter>=MAX_COMM_FAIL_CONSECUTIVE)
          {
              return false;
          }
        }while(!message_got);

        //std::cout << recvString << std::endl; //TODO: Debugger to be removed.

        message_decoded = decode_message_with_individual_time(recvString, time, servo_feedback_angle, servo_read_time);
      }while(!message_decoded);

      for(unsigned int module=0; module<number_of_modules; module++)
      {
        //servo_feedback[module]->set_new_value(elapsed_evaluation_time + servo_read_time[module], servo_feedback_angle[module]);
        servo_feedback[module]->set_new_value((time - get_initial_evaluation_time()) + servo_read_time[module], servo_feedback_angle[module]);
        //servo_feedback[module]->add_to_history(elapsed_evaluation_time + servo_read_time[module], servo_feedback_angle[module]);
      }
      set_elapsed_evaluation_time(time);
    }
    else
    {
      std::cerr << std::endl
                << "Flood Error: SimEnv application." << std::endl
                << "void read_servo_position_real(void) method." << std::endl
                << "Serial port not open." << std::endl;
      exit(1);
    }

    return true;
}


//-- To get servo position via serial communication with the controller board [Skymega], along with individual time value, implemented as a speperate thread.
//-- Compatible with Arduino code --> servo_controller_charArray_V10.pde
void Y1ModularRobot::get_all_moduleServo_position_with_individual_time_THREAD(vector<ServoFeedback*>& servo_feedback)
{
    std::vector<char> inString;

    unsigned long time = 0;

    std::vector<double> servo_feedback_angle;
    servo_feedback_angle.resize(number_of_modules);

    std::vector<long> servo_read_time;
    servo_read_time.resize(number_of_modules);

    bool message_got = false;
    bool message_decoded = false;
    char c;

    unsigned int comm_fail_consec_counter=0;

    if(serial_port)
    {
      if(receive_broadcast)
      {
        //--Turn On Broadcast
        //unsigned char Broad_On[8] = {'#','%','5','%','&','4','&','$'};  // To request Turn_On_Broadcast.
        //cprintf(serial_port, (char *)Broad_On); // Send message requesting Turn_On_Broadcast.
        turn_on_broadcast();
      }

      while(receive_broadcast)
      {
        set_broadcast_thread(true);

        //std::vector<char> inString; //--BUG
        message_got = get_message_with_time_THREAD(inString);
        if(message_got)
        {
          do
          {
            char recvString[100];
            unsigned int recvString_size=0;

            do
            {
              c = inString[0];
              inString.erase(inString.begin());
            }while(c != '#' && inString.size() > 0);
            recvString[recvString_size++] = c;

            if(contains(inString,'$'))
            {

                do
                {
                  c = inString[0];
                  inString.erase(inString.begin());
                  recvString[recvString_size++] = c;
                }while(c != '$' && inString.size() > 0);
                recvString[recvString_size] = '\0'; //--Always put a "null" at the end of a string!

                //message_decoded = decode_message_with_individual_time(recvString, time, servo_feedback_angle, servo_read_time);
                message_decoded = decode_message_with_individual_time_V2(recvString, time, servo_feedback_angle, servo_read_time);

                if(message_decoded)
                {
                  //std::cout << (double)(time - get_initial_evaluation_time())/1000000.0 << " "; //TODO: Debugger to be removed
                  //std::cout << (time - get_initial_evaluation_time()) << " "; //TODO: Debugger to be removed
                  for(unsigned int module=0; module<number_of_modules; module++)
                  {
                    servo_feedback[module]->set_new_value((time - get_initial_evaluation_time()) + servo_read_time[module], servo_feedback_angle[module]);
                    //servo_feedback[module]->set_new_value((time - get_initial_evaluation_time()), servo_feedback_angle[module]);
                  }
                  //std::cout << std::endl; //TODO: Debugger to be removed
                  set_elapsed_evaluation_time(time);
                }
            }
            else
            {
              inString.insert(inString.begin(),'#');
            }
          }while(contains(inString,'#') && contains(inString,'$') && receive_broadcast);
        }
        else
        {
          comm_fail_counter++;
          comm_fail_consec_counter++;
        }

        if(comm_fail_counter>=MAX_COMM_FAIL || comm_fail_consec_counter>=MAX_COMM_FAIL_CONSECUTIVE)
        {
          //return false;
            std::cout << std::endl << "Comm Failed: Comm fail counter reached MAX." << std::endl;
        }
      }

      if(!receive_broadcast)
      {
        turn_off_broadcast(10000);
      }
    }
    else
    {
      std::cerr << std::endl
                << "Flood Error: SimEnv application." << std::endl
                << "void get_all_moduleServo_position_with_individual_time_THREAD(vector<ServoFeedback*>&) method." << std::endl
                << "Serial port not open." << std::endl;
      exit(1);
    }

    set_broadcast_thread(false);
}


unsigned int Y1ModularRobot::get_current_time(void)
{
    unsigned long time = 0;

    std::vector<double> servo_feedback_angle;
    servo_feedback_angle.resize(number_of_modules);

    std::vector<long> servo_read_time;
    servo_read_time.resize(number_of_modules);

    unsigned int trial_counter=0;

    if(serial_port)
    {
      std::stringstream outSS;
      unsigned char outBuf[10]; //= {'#','%','3','%','&','4','&','$'};
      bool message_got = false;
      bool message_decoded = false;

      outSS << "#%3%&4&$"; //-- To request current time [PC to Skymega]. // TODO: 'To' address [&4&] should not be manditory. This should be changed in Arduino code as well.
      for(unsigned int i=0; i<outSS.str().size(); i++)
      {
        outBuf[i] = outSS.str()[i];
      }

      do
      {
        char recvString[40];
        do
        {
          SendBuf(serial_port, outBuf, outSS.str().size()); //-- Sending message requesting current time from Skymega.
          trial_counter++;

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

        message_decoded = decode_message_with_individual_time(recvString, time, servo_feedback_angle, servo_read_time);
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

    return trial_counter;
}


bool Y1ModularRobot::init_elapsed_evaluation_time(unsigned long new_initial_evaluation_time)
{
  if(new_initial_evaluation_time >= 4230000000)
  {
    std::cout << "Skymega time about to rollover. Sleeping for 10 seconds..." << std::endl;
    usleep(10000000);
    return false;
  }
  else
  {
    previous_read_evaluation_time = 0;
    elapsed_evaluation_time = 0;
    initial_evaluation_time = new_initial_evaluation_time;
  }
  return true;
}


void Y1ModularRobot::set_elapsed_evaluation_time(unsigned long new_elapsed_evaluation_time)
{
  //unsigned long counter = 0;
  long long time_diff;
  struct timeval start_time;
  struct timeval now;

  gettimeofday(&start_time, NULL);

  //--Wait until the current processing is completed before updating the time.
  /*while(get_processing_flag())
  {
    //counter++;

    //-- Get current time
    gettimeofday(&now, NULL);
    time_diff = timeval_diff(NULL, &now, &start_time);

    //if(time_diff >= 1000000) //--In milliseconds.
    if(time_diff >= 15000) //--In milliseconds.
    {
      set_receive_broadcast(false);
      set_processing_flag(false);

      std::cout << "WAIT TIME expired in Communication Thread: " << time_diff << std::endl;
      return;
    }
  }*/

  updating_elapsed_evaluation_time = true;
  set_processing_flag(true);

  previous_read_evaluation_time = elapsed_evaluation_time;
  elapsed_evaluation_time = new_elapsed_evaluation_time - initial_evaluation_time;

  updating_elapsed_evaluation_time = false;

  //std::cout << elapsed_evaluation_time << std::endl; // TODO: Debugger to be removed.
}


unsigned long Y1ModularRobot::get_elapsed_evaluation_time(void)
{
  //while(updating_elapsed_evaluation_time);
  return(elapsed_evaluation_time);
}


unsigned long Y1ModularRobot::get_previous_read_evaluation_time(void)
{
  return(previous_read_evaluation_time);
}


double Y1ModularRobot::calculate_distance_travelled_euclidean(void)
{
  double distanceTravelled;
  std::vector<double> robot_pos_final;

  // Capture the current position of the robot
  robot_pos_final = get_robot_XY("Final");

  distanceTravelled = euclidean_distance(robot_pos_initial, robot_pos_final);

  return(distanceTravelled);
}


void Y1ModularRobot::measure_cumulative_distance(void)
{
  //-- TODO
  distance_travelled = distance_travelled + 1;
}


double Y1ModularRobot::get_robot_X(void)
{
  //-- To be implemnted
    return(robot_pos_initial[0]);
}


double Y1ModularRobot::get_robot_Y(void)
{
    //-- To be implemnted
    return(robot_pos_initial[1]);
}


void Y1ModularRobot::step(const std::string& type)
{
  set_processing_flag(false);

  //-- Wait until elapsed_evaluation_time has been updated.
  while(previous_elapsed_evaluation_time == get_elapsed_evaluation_time())
  {
    //std::cout << std::endl;
    //elap = get_elapsed_evaluation_time();
    //std::cout << previous_elapsed_evaluation_time << "--" << get_elapsed_evaluation_time() << ">>";
  }

  previous_elapsed_evaluation_time = get_elapsed_evaluation_time();
}


double Y1ModularRobot::euclidean_distance(const std::vector<double> pos_1, const std::vector<double> pos_2)
{
    return( sqrt(((pos_1[0] - pos_2[0])*(pos_1[0] - pos_2[0])) + ((pos_1[1] - pos_2[1])*(pos_1[1] - pos_2[1]))));
}


void Y1ModularRobot::turn_on_broadcast(void)
{
    if(serial_port)
    {
      std::stringstream outSS;
      unsigned char outBuf[10];
      bool message_got = false;
      bool message_decode = false;

      outSS << "#%5%&4&$"; //--To request Turn_On_Broadcast.
      for(unsigned int i=0; i<outSS.str().size(); i++)
      {
        outBuf[i] = outSS.str()[i];
      }

      do
      {
        char recvString[40];
        do
        {
          SendBuf(serial_port, outBuf, outSS.str().size()); //-- Sending message requesting Turn_On_Broadcast.

          std::vector<char> inString;
          unsigned int recvString_size=0;
          char c;

          message_got = get_message_with_time_THREAD(inString);
          if(message_got)
          {
            do
            {
              c = inString[0];
              inString.erase(inString.begin());
            }while(c != '#' && inString.size() > 0);
            recvString[recvString_size++] = c;

            if(contains(inString,'$'))
            {
              do
              {
                c = inString[0];
                inString.erase(inString.begin());
                recvString[recvString_size++] = c;
              }while(c != '$' && inString.size() > 0);
              recvString[recvString_size] = '\0'; //--Always put a "null" at the end of a string!
            }
          }
        }while(!message_got);

        if(decode_message_with_ackn(recvString) == Turn_On_Broadcast_Ackn)
        {
            message_decode = true;
        }
        else
        {
            message_decode = false;
        }
      }while(!message_decode);
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


void Y1ModularRobot::turn_off_broadcast(unsigned long usleep_time)
{
  unsigned char Broad_Off[8] = {'#','%','6','%','&','4','&','$'};  // To request Turn_Off_Broadcast.

  do
  {
    cprintf(serial_port, (char *)Broad_Off); //--Send message requesting Turn_Off_Broadcast.
    usleep(usleep_time);
  }while(clear_cport(serial_port));
}


void Y1ModularRobot::displace_robot(void)
{
  std::vector<double> random_servo_pos(number_of_modules);

  for(unsigned int it=0; it<DISPLACEMENT_LENGTH; it++)
  {
    double random_pos = calculate_random_uniform(0.0, 40.0);
    for(unsigned int module=0; module < number_of_modules; module++)
    {
      random_servo_pos[module] = random_pos + (10.0 * module);
    }
    this->set_all_moduleServo_position(random_servo_pos);
    usleep(1000000);
    reset_modules();
  }
}


void changemode(int dir)
{
  static struct termios oldt, newt;

  if ( dir == 1 )
  {
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  }
  else
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}


int kbhit (void)
{
  struct timeval tv;
  fd_set rdfs;

  tv.tv_sec = 0;
  tv.tv_usec = 0;

  FD_ZERO(&rdfs);
  FD_SET (STDIN_FILENO, &rdfs);

  select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &rdfs);
}


bool operator==(const std::vector<double>& lhs, const std::vector<double>& rhs)
{
  if(lhs.size() != rhs.size())
  {
    std::cerr << "MorphoMotion Error: Y1ModularRobot class." << std::endl
              << "operator==(const std::vector<double>&, const std::vector<double>&) method." << std::endl
              << "Size of vectors lhs: " << lhs.size() << " is NOT equal to size of vector rhs: " << rhs.size() << std::endl;

    exit(1);
  }
  else
  {
    for(unsigned int i=0; i<lhs.size(); i++)
    {
      if(int(lhs[i]) != int(rhs[i]))
      {
        return false;
      }
    }
  }

  return true;
}
