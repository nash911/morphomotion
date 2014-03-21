/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   S E R I A L   C O M M U N I C A T I O N   E V A L U A T O R                                                                */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <thread>

#include "SerialCommunication.h"

#define BAUD_RATE 115200
//#define BAUD_RATE 57600

//#define MAX_POLL_TRIALS 10  //-- For Wired-Communication between PC and Skymega
#define MAX_POLL_TRIALS 20  //-- For XBEE-Communication between PC and Skymega

unsigned int set_serial_port(const std::string& new_serial_port, unsigned int baud_rate)
{
  unsigned int serial_port=0;

  if(new_serial_port == "/dev/ttyUSB0")
  {
    serial_port = 16;
    if(OpenComport(serial_port, baud_rate))
    {
      serial_port = 0;
      exit(1);
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
  }
  else if(new_serial_port == "/dev/ttyUSB2")
  {
    serial_port = 18;
    if(OpenComport(serial_port, baud_rate))
    {
      serial_port = 0;
      exit(1);
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
  }
  else
  {
    std::cout << "Unknown Serial Port: " << new_serial_port << "." <<std::endl;

    exit(1);
  }

  return serial_port;
}


double calculate_random_uniform(double minimum, double maximum)
{
   double random = (double)rand()/(RAND_MAX+1.0);

   double random_uniform = minimum + (maximum-minimum)*random;

   return(random_uniform);
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


bool contains(std::vector<char> V, char c)
{
  for(unsigned int i=0; i<V.size(); i++)
  {
    if(V[i] == c)
    {
      return true;
    }
  }

  return false;
}


//--Compatible with Arduino code V8
void send_reveive_loop(unsigned int serial_port)
{
  unsigned int poll_counter = 0;
  int n;
  //int comm_id = 0;
  unsigned int i=0;
  bool foundEndBit = false;
  unsigned int poll_trail_fail_counter = 0;

  while(i < 1000000)
  {
    //flush_cport(serial_port, 1);
    unsigned char outBuf[20] = {'#','%','1','%','&','4','&','*','4','5','.','0','*','$'};  // To request current servo position of all modules [PC to Skymega].
    cprintf(serial_port, (char *)outBuf); // Send message requesting currnt servo position of all modules to Skymega.

    poll_counter=0;
    unsigned char message[50];
    unsigned int message_size=0;

    do
    {
      foundEndBit = false;
      unsigned char inBuf[70] = {'x','x','x','x','x','x','x','x','x','x',
                                 'x','x','x','x','x','x','x','x','x','x',
                                 'x','x','x','x','x','x','x','x','x','x',
                                 'x','x','x','x','x','x','x','x','x','x',
                                 'x','x','x','x','x','x','x','x','x','x',
                                 'x','x','x','x','x','x','x','x','x','x',
                                 'x','x','x','x','x','x','x','x','x','x'};

      n = PollComport(serial_port, inBuf, 4095);
      inBuf[n] = '\0'; // always put a "null" at the end of a string!

      for(unsigned int j=0; j<n; j++)
      {
        message[message_size++] = inBuf[j];
      }

      //std::cout << "n = " << n << " --> " << inBuf << std::endl;

      if(inBuf[n-1] == '$')
      {
        foundEndBit = true;
      }
      poll_counter++;
    }while(foundEndBit != true && poll_counter <= MAX_POLL_TRIALS);

//---------------------------------------------------PERFORMNCE TEST---------------------------------------------------//
    if(poll_counter > MAX_POLL_TRIALS)
    {
        poll_trail_fail_counter++;
    }
    else
    {
        poll_trail_fail_counter = 0;
    }

    if(poll_trail_fail_counter >= 3)
    {
        std::cout << std::endl << std::endl << "EXCEEDED POLL COUNTER! --> " << poll_counter << std::endl;
        exit(0);
    }
//---------------------------------------------------PERFORMNCE TEST---------------------------------------------------//

    message[message_size] = '\0'; //--Always put a "null" at the end of a string!

    //std::cout << "Message " << i << " --> Poll Counter = " << poll_counter << " --> " << message << std::endl;

    std::cout << "Message " << i << " --> Poll Counter = " << poll_counter << " --> ";

    for(unsigned int a=0; a<message_size; a++)
    {
        if(message[a] == '[')
        {
            a++;
            while(message[a] != ']')
            {
                std::cout << message[a];
                a++;
            }
            break;
        }
    }
    std::cout << std::endl;

    i++;
  }
}


//--Compatible with Arduino code V10
void broadcast_loop(unsigned int serial_port, unsigned long& uSecond)
{
  unsigned int poll_counter = 0;
  int n;
  unsigned int i=0;
  bool foundEndBit = false;
  unsigned int poll_trail_fail_counter = 0;
  unsigned char num = 0;
  unsigned int error_counter=0;
  //unsigned long uSecond = 0;

  std::vector<char> inV;

  std::stringstream outSS_1;
  unsigned char outBuf_1[10];



  outSS_1 << "#%5%&4&$";  // Requesting Turn_On_Broadcast.

  for(unsigned int i=0; i<outSS_1.str().size(); i++)
  {
    outBuf_1[i] = outSS_1.str()[i];
  }
  SendBuf(serial_port, outBuf_1, outSS_1.str().size()); // Send message requesting Turn_On_Broadcast.

  //while(i < 1000)
  while(uSecond <= 20000000)
  {
    poll_counter=0;
    unsigned char c = '$';

    //unsigned char outBuf_1[20] = {'#','%','1','%','&','4','&','*','4','5','.','0','*','$'};  // To request current servo position of all modules [PC to Skymega].
    //cprintf(serial_port, (char *)outBuf_1); // Send message requesting currnt servo position of all modules to Skymega.

    do
    {
      foundEndBit = false;
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
      //inBuf[n] = '\0'; // always put a "null" at the end of a string!

      for(unsigned int j=0; j<n; j++)
      {
        /*if(inBuf[j] == '$')
        {
          foundEndBit = true;
        }*/

        inV.push_back(inBuf[j]);
      }

      foundEndBit = true;
      poll_counter++;
    //}while(foundEndBit != true); // && poll_counter <= MAX_POLL_TRIALS);
    }while(!contains(inV,'#') || !contains(inV,'$'));

//---------------------------------------------------PERFORMNCE TEST---------------------------------------------------//
    if(poll_counter > MAX_POLL_TRIALS)
    {
        poll_trail_fail_counter++;
    }
    else
    {
        poll_trail_fail_counter = 0;
    }

    if(poll_trail_fail_counter >= 3)
    {
        std::cout << std::endl << std::endl << "EXCEEDED POLL COUNTER! --> " << poll_counter << std::endl;
        exit(0);
    }
//---------------------------------------------------PERFORMNCE TEST---------------------------------------------------//

    if(foundEndBit)
    {
      //do
      while(contains(inV,'#') && contains(inV,'$'))
      {
        unsigned char message[120];
        unsigned int message_size=0;

        //if(i==1000)
        {
          //std::cout << std::endl << "Size = " << inV.size() << " Message: " << toString(inV) << std::endl;
        }

        /*do
        {
          c = inV[0];
          inV.erase(inV.begin());
        }while(c != '#' && inV.size()>0);
        message[message_size++] = c;*/

        do
        {
          c = inV[0];
          inV.erase(inV.begin());
          message[message_size++] = c;
        }while(c != '$');

        message[message_size] = '\0'; //--Always put a "null" at the end of a string!

        std::cout << "Message " << i << " --> Poll Counter = " << poll_counter << " --> " << message << " "; //std::endl;

        //std::cout << "Message " << i << " --> Poll Counter = " << poll_counter << " --> ";
        char number[12] = {'x','x','x','x','x','x','x','x','x','x','x','x'};

        for(unsigned int a=0, b=0; a<message_size; a++)
        {
            if(message[a] == '[')
            {
                a++;
                while(message[a] != ']')
                {
                  //std::cout << message[a];
                  number[b] = message[a];
                  a++;
                  b++;
                }
                break;
            }
        }
        //std::cout << std::endl;

        uSecond = atol(number);
        std::cout << uSecond << std::endl;

        /*if(atoi(number) >= 15) //|| atoi(number) >= 25 || atoi(number) == 35)
        {
          std::cout << std::endl << "ERROR!" << std::endl;
          //std::cout << "Message " << i  << ": Error = " << error_counter << " Message: " << message << std::endl;
          //exit(0);
          error_counter++;
        }*/

        i++;

        /*if(i%10 == 0)
        {
          std::stringstream outSS_2;
          unsigned char outBuf_2[50];

          outSS_2 << "#%0%&4&*<-5><-5><-5><-5>*$";
          //outSS_2 << "#%6%&" << num++ << "&$";

          for(unsigned int i=0; i<outSS_2.str().size(); i++)
          {
            outBuf_2[i] = outSS_2.str()[i];
          }
          SendBuf(serial_port, outBuf_2, outSS_2.str().size()); // Send message requesting Turn_Off_Broadcast.

          //std::cout << std::endl << "Turn_Off_Broadcast" << std::endl;
        }*/
      }
    }
  }

  std::stringstream outSS_3;
  unsigned char outBuf_3[20];

  outSS_3 << "#%6%&4&$";  // Requesting Turn_Off_Broadcast.

  for(unsigned int i=0; i<outSS_3.str().size(); i++)
  {
    outBuf_3[i] = outSS_3.str()[i];
  }
  SendBuf(serial_port, outBuf_3, outSS_3.str().size()); // Send message requesting Turn_On_Broadcast.

  std::cout << std::endl << "Finished broadcast_loop(unsigned int) at Time: " << uSecond << std::endl;
}


void send_message(unsigned int serial_port, unsigned long& uSecond)
{
  std::stringstream outSS_2;
  unsigned char outBuf_2[100];
  unsigned int i=0;
  unsigned int j=0;
  unsigned long time = 0;

  outSS_2 << "#%0%&4&*<-25.0><-25.0><-25.0><-25.0>*$";
  for(unsigned int i=0; i<outSS_2.str().size(); i++)
  {
    outBuf_2[i] = outSS_2.str()[i];
  }

  //while(i<1000)
  while(uSecond <= 20000000)
  {
    if(uSecond - time >= 50000) //|| uSecond >= 10000000)
    {
      //usleep(5000);
      time = uSecond;

      std::stringstream outSS_3;
      unsigned char outBuf_3[100];
      outSS_3 << "#%0%&4&*<" << -25.0 + calculate_random_uniform(-1.0, 1.0);
      outSS_3 << "><" << -25.0 + calculate_random_uniform(-1.0, 1.0);
      outSS_3 << "><" << -25.0 + calculate_random_uniform(-1.0, 1.0);
      outSS_3 << "><" << -25.0 + calculate_random_uniform(-1.0, 1.0) << ">*$";

      for(unsigned int i=0; i<outSS_3.str().size(); i++)
      {
        outBuf_3[i] = outSS_3.str()[i];
      }

      SendBuf(serial_port, outBuf_3, outSS_3.str().size()); // Send message.

      //std::cout << "Sent " << i << " --> Time: " << time << std::endl;
      i++;
    }
    else
    {
      //std::cout << "Time --> " << uSecond << std::endl;
      std::cout << "";
    }
  }

  std::stringstream outSS_3;
  unsigned char outBuf_3[100];
  outSS_3 << "#%0%&4&*<0><0><0><0>*$";
  for(unsigned int i=0; i<outSS_3.str().size(); i++)
  {
    outBuf_3[i] = outSS_3.str()[i];
  }
  SendBuf(serial_port, outBuf_3, outSS_3.str().size()); // Send message.

  //std::cout << std::endl << "Finished send_message(unsigned int) at Time: " << uSecond << std::endl;
}


int main(int argc, char* argv[])
{
  unsigned int serial_port=0;
  unsigned long uSecond = 123;

  serial_port = set_serial_port(argv[1], BAUD_RATE);
  if(serial_port == 0)
  {
    printf("Can not open comport\n");
    return(0);
  }

  usleep(2000000);
  printf("Serial port open and initialised!\n");

  //send_reveive_loop(serial_port);
  //broadcast_loop(serial_port);

  std::thread Read(broadcast_loop, serial_port, std::ref(uSecond));
  std::thread Write(send_message, serial_port, std::ref(uSecond));

  Read.join();
  Write.join();

  std::cout << std::endl << "Ended broadcast_loop(unsigned int) and send_message(unsigned int) at Time: " << uSecond << std::endl;

  return 1;
}
