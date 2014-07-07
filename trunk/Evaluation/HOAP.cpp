/****************************************************************************************************************/
/*                                                                                                              */
/*   Morphomotion: A project researching on how morphology influences locomotion in robots.                     */
/*                                                                                                              */
/*   H O A P                                                                                                    */
/*                                                                                                              */
/*   Avinash Ranganath                                                                                          */
/*   Robotics Lab, Department of Systems Engineering and Automation                                             */
/*   University Carlos III de Mardid(UC3M)                                                                      */
/*   Madrid, Spain                                                                                              */
/*   E-mail: avinash.ranganath@uc3m.es                                                                          */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                                             */
/*                                                                                                              */
/****************************************************************************************************************/

// System includes
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <termios.h>

#include "Robot.h"
#include "SimulationOpenRave.h"

#define STEP_SIZE 0.001

#define SPACE 32

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

void read_from_file(vector<vector<double> > &data, SimulationOpenRave &robot)
{
    fstream file;
    string filename = "../Evaluation_Files/tri_sqr.dat";

    file.open(filename.c_str(), std::ios::in);
    if(!file.is_open())
    {
      std::cerr << "HOAP Error." << std::endl
                << "read_from_file(vector<double>&) method" << std::endl
                << "Cannot open data file: "<< filename  << std::endl;

      exit(1);
    }

    unsigned int DOF = robot.get_number_of_modules();
    double theta;
    unsigned int row = 0;

    do
    {
        data.resize(data.size()+1);
        file >> theta;
        for(unsigned int joint=0; joint<DOF; joint++)
        {
            file >> theta;
            data[row].push_back(theta);
        }
        row++;

    }while(!file.eof());

    std::cout << std::endl << "Reached EOF." << std::endl;
    file.close();
}


void run(SimulationOpenRave &robot)
{
  unsigned int DOF = robot.get_number_of_modules();
  vector<vector<double> > data;

  read_from_file(data, robot);

  unsigned int row = 0;

  char keyboard_key;
  std::cout << "Press a key to start the simulation" << std::endl;
  std::cin.get(keyboard_key);

  /*for(row=0; row<data.size(); row++)
  {
    vector<double> theta;

    for(unsigned int joint=0; joint<DOF; joint++)
    {
      theta.push_back(data[row][joint]);
    }

    robot.set_all_moduleServo_position(theta);
    robot.step("evaluation");
    //usleep(STEP_SIZE*100000);

    row++;
  }*/


  unsigned int key = 'H';
  changemode(1);

  do
  {
    do
    {
      vector<double> theta;

      for(unsigned int joint=0; joint<DOF; joint++)
      {
        theta.push_back(data[row][joint]);
      }

      robot.set_all_moduleServo_position(theta);
      robot.step("evaluation");

      row++;
    }while(!kbhit());

    if(kbhit())
    {
      key = getchar();
    }

    if(key==SPACE)
    {
      std::cout << std::endl << "             Paused: Waiting for SPACE key...." << std::endl;
      do
      {
        while(!kbhit());
        key = getchar();
      }while(key!=SPACE);

      std::cout << std::endl << "                       Continuing...." << std::endl;
    }
  }while(row<data.size());
  changemode(0);
}


int main(int argc, char* argv[])
{

  SimulationOpenRave robot;

  robot.set_scene_file_name("../models/HOAP_Miguel/simple.env.xml");
  robot.set_robot_priority("Robot_Primary");
  robot.set_simu_resolution_microseconds(STEP_SIZE);
  robot.init_simu_env("Dummy_Controller");

  run(robot);

  return 0;
}
