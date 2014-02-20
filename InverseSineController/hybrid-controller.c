/************************************Hybrid Controller V6************************************/

#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <values.h>
#include <signal.h>
#include <sys/time.h>
#include "locoapi.h"

#define ACTIVITY_LOG
#define RECORD_DATA
#define RECORD_BETA_MAP
#define RECORD_ACTUAL_DIFF_MAP

#define NO_OF_ACTUATORS 2
#define POSITION_BUFFER_SIZE 100 //100
#define VELOCITY_BUFFER_SIZE 30 //20
#define EPSILON 125000 //125000 //-- Milliseconds
#define VELOCITY_DIRECTION_DETECTOR_WINDOW 15
#define POSITION_FILTER_THRESHOLD 2500 //-- Degrees per second

//MotorID
//int actuatorID[NO_OF_ACTUATORS] = {4,59};
int actuatorID[NO_OF_ACTUATORS] = {31,34};

//Calculated robot position based on tilt sensor value
double controlPosition[NO_OF_ACTUATORS];

float P = 7.0; //7.0
float I = 0.0; //0.0
float D = 0.5; //0.5

double Pi = 3.14159265;

double A = 50.0; //60.0
double f = 0.50; //0.65
double o = 20.0; //0.0

double p[NO_OF_ACTUATORS] = {0.0, 0.0}; //--Phase
double pRad[NO_OF_ACTUATORS]; //--Phase in radians

double alpha = 2.0; //2.0;
double beta = 20.0; //10.0;
unsigned long minimumActuationTimeMs = 10000; //400000
double Y[NO_OF_ACTUATORS];

double currentPosition[NO_OF_ACTUATORS];
double currentVelocity[NO_OF_ACTUATORS];

double Theta[NO_OF_ACTUATORS]; //--Theta: Position feedback from the motor
double V[NO_OF_ACTUATORS]; //--Velocity of the motor

double timeMain; //--Time in seconds
double t[NO_OF_ACTUATORS]; //--Time value used in SineWave()
double tPrime[NO_OF_ACTUATORS]; //--Time shift value
double yInverse[NO_OF_ACTUATORS];
double tHat[NO_OF_ACTUATORS];

double timeMain;
double actual_diff[NO_OF_ACTUATORS];
double Position_now;
unsigned int passedZero[NO_OF_ACTUATORS];

double positionBuffer[NO_OF_ACTUATORS][POSITION_BUFFER_SIZE];
struct timeval positionReadTime[NO_OF_ACTUATORS][POSITION_BUFFER_SIZE];
double velocityBuffer[NO_OF_ACTUATORS][VELOCITY_BUFFER_SIZE];

struct timeval startTimeMain;
struct timeval iterationStartTime[NO_OF_ACTUATORS];
struct timeval startTimeShift[NO_OF_ACTUATORS];
struct timeval now;
struct timeval PositionReadTime_now;


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


void copy_timeval_structure(struct timeval *source, struct timeval *destination)
{
  if(source == NULL)
  {
    printf("\n Error: \n void copy_timeval_structure(struct timeval*, struct timeval*) \n The Source structure variable is NULL. \n");
    close();
    exit(0);
  }
  else if(destination == NULL)
  {
    printf("\n Error: \n void copy_timeval_structure(struct timeval*, struct timeval*) \n The Destination structure variable is NULL. \n");
    close();
    exit(0);
  }
  else
  {
    //printf("\n The source and destination structure variable are not NULL. \n");
    destination->tv_sec  = source->tv_sec;
    destination->tv_usec = source->tv_usec;
  }

  return;
}


/*void getMyCurrentPositionAndVelocity(unsigned int actuator)
{
  unsigned int i;
  long long timeDiff;
  double epsilon;

  for(i=0; i<POSITION_BUFFER_SIZE-1; i++)
  {
    positionBuffer[actuator][i] = positionBuffer[actuator][i+1];
    positionReadTime[actuator][i] = positionReadTime[actuator][i+1];
  }

  gettimeofday(&positionReadTime[actuator][POSITION_BUFFER_SIZE-1],NULL);
  positionBuffer[actuator][POSITION_BUFFER_SIZE-1] = getActuatorPosition(actuatorID[actuator]);

  currentPosition[actuator] = positionBuffer[actuator][POSITION_BUFFER_SIZE-1];

  for(i=POSITION_BUFFER_SIZE-2; i>=0; i--)
  {
    timeDiff = timeval_diff(NULL, &positionReadTime[actuator][POSITION_BUFFER_SIZE-1], &positionReadTime[actuator][i]);
    if(timeDiff >= EPSILON)
    {
      epsilon = timeDiff / 1000000.0; //--Converting to milliseconds.
      currentVelocity[actuator] = (positionBuffer[actuator][POSITION_BUFFER_SIZE-1] - positionBuffer[actuator][i]) / epsilon;
      //currentVelocity[actuator] = (positionBuffer[actuator][i] - positionBuffer[actuator][POSITION_BUFFER_SIZE-1]) / epsilon;
      break;
    }
  }
}*/


/*void getMyCurrentPositionAndVelocity(unsigned int actuator)
{
  double filter_V = 0;

  unsigned int i=0;
  long long timeDiff;
  double epsilon;

  do
  {
    gettimeofday(&PositionReadTime_now,NULL);
    Position_now = getActuatorPosition(actuatorID[actuator]);

    timeDiff = timeval_diff(NULL, &PositionReadTime_now, &positionReadTime[actuator][POSITION_BUFFER_SIZE-1]);
    epsilon = timeDiff / 1000000.0; //--Converting to milliseconds.
    filter_V = fabs(Position_now-positionBuffer[actuator][POSITION_BUFFER_SIZE-1])/epsilon;
  }while(filter_V >= POSITION_FILTER_THRESHOLD);

  for(i=0; i<POSITION_BUFFER_SIZE-1; i++)
  {
    positionBuffer[actuator][i] = positionBuffer[actuator][i+1];
    positionReadTime[actuator][i] = positionReadTime[actuator][i+1];
  }

  //gettimeofday(&positionReadTime[actuator][POSITION_BUFFER_SIZE-1],NULL);
  //positionBuffer[actuator][POSITION_BUFFER_SIZE-1] = getActuatorPosition(actuatorID[actuator]);

  copy_timeval_structure(&PositionReadTime_now, &positionReadTime[actuator][POSITION_BUFFER_SIZE-1]);
  positionBuffer[actuator][POSITION_BUFFER_SIZE-1] = Position_now;

  currentPosition[actuator] = positionBuffer[actuator][POSITION_BUFFER_SIZE-1];

  for(i=POSITION_BUFFER_SIZE-2; i>=0; i--)
  {
    timeDiff = timeval_diff(NULL, &positionReadTime[actuator][POSITION_BUFFER_SIZE-1], &positionReadTime[actuator][i]);
    if(timeDiff >= EPSILON)
    {
      epsilon = timeDiff / 1000000.0; //--Converting to milliseconds.
      currentVelocity[actuator] = (positionBuffer[actuator][POSITION_BUFFER_SIZE-1] - positionBuffer[actuator][i]) / epsilon;
      //currentVelocity[actuator] = (positionBuffer[actuator][i] - positionBuffer[actuator][POSITION_BUFFER_SIZE-1]) / epsilon;
      break;
    }
  }
}*/


void getMyCurrentPositionAndVelocity(unsigned int actuator)
{
  double filter_V = 0;

  unsigned int i=0;
  long long timeDiff;
  double epsilon;
  double totalVelocity = 0;

  do
  {
    gettimeofday(&PositionReadTime_now,NULL);
    Position_now = getActuatorPosition(actuatorID[actuator]);

    timeDiff = timeval_diff(NULL, &PositionReadTime_now, &positionReadTime[actuator][POSITION_BUFFER_SIZE-1]);
    epsilon = timeDiff / 1000000.0; //--Converting to milliseconds.
    filter_V = fabs(Position_now-positionBuffer[actuator][POSITION_BUFFER_SIZE-1])/epsilon;
  }while(filter_V >= POSITION_FILTER_THRESHOLD);

  /*positionBuffer[actuator][POSITION_BUFFER_SIZE-2] = positionBuffer[actuator][POSITION_BUFFER_SIZE-1];
  positionReadTime[actuator][POSITION_BUFFER_SIZE-2] = positionReadTime[actuator][POSITION_BUFFER_SIZE-1];*/

  for(i=(POSITION_BUFFER_SIZE-VELOCITY_BUFFER_SIZE)-1; i<POSITION_BUFFER_SIZE-1; i++)
  {
    positionBuffer[actuator][i] = positionBuffer[actuator][i+1];
    positionReadTime[actuator][i] = positionReadTime[actuator][i+1];
  }

  copy_timeval_structure(&PositionReadTime_now, &positionReadTime[actuator][POSITION_BUFFER_SIZE-1]);
  positionBuffer[actuator][POSITION_BUFFER_SIZE-1] = Position_now;

  currentPosition[actuator] = positionBuffer[actuator][POSITION_BUFFER_SIZE-1];

  for(i=0; i<VELOCITY_BUFFER_SIZE-1; i++)
  {
    velocityBuffer[actuator][i] = velocityBuffer[actuator][i+1];
  }

  velocityBuffer[actuator][VELOCITY_BUFFER_SIZE-1] = (Position_now-positionBuffer[actuator][POSITION_BUFFER_SIZE-2])/epsilon;

  for(i=0; i<VELOCITY_BUFFER_SIZE; i++)
  {
    totalVelocity = totalVelocity + velocityBuffer[actuator][i];
  }
  currentVelocity[actuator] = totalVelocity/VELOCITY_BUFFER_SIZE;

  if(Y[actuator] > 0.0 && currentPosition[actuator] > 0.0)
  {
    passedZero[actuator] = 1;
  }
  else if(Y[actuator] < 0.0 && currentPosition[actuator] < 0.0)
  {
    passedZero[actuator] = 1;
  }
}


/*double getCurrentVelocityDirection(unsigned int actuator)
{
  unsigned int i;
  int velocityDirection=0;

  for(i=POSITION_BUFFER_SIZE-2; i >= POSITION_BUFFER_SIZE - VELOCITY_DIRECTION_DETECTOR_WINDOW; i--)
  {
    if(positionBuffer[actuator][POSITION_BUFFER_SIZE-1] - positionBuffer[actuator][i] > 0.0)
    {
      velocityDirection++;
    }
    else if(positionBuffer[actuator][POSITION_BUFFER_SIZE-1] - positionBuffer[actuator][i] < 0.0)
    {
      velocityDirection--;
    }
  }

  if(velocityDirection > 0)
  {
    return(1.0);
  }
  else if(velocityDirection < 0)
  {
    return(-1.0);
  }
  else if(velocityDirection == 0)
  {
    if(positionBuffer[actuator][POSITION_BUFFER_SIZE-1] > 0)
    {
      return(1.0);
    }
    else if(positionBuffer[actuator][POSITION_BUFFER_SIZE-1] <= 0)
    {
      return(-1.0);
    }
  }
}*/


double getCurrentVelocityDirection(unsigned int actuator)
{
  if(positionBuffer[actuator][POSITION_BUFFER_SIZE-1] > 0)
  {
    return(1.0);
  }
  else if(positionBuffer[actuator][POSITION_BUFFER_SIZE-1] <= 0)
  {
    return(-1.0);
  }
}


double inverse_sinewave_function(double theta, double V)
{
  double invSineWave;

  //--               |‾                                                              ‾|
  //--               |1-(V/|V|)   1-(θ/|θ|) * 1+(V/|V|)   |‾         Sin^-1(θ-o/A) ‾| |     1
  //--Eg: y^-1(V,θ) =|--------- + --------------------- + |(V/|V|) x -------------  | |  x ---
  //--               |    4                 4             |_               2π      _| |     f
  //--               |_                                                              _|

  //invSineWave = ((1-(V/fabs(V)))/4 + ((1-(theta/fabs(theta))) * (1+(V/fabs(V))))/4 + ((V/fabs(V)) * (asin((theta-o[actuator])/A[actuator])/(2*Pi)))) * (1/f[actuator]);

  /*--------------------------------------For Debugging--------------------------------------*/
  double X;
  double Y;
  double Z;

  X = (1-(V/fabs(V)))/4;
  Y = ((1-(theta/fabs(theta))) * (1+(V/fabs(V))))/4;
  Z = ((V/fabs(V)) * (asin((theta-o)/A)/(2*Pi)));

  invSineWave = (X + Y + Z) * (1/f);

  #ifdef RECORD_DATA
    //fprintf(dataFile," %lf %lf %lf", X, Y, Z);
  #endif
  /*--------------------------------------For Debugging--------------------------------------*/

  return invSineWave;
}


void timeShift(double theta, double V, unsigned int act)
{
  //--y^-1(V,θ)
  yInverse[act] = inverse_sinewave_function(theta, V);
  
  //--Eq: t'(V,θ) = (1/2f) + [1/2f * (1-(V/|V|))] - y^-1(V,θ)

  //--               1    |‾  1    |‾ 1 - V   ‾| ‾|
  //--Eq: t'(V,θ) = --- + |  --- x |     ---  |  | - y^-1(V,θ)
  //--              2f    |  2f    |_    |V| _|  |
  //--                    |_                    _|

  tPrime[act] = (1.0/(2.0*f)) + ((1.0/(2.0*f)) * (1 - (V/fabs(V)))) - yInverse[act];
  gettimeofday(&startTimeShift[act],NULL);
  //usleep(50000); //--To escapes the deadlock situation.
}


double actualDiff(double Yi, double theta)
{
  //--Eq: [Yi(θ/|θ|)-θ(Yi/|Yi|)] * [|(Yi/|Yi|)+(θ/|θ|)|] * 1/2
  return(((Yi * (theta/fabs(theta))) - (theta * (Yi/fabs(Yi)))) * fabs((Yi/fabs(Yi)) + (theta/fabs(theta))) / 2.0);
}


//Main loop 
int main(void)
{

  unsigned int actuator = 0;
  unsigned int i = 0;
  unsigned long n = 0;
  long long timeDiff = 0;

  //struct timeval first;

  
  for(actuator=0; actuator<NO_OF_ACTUATORS; actuator++)
  {
    //Sets the motor to its rellative position
    setRegisterValueF("CURRENT_POSITION",actuatorID[actuator],0);

    //Sets the PID values of the motor
    setRegisterValueF("I",actuatorID[actuator],I);
    setRegisterValueF("P",actuatorID[actuator],P);
    setRegisterValueF("D",actuatorID[actuator],D);

    //Sets the controller mode
    setRegisterValueUI("CONTROLLER_MODE",actuatorID[actuator],4);

    //--Reset the control position buffer to 0°
    setActuatorPosition(0.0, actuatorID[actuator]);

    //--Initialize Theta to a very small value, as the first output of the Sine() could be 0°
    Theta[actuator] = 0.000001;

    //--Initialze velocity of the motor to a positive value (1.0). This is based on the assumption that with the sine controller, the velocity starts with a positive value.
    V[actuator] = 1.0;

    tPrime[actuator] = 0;

    for(i=0; i<POSITION_BUFFER_SIZE; i++)
    {
      positionBuffer[actuator][i] = getActuatorPosition(actuatorID[actuator]);
      gettimeofday(&positionReadTime[actuator][i],NULL);
    }

    for(i=0; i<VELOCITY_BUFFER_SIZE; i++)
    {
      velocityBuffer[actuator][i] = 0;
    }
  }

  for(actuator=0; actuator<NO_OF_ACTUATORS; actuator++)
  {
    pRad[actuator] = (p[actuator] * Pi)/180.0;
    passedZero[actuator] = 1;
  }

  //Prints messege
  printf("Running\n\r");

  for(actuator=0; actuator<NO_OF_ACTUATORS; actuator++)
  {
    Y[actuator] = A + o;
  }

sleep(3);

  //-- Get current time
  gettimeofday(&now,NULL);

  //--Copy current time as the beginning time for each actuator
  for(actuator=0; actuator<NO_OF_ACTUATORS; actuator++)
  {
    copy_timeval_structure(&now, &iterationStartTime[actuator]);
    copy_timeval_structure(&now, &startTimeShift[actuator]);
  }
 
  while(timeMain <= 300.0)
  {
    //-- Get current time
    gettimeofday(&now,NULL);


    for(actuator=0; actuator<NO_OF_ACTUATORS; actuator++)
    {
      timeDiff = timeval_diff(NULL,&now,&iterationStartTime[actuator]);

      /*currentPosition[actuator] = getActuatorPosition(actuatorID[actuator]);
      currentVelocity[actuator] = getActuatorVelocity(actuatorID[actuator]);*/

      getMyCurrentPositionAndVelocity(actuator);

      actual_diff[actuator] = actualDiff(Y[actuator], currentPosition[actuator]);

      //-- Check if the minimul initialization time has elapsed since the last control signal.
      //if(timeDiff >= minimumActuationTimeMs || actual_diff[actuator] < 0.0)
      if(passedZero || actual_diff[actuator] < 0.0)
      //if((timeDiff>=minimumActuationTimeMs && passedZero) || actual_diff[actuator] < 0.0)
      {
        //-- Check if the actuator has reached or over shot the desired position, or  if the velocity has dropped below the set threshold.
        //if(fabs(Y[actuator]-currentPosition[actuator]) <= alpha || fabs(currentVelocity[actuator]) <= beta)

        //actual_diff[actuator] = actualDiff(Y[actuator], currentPosition[actuator]);
        if(fabs(Y[actuator]-currentPosition[actuator]) <= alpha || actual_diff[actuator] < 0.0 || fabs(currentVelocity[actuator]) <= beta)
        {

          //--Make sure Theta is NOT EQUAL to 0. To avoid division by zero.
          if(currentPosition[actuator] != 0)
          {
            //--Make sure that Theta is between the range of Amplitude+Offset > Theta > -Amplitude+Offset.
            if(currentPosition[actuator] <= A+o && currentPosition[actuator] >= ((-1.0)*A)+o)
            {
              Theta[actuator] = currentPosition[actuator];
            }
            else if(currentPosition[actuator] >= A+o)
            {
              Theta[actuator] = A+o;
            }
            else if(currentPosition[actuator] <= ((-1.0)*A)+o)
            {
              Theta[actuator] = ((-1.0)*A)+o;
            }
          }
          else if(currentPosition[actuator] == 0)
          {
            //printf("\n Stopped because current Position of actuator %d = %lf \n", actuator+1, currentPosition[actuator]);
            /*close();
            exit(0);*/
          }

          //--Make sure Velocity is NOT EQUAL to 0. To avoid division by zero.
          V[actuator] = getCurrentVelocityDirection(actuator);

          //--Calculate the Time Shift factor.
          timeShift(Theta[actuator], V[actuator], actuator);

          //-- Record start time of the current iteration
          gettimeofday(&iterationStartTime[actuator],NULL);
          passedZero[actuator] = 0;

          /*//-- Calculate the new destination of the actuator
          tHat[actuator] = tPrime[actuator]*f;

          if(tHat[actuator] < 0.0)
          {
            printf("\n ERROR: tHat[%d] = %lf\n \n \n", actuator+1, tHat[actuator]);
            close();
            exit(0);
          }
          else if(tHat[actuator] >= 0.0 && tHat[actuator] < 0.25)
          {
            tHat[actuator] = 0.25/f;
          }
          else if(tHat[actuator] >= 0.25 && tHat[actuator] < 0.75)
          {
            tHat[actuator] = 0.75/f;
          }
          else if(tHat[actuator] >= 0.75 && tHat[actuator] <= 1.0)
          {
            tHat[actuator] = 0.25/f;
          }
          else if(tHat[actuator] > 1.0)
          {
            printf("\n ERROR: tHat[%d] = %lf\n \n \n", actuator+1, tHat[actuator]);
            close();
            exit(0);
          }*/

          //-- Calculate the new destination of the actuator
          if(currentPosition[actuator] >= 0.0)
          {
            tHat[actuator] = 0.75/f;
          }
          else if(currentPosition[actuator] < 0.0)
          {
            tHat[actuator] = 0.25/f;
          }

          //-- Updating new destination of the actuator based on tHat, which is calculated based on tPrime.
          Y[actuator] = A * sin(2*Pi*f*tHat[actuator]) + o;
        }
        else
        {

        }
      }
      else
      {

      }

      timeDiff = timeval_diff(NULL,&now,&startTimeShift[actuator]);  // BUG(?): When Time Shift takes place, startTimeShift[actuator] > now. This is incorrect because startTimeShift[actuator] should always be <= now.
                                                                     // Bug resolved in void timeShift(double, double, unsigned int)
      t[actuator] = (double)(timeDiff/1000000.0);
      t[actuator] = t[actuator] + tPrime[actuator];

      //--Sine Wave Function
      //controlPosition[actuator] = A * sin(2*Pi*f*t[actuator]) + o;
      controlPosition[actuator] = A * sin(2*Pi*f*t[actuator] + pRad[actuator]) + o;  //--With Phase

      //-- Send control signal to the actuator
      setActuatorPosition(controlPosition[actuator], actuatorID[actuator]);
  }

  close();

  return 0;
}  

