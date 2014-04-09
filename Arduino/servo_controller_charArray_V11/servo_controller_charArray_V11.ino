// Move the servo to a specified value 
// by Avinash Ranganath <nash911@gmx.com> 

/* Previous
 V11 -- Inherited from V10 -- 1. Eliminated the use of myservo_previous_input[].
                                2. Changed 'From' metadata to a fixed value in function unsigned int OutputStringStream_All(unsigned char*) */
                                
                                
// Move the servo to a specified value 
// by Avinash Ranganath <nash911@gmx.com> 

/* V11 -- Inherited from V10 -- 1. Added Send_Acknowledgement. */

#include <stdio.h>
#include <Servo.h> 
#include <stdlib.h>
#include <math.h>

#include <Hacked_servo.h>

#define NO_OF_SERVOS 4

#define ANGLE_MAX 90.0
#define ANGLE_MIN -90.0

//--Module ID-All
#define THETA0 -45.033886
#define THETA1 0.65493

/*
//--Module ID-1
#define THETA0_0 -45.819169
#define THETA1_0 0.64441

//--Module ID-2
#define THETA0_1 -45.812751
#define THETA1_1 0.651827

//--Module ID-3
#define THETA0_2 0.0
#define THETA1_2 0.0

//--Module ID-4
#define THETA0_3 -45.731538
#define THETA1_3 0.661175
*/


//--Module ID-6
#define THETA0_0 -45.435066
#define THETA1_0 0.660404

//--Module ID-7
#define THETA0_1 -45.107922
#define THETA1_1 0.68851

//--Module ID-4
#define THETA0_2 -45.731538
#define THETA1_2 0.661175

//--Module ID-5
#define THETA0_3 -45.916238
#define THETA1_3 0.643767


Hacked_servo myservo[NO_OF_SERVOS];
enum message_type{Request_Servo, Command, Request_Time, Turn_On_Broadcast, Turn_Off_Broadcast};

double myservo_previous_input[NO_OF_SERVOS];
bool broadcast_servo = false;

unsigned long time = 0;

void setup() 
{ 
  if(NO_OF_SERVOS >= 1)
  {
    myservo[0].attach(8,A0); // attaches the servo object 0 to pin A0 and 8 on ports 1 and 2 respectiverly.
  }
  
  if(NO_OF_SERVOS >= 2)
  {
    myservo[1].attach(9,A1); // attaches the servo object 1 to pin A1 and 9 on ports 3 and 4 respectiverly.
  }
  
  if(NO_OF_SERVOS >= 3)
  {
    myservo[2].attach(10,A2); // attaches the servo object 2 to pin A2 and 10 on ports 5 and 6 respectiverly.
  }
  
  if(NO_OF_SERVOS >= 4)
  {
    myservo[3].attach(11,A3); // attaches the servo object 3 to pin A3 and 11 on ports 7 and 8 respectiverly.
  }
  
  for(int servo=0; servo<NO_OF_SERVOS; servo++)
  {
    myservo_previous_input[servo] = 90;
  }
  // start serial port at 115200 bps:
  //Serial.begin(57600);
  //Serial.begin(111111);  // Baud Rate on the rest [PC, Xbee_Source, Xbee_Dest] should be 115200 kbps. Reason explained here
  Serial.begin(115200);
} 


void loop() 
{ 
  if(Serial.available()) //--If there is an incoming message.
  {
    char in_char = 'X';
    char m_type[1];
    char to[1] = {0};
    char data[4][10] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int i=0;
    int mType_i = 0;
    int to_i = 0;
    int data_i = 0;
    int servo = 0;
  
    message_type mType = Command;
  
    bool mType_available = false;
    bool to_available = false;
    bool data_available[4] = {false, false, false, false};
  
    in_char = Serial.read();
    if(in_char == '#')
    {
      do
      {
        while(!Serial.available());
        
        in_char = Serial.read();
        if(in_char == '$')
        {
          break;
        }
        else if(in_char == '*')
        {
          data_i = 0;
          do
          {
            while(!Serial.available());
            
            in_char = Serial.read();
            if(in_char == '*')
            {
              break;
            }
            else if(in_char == '$')
            {
              break;
            }
            else if(in_char == '<')
            {
              data_i = 0;
              do
              {
                while(!Serial.available());
                
                in_char = Serial.read();
                if(in_char == '>')
                {
                  if(data_i > 0)
                  {
                    data_available[servo] = true;
                  }
                  else
                  {
                    data_available[servo] = false;
                  }
                  break;
                }
                else if(in_char == '$')
                {
                  break;
                }
                else
                {
                  data[servo][data_i] = in_char;
                  data_i++;
                }
              }while(in_char != '>');
              servo++;
            }
          }while(in_char != '*');
        }
        else if(in_char == '&')
        {
          to_i = 0;
          do
          {
            while(!Serial.available());
            
            in_char = Serial.read();
            if(in_char == '&')
            {
              if(to_i > 0)
              {
                to_available = true;
              }
              else
              {
                to_available = false;
              }
              break;
            }
            else if(in_char == '$')
            {
              to_available = false;
              break;
            }
            else
            {
              to[to_i] = in_char;
              to_i++;
            }
          }while(in_char != '&');
        }
        else if(in_char == '%')
        {
          mType_i = 0;
          do
          {
            while(!Serial.available());
            
            in_char = Serial.read();
            if(in_char == '%')
            {
              if(mType_i > 0)
              {
                mType_available = true;
              }
              else
              {
                mType_available = false;
              }
              break;
            }
            else if(in_char == '$')
            {
              break;
            }
            else
            {
              switch(atoi(&in_char))
              {
                case 0: {
                  mType = Command;
                  mType_i++;
                  break;
                }
                case 1: {
                  mType = Request_Servo;
                  mType_i++;
                  break;
                }
                case 3: {
                  mType = Request_Time;
                  mType_i++;
                  break;
                }
                case 5: {
                  mType = Turn_On_Broadcast;
                  mType_i++;
                  break;
                }
                case 6: {
                  mType = Turn_Off_Broadcast;
                  mType_i++;
                  break;
                }
                default: {mType_i++; break;}
              }
            }
          }while(in_char != '%');
        }
      }while(in_char != '$');
    }
    else
    { }
    
    if(mType_available && to_available)  //--TODO: 'To' address should not be manditory. This should be changed in Y1ModularRobot::get_current_time(void)... 
                                         //-- ...and  Y1ModularRobot::Turn_On_Broascast(void) as well.
    {
      int servo_add = 0;
      servo_add = atoi(to);
      
      if(mType == Command)
      {
        float servo_value[4]={0,0,0,0};
        for(int i=0; i<servo; i++)
        {
          servo_value[i] = atof(data[i]);
          servo_value[i] = servo_value[i] + 90; // Scaling the input of the servo back to its original range of between 0º and 179º
        }
        
        switch(servo_add)
        {
          case 0: {
            if(myservo_previous_input[0] != servo_value[0])
            {
              myservo[0].write(servo_value[0]); // sets the servo position according to the scaled value
              myservo_previous_input[0] = servo_value[0];
            }
            break;
          }
          case 1: {
            if(myservo_previous_input[1] != servo_value[1])
            {
              myservo[1].write(servo_value[1]); // sets the servo position according to the scaled value
              myservo_previous_input[1] = servo_value[1];
            }
            break;
          }
          case 2: {
            if(myservo_previous_input[2] != servo_value[2])
            {
              myservo[2].write(servo_value[2]); // sets the servo position according to the scaled value
              myservo_previous_input[2] = servo_value[2];
            }
            break;
          }
          case 3: {
            if(myservo_previous_input[3] != servo_value[3])
            {
              myservo[3].write(servo_value[3]); // sets the servo position according to the scaled value
              myservo_previous_input[3] = servo_value[3];
            }
            break;
          }
          case 4: {
            for(int i=0; i<NO_OF_SERVOS; i++)
            //for(int i=0; i<servo; i++)
            {
              if(myservo_previous_input[i] != servo_value[i])
              {
                myservo[i].write(servo_value[i]); // sets the servo position according to the scaled value
                myservo_previous_input[i] = servo_value[i];
              }
            }
            //time = servo_value[3]; // To be removed.
            //digitalWrite(13, HIGH);
            break;
          }
          default: {
//---------------------------Debugger---------------------------//
            blink();
            String outString;
            unsigned char outBuf[100];
            outString = outString + servo_add;
            StringToCharArray(outString, outBuf);
            Serial.write(outBuf, servo_add);
            break;
//---------------------------Debugger---------------------------//
            break;
          }
        }
      }
      
      else if(mType == Request_Servo)
      {
        String outString;
        unsigned char outBuf[100];
        
        switch(servo_add)
        {
          case 0: {
            outString = OutputStringStream(servo_add, myservo[0].readPos_raw());
            StringToCharArray(outString, outBuf);
            Serial.write(outBuf, outString.length());
            break;
          }
          case 1: {
            outString = OutputStringStream(servo_add, myservo[1].readPos_raw());
            StringToCharArray(outString, outBuf);
            Serial.write(outBuf, outString.length());
            break;
          }
          case 2: {
            outString = OutputStringStream(servo_add, myservo[2].readPos_raw());
            StringToCharArray(outString, outBuf);
            Serial.write(outBuf, outString.length());
            break;
          }
          case 3: {
            outString = OutputStringStream(servo_add, myservo[3].readPos_raw());
            StringToCharArray(outString, outBuf);
            Serial.write(outBuf, outString.length());
            break;
          }
          case 4: {
            int message_size = OutputStringStream_All(servo_add,outBuf);
            Serial.write(outBuf, message_size);
            break;
          }
          default: {
//---------------------------Debugger---------------------------//
            /*blink();
            String outString;
            unsigned char outBuf[100];
            outString = outString + servo_add;
            StringToCharArray(outString, outBuf);
            Serial.write(outBuf, servo_add);
            break;*/
//---------------------------Debugger---------------------------//
            break;
          }
        }
      }
      
      else if(mType == Request_Time)
      {
        String outString;
        unsigned char outBuf[100];
        
        int n = OutputStringStream_Time(outBuf);
        Serial.write(outBuf, n);
      }
      
      else if(mType == Turn_On_Broadcast)
      {
        broadcast_servo = true;
        
        unsigned char outBuf[10];
        int n = Send_Ackn(outBuf, '7');
        Serial.write(outBuf, n);
      }
      
      else if(mType == Turn_Off_Broadcast)
      {
        broadcast_servo = false;

        /*unsigned char outBuf[10];
        int n = Send_Ackn(outBuf, '8');
        Serial.write(outBuf, n);*/
      }
    }
  }
  else if(!Serial.available() && broadcast_servo) //--If there is NO incoming message AND if Broadcast_Servo is ON.
  {
    unsigned int message_size = 0;
    unsigned char outBuf[100];
    
    message_size = OutputStringStream_All(4,outBuf);
    Serial.write(outBuf, message_size);
  }

}


int Send_Ackn(unsigned char* charBuf, char ackn_type)
{
  unsigned int charBufLength = 0;
    
  charBuf[charBufLength++] = '#';
  
  charBuf[charBufLength++] = '%';
  charBuf[charBufLength++] = ackn_type;
  charBuf[charBufLength++] = '%';

  charBuf[charBufLength++] = '$';

  return charBufLength;
}


String OutputStringStream(int from, double rawData)
{
  String serialOut;
  int base10Exp = 2;
  double predictedAngle = 0;
  
  if(from==0)
  {
    predictedAngle = (THETA0_0 + (rawData * THETA1_0)) - 90;
  }
  else if(from==1)
  {
    predictedAngle = (THETA0_1 + (rawData * THETA1_1)) - 90;
  }
  else if(from==2)
  {
    predictedAngle = (THETA0_2 + (rawData * THETA1_2)) - 90;
  }
  else if(from==3)
  {
   predictedAngle = (THETA0_3 + (rawData * THETA1_3)) - 90;
  }
  
  long longData = predictedAngle * pow(10, base10Exp);

  serialOut = serialOut + "#";
  serialOut = serialOut + "%" + "2" + "%";
  serialOut = serialOut + "&" + from + "&";
  serialOut = serialOut + "*<" + longData + "/" + base10Exp + ">*";
  serialOut = serialOut + "$";

  return serialOut;
}


unsigned int OutputStringStream_All(int from, unsigned char* charBuf)
{
  String fromString;
  String timeString;
  int base10Exp = 2;  //-- This parameter has to be the same both here and the PC code [bool Y1ModularRobot::decode_message_with_individual_time(const char, vector<double>&)].
  unsigned int charBufLength = 0;
  unsigned long time_main = 0;
  unsigned long time_individualServo_read[NO_OF_SERVOS];
  double rawData[NO_OF_SERVOS];
  double predictedAngle = 0;
  unsigned int module, i;

  charBuf[charBufLength++] = '#';
  
  charBuf[charBufLength++] = '%';
  charBuf[charBufLength++] = '2'; // Message Type 2 --> Send Servo with Time
  charBuf[charBufLength++] = '%';
  
  charBuf[charBufLength++] = '&';
  fromString = fromString + from;
  for(i=0;i<fromString.length();i++)
  {
    charBuf[charBufLength++] = fromString[i];
  }
  charBuf[charBufLength++] = '&';
  
  //-- Read Time (Milliseconds).
  time_main = micros();
  
  //-- Read Servo positions.
  for(module=0; module<NO_OF_SERVOS; module++)
  {
    time_individualServo_read[module] = micros();
    rawData[module] = myservo[module].readPos_raw();
  }

//----------------TIME-(MILLISECONDS)-RELATED----------------//
  charBuf[charBufLength++] = '[';
  timeString = timeString + time_main;
  //timeString = timeString + time++; // To be removed.
  for(i=0; i<timeString.length(); i++)
  {
    charBuf[charBufLength++] = timeString[i];
  }
  charBuf[charBufLength++] = ']';
//----------------TIME-(MILLISECONDS)-RELATED----------------//
  
  charBuf[charBufLength++] = '*';
  for(module=0; module<NO_OF_SERVOS; module++)
  {
    charBuf[charBufLength++] = '<';
    
    if(module == 0)
    {
      predictedAngle = (THETA0_0 + (rawData[module] * THETA1_0)) - 90;
    }
    else if(module == 1)
    {
      predictedAngle = (THETA0_1 + (rawData[module] * THETA1_1)) - 90;
    }
    else if(module == 2)
    {
      predictedAngle = (THETA0_2 + (rawData[module] * THETA1_2)) - 90;
    }
    else if(module == 3)
    {
      predictedAngle = (THETA0_3 + (rawData[module] * THETA1_3)) - 90;
    }
    
    //--Temperary fix for prediction offset that exists in the linear model.
    predictedAngle = predictedAngle - 5.5;
    
    if(predictedAngle > ANGLE_MAX)
    {
      predictedAngle = ANGLE_MAX;
    }
    else if(predictedAngle < ANGLE_MIN)
    {
      predictedAngle = ANGLE_MIN;
    }
    
    long longServoData = predictedAngle * pow(10, base10Exp);
    
    String dataString;
    //dataString = dataString + longServoData + '/' + int(time_main - time_individualServo_read[module]);  //BUG: Fixed in the below line
    dataString = dataString + longServoData + '/' + (unsigned int)(time_individualServo_read[module] - time_main);
    for(i=0;i<dataString.length();i++)
    {
      charBuf[charBufLength++] = dataString[i];
    }
   
    charBuf[charBufLength++] = '>';
  }
  charBuf[charBufLength++] = '*';
  
  charBuf[charBufLength++] = '$';

  return charBufLength;
}


int OutputStringStream_Time(unsigned char* charBuf)
{
  String timeString;
  unsigned int charBufLength = 0;
  unsigned long time_main=0;
  unsigned int i;
    
  charBuf[charBufLength++] = '#';
  
  charBuf[charBufLength++] = '%';
  charBuf[charBufLength++] = '4'; // Message Type 4 --> Send Time
  charBuf[charBufLength++] = '%';
    
  //-- Read Time (Milliseconds).
  time_main = micros();

//----------------TIME-(MILLISECONDS)-RELATED----------------//
  charBuf[charBufLength++] = '[';
  timeString = timeString + time_main;
  for(i=0; i<timeString.length(); i++)
  {
    charBuf[charBufLength++] = timeString[i];
  }
  charBuf[charBufLength++] = ']';
//----------------TIME-(MILLISECONDS)-RELATED----------------//
    
  charBuf[charBufLength++] = '$';

  return charBufLength;
}


void StringToCharArray(String sourceString, unsigned char* charBuf)
{
  for(int i=0; i<sourceString.length(); i++)
  {
    charBuf[i] = sourceString[i];
  }
}


void blink() {
  for(int i=0; i<2; i++)
  {
    digitalWrite(13, HIGH);   // set the LED on
    delay(10);              // wait for a second
    digitalWrite(13, LOW);    // set the LED off
    delay(10);    // wait for a second
  }
  //digitalWrite(13, HIGH);
}


void blink_long() {
  for(int i=0; i<10; i++)
  {
    digitalWrite(13, HIGH);   // set the LED on
    delay(50);              // wait for a second
    digitalWrite(13, LOW);    // set the LED off
    delay(50);    // wait for a second
  }
  digitalWrite(13, HIGH);
}
