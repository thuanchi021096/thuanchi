// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
 
#include <Pixy2.h>

Pixy2 pixy;
double kp = 5;
double ki = 0.005;
double kd = 0.01;
#define RIGHT_FWD 4 
#define RIGHT_REV 7
#define LEFT_FWD 2
#define LEFT_REV 3 
#define RIGHT_PWM 6
#define LEFT_PWM 5
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
double OUT_PID,Input_PID,temp_PID;
bool check_dir=false,stop_motor=false;
int time_mili=0;
int c=0,d=0,a=0,b=0;
void setup()
{
  pinMode(RIGHT_FWD,OUTPUT);
  pinMode(RIGHT_REV,OUTPUT);
  pinMode(LEFT_FWD,OUTPUT);
  pinMode(LEFT_REV,OUTPUT);
  Serial.begin(115200);
  Serial.print("Starting...\n");
 setPoint = 42;
  pixy.init();
  // change to the line_tracking program.  Note, changeProg can use partial strings, so for example,
  // you can change to the line_tracking program by calling changeProg("line") instead of the whole
  // string changeProg("line_tracking")
  Serial.println(pixy.changeProg("line"));
}
 
void loop()
{
  int8_t i;
  char buf[128];
 if(!stop_motor){
    pixy.line.getMainFeatures();
    if(check_dir)
    {
      delay(1000);
      if (pixy.line.numVectors)
      {
         a=pixy.line.vectors->m_x0;
         b= pixy.line.vectors->m_x1;
        
       }
      LEFT_FORWARD();
      RIGHT_FORWARD();
      if(c==1)
      {
        
        
          analogWrite(RIGHT_PWM,110);
          analogWrite(LEFT_PWM,70);
          delay(2000);
          analogWrite(RIGHT_PWM,0);
          analogWrite(LEFT_PWM,0);
          pixy.line.getMainFeatures();
            if (pixy.line.numVectors)
            {
             a=pixy.line.vectors->m_x0;
             b= pixy.line.vectors->m_x1;
            }
             time_mili=millis();
          while(((a+b)/2>=40))
          {
            Serial.print("ab");
            Serial.println((a+b)/2);
            Serial.print("a");
            Serial.println(a);
            Serial.print("b");
            Serial.println(b);
             //delay(10000000);
            analogWrite(RIGHT_PWM,110);
            analogWrite(LEFT_PWM,70);
           int time_temp=millis();
            if((time_temp-time_mili)>=5000)
            {
              analogWrite(RIGHT_PWM,110);
              analogWrite(LEFT_PWM,50);
            }
            delay(1000);
            analogWrite(RIGHT_PWM,0);
            analogWrite(LEFT_PWM,0);
            pixy.line.getMainFeatures();
            if (pixy.line.numVectors)
            {
             a=pixy.line.vectors->m_x0;
             b= pixy.line.vectors->m_x1;
            }
            
          }
          //time_mili=millis();
          
          check_dir=false;
          c=0;
        
      }
      if(c==7)
      {
        
        
        analogWrite(RIGHT_PWM,70);
        analogWrite(LEFT_PWM,110);
        delay(1000);
        analogWrite(RIGHT_PWM,0);
        analogWrite(LEFT_PWM,0);
        pixy.line.getMainFeatures();
        if (pixy.line.numVectors)
        {
           a=pixy.line.vectors->m_x0;
           b= pixy.line.vectors->m_x1;
        }
        time_mili=millis();
        while(((a+b)/2<=20))
        {
            if((time_mili-5000)>=0)
            {
              analogWrite(LEFT_PWM,150);
            }
            Serial.print("ab");
            Serial.println((a+b)/2);
            Serial.print("a");
            Serial.println(a);
            Serial.print("b");
            Serial.println(b);
             //delay(10000000);
            analogWrite(RIGHT_PWM,70);
            analogWrite(LEFT_PWM,110);
            
            delay(1000);
            
            analogWrite(RIGHT_PWM,0);
            analogWrite(LEFT_PWM,0);
            pixy.line.getMainFeatures();
            if (pixy.line.numVectors)
            {
             a=pixy.line.vectors->m_x0;
             b= pixy.line.vectors->m_x1;
            }
            
        }
        check_dir=false;
        c=0;
        
      }
      
    }
    if (pixy.line.barcodes){
      
      //pixy.line.barcodes->print();
       c=pixy.line.barcodes->m_code;
      
      Serial.println(c);
      if(c==1|c==7) {
        analogWrite(RIGHT_PWM,100);
        analogWrite(LEFT_PWM,100);
        delay(1500);
        analogWrite(RIGHT_PWM,0);
        analogWrite(LEFT_PWM,0);
        check_dir=true;
      }
      if(c==3)
      {
        analogWrite(RIGHT_PWM,100);
        analogWrite(LEFT_PWM,100);
        delay(6000);
      }
      if(c==5)
      {
        STOP();
        stop_motor=true;
        check_dir=true;
      }
    }
    if(!check_dir)
    {
      if (pixy.line.numVectors)
      {
        pixy.line.vectors->print();
         a=pixy.line.vectors->m_x0;
         b= pixy.line.vectors->m_x1;
        //Serial.println(a);
        //Serial.println(b);
        Input_PID=(a+b)/2;
        Serial.println(Input_PID);
        temp_PID=computePID(Input_PID);
        OUT_PID=constrain(temp_PID,-150,150);
        Serial.println(error);
        Serial.println(OUT_PID);
        if(-5<=error && error<=5)
        {
          //cumError=0;
          LEFT_FORWARD();
          RIGHT_FORWARD();
          analogWrite(RIGHT_PWM,100);
          analogWrite(LEFT_PWM,100);
          //Serial.println(123);
        }
        else if(error>5)
        {
          
          LEFT_FORWARD();
          RIGHT_FORWARD();
          analogWrite(RIGHT_PWM,OUT_PID);
          analogWrite(LEFT_PWM,75);
         
        }
        else if(error<-5)
        {
          LEFT_FORWARD();
          RIGHT_FORWARD();
          
          OUT_PID*=-1;
          analogWrite(RIGHT_PWM,75);
          analogWrite(LEFT_PWM,OUT_PID);
          
        }
        
        
      }
    }
    if (pixy.line.numIntersections)
    {
      
      pixy.line.intersections->print();
      
      Serial.println(pixy.line.intersections->m_n);
      Serial.println(c);
  //    analogWrite(RIGHT_PWM,0);
  //    analogWrite(LEFT_PWM,0);
  //    delay(100000);
    }
 
 }
}
double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = setPoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
void LEFT_FORWARD()
{
  digitalWrite(LEFT_FWD,HIGH);
  digitalWrite(LEFT_REV,LOW);
}
void RIGHT_FORWARD()
{
  digitalWrite(RIGHT_FWD,HIGH);
  digitalWrite(RIGHT_REV,LOW);
}
void STOP()
{
   digitalWrite(RIGHT_FWD,LOW);
  digitalWrite(RIGHT_REV,LOW);
   digitalWrite(LEFT_FWD,LOW);
  digitalWrite(LEFT_REV,LOW);
  analogWrite(RIGHT_PWM,0);
  analogWrite(LEFT_PWM,0);
}
