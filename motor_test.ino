#include <Servo.h>
#include <AFMotor.h>
#include <Pixy2.h>
#include <Pixy2SPI_SS.h>
#include "math.h"

//
float runtime_init=0,runtime_final=0,runtime=0;
// DATA compressing
const int readMax = 3;  
float readings[2][readMax]={0}; 
int readIndex = 0; //reading[readIndex]
float total[2][1] = {0};  
float avg[2] = {0}; 

Pixy2 pixy;
// frequent constant
//166,92
float origin[2]={164,93};
// before smoothing float kp=(0.22222)*1.80,kd=5.8, ki=0.001;
float kp=(0.22222)*1.55,kd=5.8, ki=0.00012;

//pixy variables
float left_end=58,right_end=251,top_end=7,down_end=190;
//float origin[2]={(left_end+right_end)/2,(top_end+down_end)/2}; //x,y 

float ball[2]={0};
float ball_avg[2]={0};

//Inverse Kin ----------------------------
	// link length
	float a1 = 7.5, a2 = 11;
  float space=0;
	float L = 10.5;
	//int offset_H=2, 
	float H = a2;

	//motor angle limit 
  float plate_angle_limit=30;
	float motor_limit = 30; //deg
	float max_motor_1_rad = abs(motor_limit) * ((2*M_PI) / 360);
	float max_motor_2_rad = abs(motor_limit) * ((2 * M_PI) / 360);
	// set plate deg
	float alpha_deg = 0, beta_deg = 0; //alpha=motor_1 beta=motor_2

	//index for motor1 and 2
	int index_1 = 1, index_2 = 1;

	//end point
	float px[2] = { 0 }, py[2] = {0}; // {x_1,x_2} {y_1,y_2}

	//th2 th1
	// th2: {{M1_th2, M1'_th2},{M2_th2, M2'_th2}}  th1:{{M1_th1, th_1},{M2_th1, th1}} [motor_index=0,1][n_th sol]
	double th2[2][2] = { 0 }, th1[2][2] = {0};

	//th2 th1 to deg
	double th2_deg[2][2] = { 0 }, th1_deg[2][2] = { 0 };

//---------------------------------------
//define servo number
Servo servo1; 
Servo servo2; 
#define home_angle 90
#define motor2_x_offset 1
#define motor2_offset -5
#define motor1_offset 2

// constants
int x = 0, y = 1, z = 2; 
int a=0, b=1; //motor_a, motor_b

//PID variables-----------------------
//gain
//float kp=(0.22222)*0.5,kd=25, ki=0;

//error
float pos_avg[2][1]={0};
double e_max[2]={97,91};//error max_x y
double e[2]; //error term
double e_prev[2];
//D
double d[2];       // derivative of the error
float d_prev[2];  // previous derivative value
//I
double i[2] ={0,0}; //integrator
//PID_out
float P[2]={0,0};
float I[2]={0,0};
float D[2]={0,0};
float PID[2]={0,0}; //PID OUT
//time
float time_init;         // time_init
float time_final;         // time_final
float time_delta;           //delta time
//coordinate
double r_ball;
float r_limit=251-origin[0];
//------------------------------------
void setup() {
  //camera INIT()
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  pixy.init();
  Serial.println(pixy.changeProg("line"));

  //servo INIT()
  servo1.attach(10);  //ser1 -> pin_10
  servo2.attach(9); //ser2 -> pin_9
  servo1.write(90);
  servo2.write(90);

// Filter clear
  for (int i = 0; i < readMax; i++) {
    readings[x][i] = pixy.ccc.blocks[0].m_x; 
    delay(2);
    readings[y][i] = pixy.ccc.blocks[0].m_y;  
    delay(2);
        total[x][0] = total[x][0] + readings[x][i];
        total[y][0] = total[y][0] + readings[y][i];
  }
}

void loop() {
  // runtime_init=millis();
  getBall();
  // runtime_final=millis();
  // runtime=runtime_final-runtime_init;
  // Serial.println((String)"Run time: ["+runtime+"]"); //print location of the ball


  //Serial.print(avg[y]);
  //Serial.print(" "); // a space ' ' or  tab '\t' character is printed between the two values.
  //Serial.println(ball[y]);

//control();
//servo_test();
  
 /*

 for (pos = 0; pos <= 180; pos += 1) { 
    
    servo1.write(0);     
         
    delay(20);                       
  }
  */
   //  if(ball[x]==-1 && ball[y]==-1) // no ball detected
  // {
  //    Serial.println((String)"BALL LOCATION: [" + ball[x]+", "+ball[y]+"]"); //print location of the ball
  //    // servo -> 0,0
  //  }
  //  else // routine
  //  {
  //    Serial.println((String)"BALL LOCATION: [" + ball[x]+", "+ball[y]+"]"); //print location of the ball
  //    //delay(10)

  //  }

 //Serial.println((String)"BALL LOCATION: [" + ball[x]+", "+ball[y]+"]"); //print location of the ball
 //Serial.println((String)"PID[x]: [" +PID[x]+ "]" "PID[y]: ["+PID[y]+ "]");
 /*
 Serial.println((String)"BALL RADIUS: [" +r_ball+ "]"); // print radius of the ball from the origin
 Serial.println((String)"PD_error: [" +e[x]+ "," +e[y]+ "]"); // print PD error term
 */
}

void getBall()//get ball's position
{
//plate init
  pixy.ccc.getBlocks(); //get blocks data from the pixy

  if(pixy.ccc.numBlocks==1)// number of the block==1
  {
    
// smoothing data
for(int i=0;i<2;i++)
{
  if(i==x)
  {
  total[i][0] = total[i][0] - readings[i][readIndex];  
  readings[i][readIndex] = pixy.ccc.blocks[0].m_x;
  delay(2);
  total[i][0] = total[i][0] + readings[i][readIndex];  
  readIndex = readIndex + 1;
  if (readIndex >= readMax) {  
    readIndex = 0;
  }
  avg[x] = total[i][0]/readMax;
  }

  else 
  {
      total[i][0] = total[i][0] - readings[i][readIndex];  
  readings[i][readIndex] = pixy.ccc.blocks[0].m_y;
  delay(2);
  total[i][0] = total[i][0] + readings[i][readIndex];  
  readIndex = readIndex + 1;
  if (readIndex >= readMax) {  
    readIndex = 0;
  }
  avg[y] = total[i][0] / readMax;
  }
}
  ball[x]=avg[x];
  ball[y]=avg[y];
 
    // for signature 0 object
    //  ball[x]=pixy.ccc.blocks[0].m_x;//The x location of the center of the detected object (0 to 316)
    //  ball[y]=pixy.ccc.blocks[0].m_y;//The y location of the center of the detected object (0 to 208)

  //Serial.print(avg[y]);
  //Serial.print(" "); // a space ' ' or  tab '\t' character is printed between the two values.
  //Serial.println(ball[y]);

    control();
  }

  else if(pixy.ccc.numBlocks>1)//multiple balls detected
  {
     ball[x]=-1;
     ball[y]=-1;
     Serial.println("Multiple balls detected");
  }
  else //no ball detected=set to zero
  {
    //Serial.println("No ball detected");
    ball[x]=-1;
    ball[y]=-1;
  }
}

void control()
{
  // stage1: calculate error_term = e(t)
  e[x] = ball[x] - origin[x];  // error of x-position
  e[y] = ball[y] - origin[y];  // error of y-position
  // Serial.print(e[x]);
  // Serial.print(" "); // a space ' ' or  tab '\t' character is printed between the two values.
  // Serial.println(e[y]);

  // stage2: calculate d_term = v(t)
  time_final = millis();       // set final time
  time_delta = time_final - time_init;  // change in time
  time_init = millis();
  //Serial.println((String)"delta_t:" +time_delta+""); //v(x,y)

  d[x] = (e[x] - e_prev[x]) / time_delta;  // x component of derivative
  d[y] = (e[y] - e_prev[y]) / time_delta;  // y component of derivative
  //Serial.println((String)"d[x,y]: => [" +d[x]+ ","+d[y]+ "]"); //v(x,y)

  //stage3: calculate I_term =integral(e(t))dt
  i[x]+=e[x]+e_prev[x];
  i[y]+=e[y]+e_prev[y];
  for(int j=0;j<2;j++)
  {
    if(j==x)
    {
      if(i[j]>=300)
      {
        i[j]=300;
      }
      else if(i[j]<=-300)
      {
        i[j]=-300;
      }

    }
    else if(j==y)
    {
      if(i[j]>=300)
      {
        i[j]=300;
      }
      else if(i[j]<=-300)
      {
        i[j]=-300;
      }
    }
  }
  //Serial.println(i[x]);

  e_prev[x]=e[x]; //set x_error
  e_prev[y]=e[y]; //set y_error
  r_ball=sqrt(pow(e[x],2)+pow(e[y],2));
  Serial.println(r_ball); //print location of the ball

  if(r_ball>140) //out of range
  {
    getBall();
  }
  else//prop_out for x and y
  {
    P[x]=kp*e[x];
    P[y]=kp*e[y];
    D[x]=kd*d[x];
    D[y]=kd*d[y];

    I[x]=ki*-1*i[x];
    //Serial.println(I[x]);
    I[y]=ki*i[y];
    //Serial.println((String)"I[x,y]: => [" +I[x]+ ","+I[y]+ "]"); //Integral

    PID[x]=kp*P[x]+kd*D[x]+ki*I[x];
     if(PID[x]>plate_angle_limit) //postive:(+)
    {
      PID[x]=plate_angle_limit;
    }
    else if(-1*plate_angle_limit>PID[x])//negative:(-)
    {
      PID[x]=-1*plate_angle_limit;
    }
    
    PID[y]=kp*P[y]+kd*D[y]+ki*I[y];
       if(PID[y]>plate_angle_limit)
    {
      PID[y]=plate_angle_limit;
    }
     else if(-1*plate_angle_limit>PID[y])
    {
      PID[y]=-1*plate_angle_limit;
    }

  Inverse_kin(PID[x],PID[y]);
  //Serial.println((String)"Deriv_x: => [" +D[x]+ "]"); //print location of the ball
    
    //Serial.println((String)"prop_x: => [" +P[x]+ "]"); //print location of the ball
    //Serial.println((String)"prop_y: => [" +P[y]+ "]"); //print location of the ball
  }
}


void Inverse_kin(float prop_x, float prop_y)
{

  // stage 1: check valid angle
  //Serial.println((String)"prop_x: => [" +prop_x+ "]"); //print location of the ball
  //Serial.println((String)"prop_y: => [" +prop_y+ "]"); //print location of the ball
  alpha_deg=prop_y;
  beta_deg=prop_x;

  // stage 2: calculate end point
	for (int i = 0; i < 2; i++)
	{
		if (i == 0) //motor1
		{
			px[i] = a1 - L * (1 - cos(alpha_deg * ((2 * M_PI) / 360)));
			py[i] = H + L * sin(alpha_deg * ((2 * M_PI) / 360));
            //Serial.println((String)"MOTOR1_endpoint => px:" +px[i]+"  py"+py[i]+""); //print location of the ball
      
		}

		else if (i == 1) //motor2
		{
			px[i] = a1 - L * (1 - cos(beta_deg * ((2 * M_PI) / 360)));
			py[i] = H + L * sin(beta_deg * ((2 * M_PI) / 360));
            //Serial.println((String)"MOTOR2_endpoint => px:" +px[i]+"  py"+py[i]+""); //print location of the ball
		}
	}


	 //stage 3: calculate th2, th1
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			if (j == 0)//first solution
			{
				th2[i][j] = 2 * atan(pow((((a1 + a2) * (a1 + a2) - (px[i] * px[i] + py[i] * py[i])) / (px[i] * px[i] + py[i] * py[i] - (a1 - a2) * (a1 - a2))), 0.5));
				th2_deg[i][j] = th2[i][j] * (360 / (2 * M_PI));
				th1[i][j] = atan2(py[i], px[i]) - atan(a2 * sin(th2[i][j]) / (a1 + a2 * cos(th2[i][j])));
				th1_deg[i][j] = th1[i][j] * (360 / (2 * M_PI));

        // if(i==0)//data check
        // {
        //   Serial.println((String)"MOTOR1_1st sol => th2_deg:" +th2[i][j]+"  th1_deg:"+th1[i][j]+""); //print location of the ball
        

        // }
        // else if(i==1)
        // {
        //   Serial.println((String)"MOTOR2_1st sol => th2_deg:" +th2[i][j]+"  th1_deg:"+th1[i][j]+""); //print location of the ball

        // }

			}
			else if (j == 1)//second solution
			{
				th2[i][j] = -2 * atan(pow((((a1 + a2) * (a1 + a2) - (px[i] * px[i] + py[i] * py[i])) / (px[i] * px[i] + py[i] * py[i] - (a1 - a2) * (a1 - a2))), 0.5));
				th2_deg[i][j] = th2[i][j] * (360/(2*M_PI));
				th1[i][j] = atan2(py[i], px[i]) - atan(a2 * sin(th2[i][j]) / (a1 + a2 * cos(th2[i][j])));
				th1_deg[i][j] = th1[i][j] * (360 / (2 * M_PI));
        
        // if(i==0)//data check 
        // {
        //   Serial.println((String)"MOTOR1_2nd sol => th2_deg:" +th2[i][j]+"  th1_deg:"+th1[i][j]+""); //print location of the ball

        // }
        // else if(i==1)
        // {
        //   Serial.println((String)"MOTOR2_2nd sol => th2_deg:" +th2[i][j]+"  th1_deg:"+th1[i][j]+""); //print location of the ball

        // }
			}
		}
	}


//Serial.println((String)"alpha => [th1_deg:" ); //print location of the ball

	//consider condition
  //Serial.println((String)"alpha => [th1_deg:" +max_motor_1_rad+ ""); //print location of the ball
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			if (i == 0)//motor1
			{
				if (abs(th1[i][j]) < max_motor_1_rad)
				{
          
					index_1 = j;
          
          th1_deg[i][index_1]=(90)-th1_deg[i][index_1];
          //Serial.println((String)"alpha_deg => [" +th1_deg[i][index_1]+ "] +90 compensated"); //print location of the ball
				}	
			}

			else if (i == 1)//motor2
			{
				if (abs(th1[i][j]) <= max_motor_2_rad)
				{
					index_2 = j;
          th1_deg[i][index_2]+=(90);
          //Serial.println((String)"beta_deg => [" +th1_deg[i][index_2]+ "] +90 compensated"); //print location of the ball
				}
			}
		}
	}
//PID out
  servo(th1_deg[0][index_1],th1_deg[1][index_2]); //motor1, motor2
}

void servo(int motor1_deg, int motor2_deg)
{

    servo1.write(motor1_deg);//alpha: y-axis     
    servo2.write(motor2_deg);//beta: x-axis    
    //delay(10);       
  
}



