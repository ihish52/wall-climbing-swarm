#include "motor_controller.h"

//Motor encoder variables
int a1State;
int a1LastState; 
int a2State;
int a2LastState;
//motor encoder counter variables in main file
extern int counter1;
extern int counter2;

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
extern const int offsetA;
extern const int offsetB;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
extern Motor motor1;
extern Motor motor2;

//extern Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY, PWM_CH_A);
//extern Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY, PWM_CH_B);

void setupMotors()
{
   pinMode (outputA1,INPUT);
   pinMode (outputB1,INPUT);
   pinMode (outputA2,INPUT);
   pinMode (outputB2,INPUT);
   
   // Reads the initial state of the outputA
   a1LastState = digitalRead(outputA1);
   a2LastState = digitalRead(outputA2);
}

//must run in every loop
void updateEnc()
{
	a1State = digitalRead(outputA1); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (a1State != a1LastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB1) != a1State) { 
       counter1 ++;
     } else {
       counter1 --;
     }
   } 
   a1LastState = a1State; // Updates the previous state of the outputA with the current state
   
   //repeat for counter 2
   a2State = digitalRead(outputA2); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (a2State != a2LastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB2) != a2State) { 
       counter2 ++;
     } else {
       counter2 --;
     }
   } 
   a2LastState = a2State; // Updates the previous state of the outputA with the current state
}

