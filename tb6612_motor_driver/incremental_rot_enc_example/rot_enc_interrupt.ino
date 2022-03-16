 /*     Arduino Rotary Encoder Tutorial
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
 
 //Modified to use interrupt pins
 
 #define outputA 6
 #define outputB 7

 int counter = 0; 
 int aState;
 int aLastState;  

 void setup() { 
   pinMode (outputA,INPUT);
   pinMode (outputB,INPUT);
   
   attachInterrupt(outputA, isr_a, CHANGE); //Triggers interrupt when pin goes low 
   
   Serial.begin (9600);
   // Reads the initial state of the outputA
   aLastState = digitalRead(outputA);   
 } 
                   
void isr_a() {
  aState = digitalRead(outputA); // Reads the "current" state of the output
   // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
   if (digitalRead(outputB) != aState) { 
     counter ++;
   } else {
     counter --;
   }
   Serial.print("Position: ");
   Serial.println(counter);

}

 void loop() { 

 }
