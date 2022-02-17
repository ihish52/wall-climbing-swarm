#include "led_controller.h"

void LED_controller::setup(){
	pixels.setPin(PIN);
	pixels.updateLength(NUMPIXELS);
	pixels.updateType(NEO_GRB + NEO_KHZ800);
	pixels.begin();
	pixels.clear();
}

void LED_controller::led_function(){
	
	// -----RESET--------//
	for (uint8_t i = 0; i < 13; i++){
		toggle_digit_1[i] = 0;
		toggle_digit_2[i] = 0;
		toggle_digit_3[i] = 0;
		
		H_digit_1[i] = 0;
		H_digit_2[i] = 0;
		H_digit_3[i] = 0;
		
		S_digit_1[i] = 1;
		S_digit_2[i] = 1;
		S_digit_3[i] = 1;
		
		V_digit_1[i] = 100;
		V_digit_2[i] = 100;
		V_digit_3[i] = 100;
		
	}
	
	toggle_decimal_1 = 0;
	toggle_decimal_2 = 0;
	
	H_decimal_1 = 0;
	H_decimal_2 = 0;
	
	S_decimal_1 = 1;
	S_decimal_2 = 1;
	
	V_decimal_1 = 100;
	V_decimal_2 = 100;
	
	
	 //------ENABLE LEDS----------//
	//PROGRESS BAR
	if(modes == 0 || modes == 4){
		toggle_digit_1[6] = bar(10,0);
		toggle_digit_1[7] = bar(9,0);
		toggle_decimal_1 = bar(8,0);
		toggle_digit_2[5] = bar(7,0);
		toggle_digit_2[6] = bar(6,0);
		toggle_digit_2[7] = bar(5,0);
		toggle_decimal_2 = bar(4,0);
		toggle_digit_3[5] = bar(3,0);
		toggle_digit_3[6] = bar(2,0);
		toggle_digit_3[7] = bar(1,0);
		
		H_digit_1[6] = bar(10,1);
		H_digit_1[7] = bar(9,1);
		H_decimal_1 = bar(8,1);
		H_digit_2[5] = bar(7,1);
		H_digit_2[6] = bar(6,1) ;
		H_digit_2[7] = bar(5,1);
		H_decimal_2 = bar(4,1);
		H_digit_3[5] = bar(3,1);
		H_digit_3[6] = bar(2,1);
		H_digit_3[7] = bar(1,1);
		
		S_digit_1[6] = bar(10,2);
		S_digit_1[7] = bar(9,2);
		S_decimal_1 = bar(8,2);
		S_digit_2[5] = bar(7,2);
		S_digit_2[6] = bar(6,2);
		S_digit_2[7] = bar(5,2);
		S_decimal_2 = bar(4,2);
		S_digit_3[5] = bar(3,2);
		S_digit_3[6] = bar(2,2);
		S_digit_3[7] = bar(1,2);
		
		V_digit_1[6] = bar(10,3);
		V_digit_1[7] = bar(9,3);
		V_decimal_1 = bar(8,3);
		V_digit_2[5] = bar(7,3);
		V_digit_2[6] = bar(6,3);
		V_digit_2[7] = bar(5,3);
		V_decimal_2 = bar(4,3);
		V_digit_3[5] = bar(3,3);
		V_digit_3[6] = bar(2,3);
		V_digit_3[7] = bar(1,3);
		
	}

	//COMPASS
	if(modes == 2 || modes == 3 || modes == 6){
		toggle_digit_1[0] = compass(8,0);
		toggle_digit_1[2] = compass(1,0);
		toggle_digit_1[4] = compass(0,0);
		toggle_digit_1[5] = compass(7,0);
		toggle_digit_1[6] = compass(0,0);
		toggle_digit_1[7] = compass(0,0);
		toggle_digit_1[9] = compass(0,0);
		toggle_digit_1[10] = compass(6,0);
		toggle_digit_1[12] = compass(5,0);
		toggle_decimal_1 = compass(0,0);
		toggle_digit_2[0] = compass(2,0);
		toggle_digit_2[5] = compass(3,0);
		toggle_digit_2[10] = compass(4,0);
		
		H_digit_1[0] = compass(8,1);
		H_digit_1[2] = compass(1,1);
		H_digit_1[4] = compass(0,1);
		H_digit_1[5] = compass(7,1);
		H_digit_1[6] = compass(0,1);
		H_digit_1[7] = compass(0,1);
		H_digit_1[9] = compass(0,1);
		H_digit_1[10] = compass(6,1);
		H_digit_1[12] = compass(5,1);
		H_decimal_1 = compass(0,1);
		H_digit_2[0] = compass(2,1);
		H_digit_2[5] = compass(3,1);
		H_digit_2[10] = compass(4,1);
		
		S_digit_1[0] = compass(8,2);
		S_digit_1[2] = compass(1,2);
		S_digit_1[4] = compass(0,2);
		S_digit_1[5] = compass(7,2);
		S_digit_1[6] = compass(0,2);
		S_digit_1[7] = compass(0,2);
		S_digit_1[9] = compass(0,2);
		S_digit_1[10] = compass(6,2);
		S_digit_1[12] = compass(5,2);
		S_decimal_1 = compass(0,2);
		S_digit_2[0] = compass(2,2);
		S_digit_2[5] = compass(3,2);
		S_digit_2[10] = compass(4,2);
		
		V_digit_1[0] = compass(8,3);
		V_digit_1[2] = compass(1,3);
		V_digit_1[4] = compass(0,3);
		V_digit_1[5] = compass(7,3);
		V_digit_1[6] = compass(0,3);
		V_digit_1[7] = compass(0,3);
		V_digit_1[9] = compass(0,3);
		V_digit_1[10] = compass(6,3);
		V_digit_1[12] = compass(5,3);
		V_decimal_1 = compass(0,3);
		V_digit_2[0] = compass(2,3);
		V_digit_2[5] = compass(3,3);
		V_digit_2[10] = compass(4,3);
	}
	
	//PACE
	if(modes == 1){
		for(uint8_t i = 0; i < 13; i++){
			toggle_digit_1[i] = 0;
			toggle_digit_2[i] = 0;
			toggle_digit_3[i] = 0;
		}
		
		
		digits_control();
    
	//-----Select colour----120degrees = green--//
		for(uint8_t i = 0; i < 13; i++){
			H_digit_1[i] = 300;
			H_digit_2[i] = 300;
			H_digit_3[i] = 300;
		}
		H_decimal_1 = 300;
		H_decimal_2 = 300;
		
		for(uint8_t i = 0; i < 13; i++){
			S_digit_1[i] = 1;
			S_digit_2[i] = 1;
			S_digit_3[i] = 1;
		}
		S_decimal_1 = 1;
		S_decimal_2 = 1;
		
	//------Brightness------//
		for(uint8_t i = 0 ; i < 13 ; i++){
			V_digit_1[i] = 50;
			V_digit_2[i] = 50;
			V_digit_3[i] = 50;
		}
		V_decimal_1 = 50;
		V_decimal_2 = 50;
	}
		
	if(modes == 5){ //NO
		for(uint8_t i = 0; i < 13; i++){
			toggle_digit_1[i] = 0;
			toggle_digit_2[i] = 0;
			toggle_digit_3[i] = 0;
		}
		
		no_fix_text();
		
		for(uint8_t i = 0; i < 13; i++){
			H_digit_1[i] = 0;
			H_digit_2[i] = 0;
			H_digit_3[i] = 0;
		}
		H_decimal_1 = 0;
		H_decimal_2 = 0;
		
		for(uint8_t i = 0; i < 13; i++){
			S_digit_1[i] = 1;
			S_digit_2[i] = 1;
			S_digit_3[i] = 1;
		}
		S_decimal_1 = 1;
		S_decimal_2 = 1;
		
	//------Brightness------//
		for(uint8_t i = 0 ; i < 13 ; i++){
			V_digit_1[i] = 50;
			V_digit_2[i] = 50;
			V_digit_3[i] = 50;
		}
		V_decimal_1 = 50;
		V_decimal_2 = 50;
	}
  
  //-----------------------------------SET PIXELS-----------------------------------------
  
}

void LED_controller::setPixels(){
    for (uint8_t i = 0; i < 13 ; i++){
    pixels.setPixelColor(LED_seq[i], pixels.ColorHSV((int)H_digit_3[i]*65536/360, (int)255*S_digit_3[i], (int)(toggle_digit_3[i]*brightness*blink_var*((float)V_digit_3[i]/100))));
  }

  for (uint8_t i = 0; i < 13 ; i++){
    pixels.setPixelColor(LED_seq[i]+14, pixels.ColorHSV((int)H_digit_2[i]*65536/360, (int)255*S_digit_2[i], (int)(toggle_digit_2[i]*brightness*blink_var*((float)V_digit_2[i]/100))));
  }

  for (uint8_t i = 0; i < 13 ; i++){
    pixels.setPixelColor(LED_seq[i]+14+14, pixels.ColorHSV((int)H_digit_1[i]*65536/360, (int)255*S_digit_1[i], (int)(toggle_digit_1[i]*brightness*blink_var*((float)V_digit_1[i]/100))));
  }

  //Decimals
  pixels.setPixelColor(13, pixels.ColorHSV((int)H_decimal_2*65536/360, (int)255*S_decimal_2, (int)(toggle_decimal_2*brightness*blink_var*((float)V_decimal_2/100))));
  pixels.setPixelColor(27, pixels.ColorHSV((int)H_decimal_1*65536/360, (int)255*S_decimal_1, (int)(toggle_decimal_1*brightness*blink_var*((float)V_decimal_1/100))));
    
  pixels.show();   
}

uint8_t LED_controller::bar(uint8_t prog, uint8_t var){

  if(modes == 0 || modes == 4){
    if (var == 0) return 1;
    if (var == 1) {
		if(modes == 0)
			return 30; //Hue
		if(modes == 4)
			if(input > 20) return 120;
			else return 0;
	}
    if (var == 2) 
      if(input < 100) return ((((int)input/10) % 10) >= prog); //Saturation 
        else return 1;
    if (var == 3) 
      if(input < 100)return (((((int)input/10) % 10) >= prog)*90)+10; //Vibrance (Brightness);
        else return 100;
  }
  else{
    if(var == 0) return 0;
      else return 1;
  }
}

uint8_t LED_controller::compass(uint8_t pointing, uint8_t var){
  if(modes == 2 || modes == 3 || modes == 6){
    if (var == 0) return 1;
    if (var == 1){ //Hue
		if(modes == 2) return 0;    //red for compass
		if(modes == 3) return 120;
		if(modes == 6) return 30;	//orange for wayfinding
	}
    if (var == 2){ //Saturation 
    	if (pointing == 1 && (input <= 22 || input > (360-22)))
    		return 1;
    	else if (pointing == 2 && (input <= 22 + 45 && input > (22)))
    		return 1;
    	else if (pointing == 3 && (input <= 22 + 90 && input > (90 - 22)))
    		return 1;
    	else if (pointing == 4 && (input <= 22 + 90 + 45 && input > (90 +45 - 22)))
    		return 1;
    	else if (pointing == 5 && (input <= 22 + 180 && input > (180 - 22)))
    		return 1;
    	else if (pointing == 6 && (input <= 22 + 180 + 45 && input > (180 + 45 - 22)))
    		return 1;
    	else if (pointing == 7 && (input <= 22 + 180 + 90 && input > (180 + 90 - 22)))
    		return 1;
    	else if (pointing == 8 && (input <= 22 + 180 + 90 + 45 && input > (180 + 90  + 45 - 22)))
    		return 1;
    	else
    		return 0;
    } 
    if (var == 3){ //Vibrance
      if (pointing == 1 && (input <= 22 || input > (360-22)))
        return 100;
      else if (pointing == 2 && (input <= 22 + 45 && input > (22)))
        return 100;
      else if (pointing == 3 && (input <= 22 + 90 && input > (90 - 22)))
        return 100;
      else if (pointing == 4 && (input <= 22 + 90 + 45 && input > (90 +45 - 22)))
        return 100;
      else if (pointing == 5 && (input <= 22 + 180 && input > (180 - 22)))
        return 100;
      else if (pointing == 6 && (input <= 22 + 180 + 45 && input > (180 + 45 - 22)))
        return 100;
      else if (pointing == 7 && (input <= 22 + 180 + 90 && input > (180 + 90 - 22)))
        return 100;
      else if (pointing == 8 && (input <= 22 + 180 + 90 + 45 && input > (180 + 90  + 45 - 22)))
        return 100;
      else
        return 10;
    } 
  }
  else{
    if(var == 0) return 0;
      else return 1;
  }
}

void LED_controller::no_fix_text(){
	if(input > 0)
		for (int i = 0; i < 13; i++){
			toggle_digit_1[i] = seg_letters[2][i];
			toggle_digit_2[i] = seg_letters[3][i];
			toggle_digit_3[i] = seg_letters[4][i];
		}
	else 
		for (int i = 0; i < 13; i++){
			toggle_digit_1[i] = seg_letters[0][i];
			toggle_digit_2[i] = seg_letters[1][i];
			toggle_digit_3[i] = 0;
		}
}

void LED_controller::digits_control(){
	//-----decimal control-----//
	if (input >= 100){
		toggle_decimal_1 = 0;
		toggle_decimal_2 = 0;
	}
	else if (input >= 10){
		toggle_decimal_1 = 0;
		toggle_decimal_2 = 1;
	}
	else{
		toggle_decimal_1 = 1;
		toggle_decimal_2 = 0;
	}
	
	uint8_t digit[3] = {0};
 
	if (input >= 100){
		digit[0] = (int)input/100;
		digit[1] = ((int)input % 100 ) / 10;
		digit[2] = (int)input % 10 ;
	}
	else if (input >= 10){
		digit[0] = ((int)input % 100 ) / 10;
		digit[1] = (int)input % 10 ;
		digit[2] = (int)(input*10) % 10 ;
	}
	else{
		digit[0] = (int)input % 10;
		digit[1] = (int)(input*10) % 10 ;
		digit[2] = (int)(input*100) % 10 ;
	}
	
	for (int i = 0; i < 13; i++){
		toggle_digit_1[i] = seg_seq[digit[0]][i];
		toggle_digit_2[i] = seg_seq[digit[1]][i];
		toggle_digit_3[i] = seg_seq[digit[2]][i];
	}
}


