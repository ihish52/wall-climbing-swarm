#include <Adafruit_NeoPixel.h>

#define PIN 13
#define NUMPIXELS 41

#define DELAYVAL 500
#define BRIGHTNESS 255

class LED_controller {
	
	private: 
		Adafruit_NeoPixel pixels;
		//(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800)
		
		uint8_t bar(uint8_t prog, uint8_t var);
		uint8_t bar2(uint8_t prog, uint8_t var);
		uint8_t compass(uint8_t pointing, uint8_t var);
		void no_fix_text();
		void digits_control();
		
		const uint8_t LED_seq[13] = { 10, 9, 8, 
									  11,    7, 
									  12, 6, 0,
									   5,    1, 
									   4, 3, 2};
							   
		const bool seg_seq[10][13] = {{	1, 1, 1, 
										1,    1, 
										1, 0, 1,
										1,    1, 
										1, 1, 1},
										
									  {	0, 0, 1, 
										0,    1, 
										0, 0, 1,
										0,    1, 
										0, 0, 1},
										
									  {	1, 1, 1, 
										0,    1, 
										1, 1, 1,
										1,    0, 
										1, 1, 1},
										
									  {	1, 1, 1, 
										0,    1, 
										1, 1, 1,
										0,    1, 
										1, 1, 1},
										
									  {	1, 0, 1, 
										1,    1, 
										1, 1, 1,
										0,    1, 
										0, 0, 1},	
									   
									  {	1, 1, 1, 
										1,    0, 
										1, 1, 1,
										0,    1, 
										1, 1, 1},
									   
									  {	1, 1, 1, 
										1,    0, 
										1, 1, 1,
										1,    1, 
										1, 1, 1},
									   
									  {	1, 1, 1, 
										0,    1, 
										0, 0, 1,
										0,    1, 
										0, 0, 1},
									   
									  {	1, 1, 1, 
										1,    1, 
										1, 1, 1,
										1,    1, 
										1, 1, 1},
									   
									  {	1, 1, 1, 
										1,    1, 
										1, 1, 1,
										0,    1, 
										1, 1, 1}};
							   

		const bool seg_letters[5][13] = {{	1, 1, 0, 
											1,    1, 
											1, 0, 1,
											1,    1, 
											1, 0, 1},
											
										  {	1, 1, 1, 
											1,    1, 
											1, 0, 1,
											1,    1, 
											1, 1, 1},
											
										  {	0, 1, 1, 
											1,    0, 
											1, 1, 1,
											1,    0, 
											1, 0, 0},
											
										  {	0, 0, 1, 
											0,    0, 
											0, 0, 1,
											0,    1, 
											0, 0, 1},
											
										  {	1, 0, 1, 
											1,    1, 
											0, 1, 0,
											1,    1, 
											1, 0, 1}};

		//------------------------------------//
		bool toggle_digit_1[13] = {0};
		bool toggle_decimal_1 = 0;
		bool toggle_digit_2[13] = {0};
		bool toggle_decimal_2 = 0;
		bool toggle_digit_3[13] = {0};

		//-------------------------------------//
		//Hue in inputs 0-360
		uint16_t H_digit_1[13] = {0};
		uint16_t H_decimal_1 = 0;
		uint16_t H_digit_2[13] = {0};
		uint16_t H_decimal_2 = 0;
		uint16_t H_digit_3[13] = {0};

		//---------------------------------------//
		//Saturation in %
		uint8_t S_digit_1[13] = {1};
		uint8_t S_decimal_1 = 1;
		uint8_t S_digit_2[13] = {1};
		uint8_t S_decimal_2 = 1;
		uint8_t S_digit_3[13] = {1};

		//-------------------------------------//   
		//Vibrance in %
		uint8_t V_digit_1[13] = {100};
		uint8_t V_decimal_1 = 100;
		uint8_t V_digit_2[13] = {100};
		uint8_t V_decimal_2 = 100;
		uint8_t V_digit_3[13] = {100};
	
	public:
		float input = 0;
		uint8_t modes = 0;
		float brightness = BRIGHTNESS;
		float blink_var = 1;
		
		
		void setup();
		void smooth_blink(uint8_t speed, bool on_off);
		void fade_transition(uint8_t speed);
		void fade(bool inout);
		
		void led_function();
		void setPixels();

};
