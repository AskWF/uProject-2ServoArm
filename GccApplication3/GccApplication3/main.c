#define F_CPU 16000000


//****main.c****//
#include "lcd.h"
#include "avr/io.h"
#include "util/delay.h"
#include "avr/interrupt.h"

//OLED can display 21 characters in each row and 7 rows down.

int cliflag = 0;

static inline void initTimer1Servo(void) {
	/* Set up Timer1 (16bit) to give a pulse every 20ms */
	/* Use Fast PWM mode, counter max in ICR1 */
	DDRB |= (1 << DDB1) | (1 << DDB2); // set pin for output 
	TCCR1A |= (1 << WGM11); //WGM Set to Fast PWM
	TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); //8 prescaling -- counting in microseconds
	ICR1 = 40000; // TOP value = 20ms
	TCCR1A |= (1 << COM1A1); //Direct output on PB1 / OC1A
	TCCR1A |= (1 << COM1B1); //Direct output on PB2 / OC1B
}


static inline void initADC(void) {
	ADCSRA |= (1<<ADEN) | (1<<ADIE); //ADEN togglet til 1 gir oss ADC enabled,  ADIE aktiverer ADC interrupts
	ADCSRA |= (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2); //Setter ADC klokka til 125kHz hvis systemklokka er 16mHz
	ADMUX |=  (1<<ADLAR) |(1<<REFS0);  //Toggle AVCC reference and ADLAR
	DIDR0 |=  (1<<ADC0D)|(1<<ADC1D);	//Digital input disable p� pin A0 og A1 p� arduino.
	startConversion();
}

int startConversion(void){
	ADCSRA |= (1<<ADSC); //Start ADC conversion
	do {} while (ADCSRA & (1<<ADSC)); //Wait until ADC has finished reading
	int adc_result = ADCH;
	return adc_result;
}

void initInterrupt0(void){
	EIMSK = (1<<INT0);
	EICRA = (1<<ISC01);
	PORTD = (1<<PORTD2);
}


int main(void){
	lcd_init(LCD_DISP_ON);    // init lcd and turn on
	initADC();
	initTimer1Servo();
	initInterrupt0();
  
  
	lcd_puts("Initialization");  // put string from RAM to display (TEXTMODE) or buffer (GRAPHICMODE)
	lcd_gotoxy(0,1);          // set cursor to first column at line 3
	lcd_puts("0_0");
  
	OCR1A = 4000;
	_delay_ms(1000);
	lcd_clrscr();
	OCR1A = 2000;
	_delay_ms(1000);

  
  while(1){
	char adc_string[2]; //Buffer for array where ADC value will be stored as string.
	int joystick;
	
	/*Very simple debounce*/
	if(cliflag == 0){
		sei(); //Set external interrupts
	}
	else{
		cli(); //Close
		_delay_ms(1000);
		cliflag = 0;
	}
	   
	joystick = startConversion(); //New ADC reading
	itoa(joystick, adc_string, 10); //Converts ADC to string
	lcd_gotoxy(9,1); //Coordinates to print
	lcd_puts(adc_string); //Prints ADC to selected coordinate
	OCR1A = joystick*13 + 800; //Sends a scaled ADC value to PB1 to create an appropriate waveform for the servo.
    		
	ADMUX |= (1<<MUX0); //Switches from ADC0 to ADC1
	
	joystick = startConversion(); //New ADC reading
	lcd_gotoxy(9,3);
	itoa(joystick, adc_string, 10);
	lcd_puts(adc_string);
	OCR1B = joystick*13 + 800;
    		
	ADMUX &= ~(1<<MUX0); //Switches from ADC1 back to ADC0
  }
  return 0;
  
}

ISR(INT0_vect){
	lcd_clrscr();
	lcd_puts("interrupt!!");
	/*OCR1A = 800;
	OCR1B = 5000;*/
	cliflag = 1;
}