#include <avr/io.h>
#include <avr/interrupt.h>

// SER (DS)
// SRCLK (SHCP/clockPin) - tõusval frondil nihutatakse kõik bitid ühe võrra edasi ja nullinda biti sisse salvestatakse seda väärtust, mis oli frondi hetkel DS jala peal
// RCLK (STCP/latchPin) -  Latch salvestab andmed STCP tõusva frondi peale

#define HC595_PORT   PORTB
#define HC595_DDR    DDRB
#define SER_PORT PORTD
#define SER_DDR DDRD

#define HC595_DS_POS PORTD7      //Data pin (DS)
#define HC595_SH_CP_POS PORTB4      //Shift Clock (SH_CP)
#define HC595_ST_CP_POS PORTB5      //Store Clock (ST_CP)

#define DIGIT_DS_POS PORTB6      //Data pin (DS)
#define DIGIT_SH_CP_POS PORTB2      //Shift Clock (SH_CP)
#define DIGIT_ST_CP_POS PORTB3      //Store Clock (ST_CP)

void timer0_init() {
	// prescaler = 8 ja CTC mode
	TCCR0B |= (1 << WGM02)|(1 << CS01);
	// init counter
	TCNT0 = 0;
	// init compare value
	OCR0A = 30;
	// enable compare interrupt
	TIMSK0 |= (1 << OCIE0A);
}

void timer1_init() {
	// prescaler = 64 ja CTC mode
	TCCR1B |= (1 << WGM12)|(1 << CS11)|(1 << CS10);
	// init counter
	TCNT1 = 0;
	// init compare value
	OCR1A = 30;
	// enable compare interrupt
	TIMSK1 |= (1 << OCIE1A);
}

uint16_t gravity = 10000;

void timer3_init() {
	// prescaler = 64 ja CTC mode
	TCCR3B |= (1 << WGM32)|(1 << CS31)|(1 << CS30);
	// init counter
	TCNT3 = 0;
	// init compare value
	OCR3A = gravity;
	// enable compare interrupt
	TIMSK3 |= (1 << OCIE3A);
}

void HC595Init() {
   // Data(DS), Shift clock (SH_CP), Store Clock (ST_CP) outputideks
   HC595_DDR|=((1<<HC595_SH_CP_POS)|(1<<HC595_ST_CP_POS)|(1<<DIGIT_SH_CP_POS)|(1<<DIGIT_ST_CP_POS)|(1<<DIGIT_DS_POS));
   SER_DDR|=(1<<HC595_DS_POS);
}

#define HC595DataHigh() (SER_PORT|=(1<<HC595_DS_POS))
#define HC595DataLow() (SER_PORT&=(~(1<<HC595_DS_POS)))

#define DigitDataHigh() (HC595_PORT|=(1<<DIGIT_DS_POS))
#define DigitDataLow() (HC595_PORT&=(~(1<<DIGIT_DS_POS)))

void HC595Pulse() {
   // Pulse the Shift Clock
   HC595_PORT|=(1<<HC595_SH_CP_POS);//HIGH
   HC595_PORT&=(~(1<<HC595_SH_CP_POS));//LOW
}

void DigitPulse() {
	// Pulse the Shift Clock
	HC595_PORT|=(1<<DIGIT_SH_CP_POS);//HIGH
	HC595_PORT&=(~(1<<DIGIT_SH_CP_POS));//LOW
}

void HC595Latch() {
   // Pulse the Store Clock
   HC595_PORT|=(1<<HC595_ST_CP_POS);//HIGH
   HC595_PORT&=(~(1<<HC595_ST_CP_POS));//LOW
}

void DigitLatch() {
	// Pulse the Store Clock
	HC595_PORT|=(1<<DIGIT_ST_CP_POS);//HIGH
	HC595_PORT&=(~(1<<DIGIT_ST_CP_POS));//LOW
}

void HC595Write(uint8_t *data) {
	for (uint8_t j=0;j<3;j++) {
		uint8_t byte = data[j];
	   for(uint8_t i=0;i<8;i++) {
		  if(byte & 0b10000000) {
			 HC595DataHigh();
		  }
		  else {
			 HC595DataLow();
		  }
		  HC595Pulse();
		  byte = byte << 1;
	   }
	}
   HC595Latch();
}

void DigitWrite(uint8_t *data) {
	for (uint8_t j=0;j<2;j++) {
		uint8_t byte = data[j];
		for(uint8_t i=0;i<8;i++) {
			if(byte & 0b10000000) {
				DigitDataHigh();
			}
			else {
				DigitDataLow();
			}
			DigitPulse();
			byte = byte << 1;
		}
	}
	DigitLatch();
}


uint8_t arv = 0;
uint8_t row = 0;

// liigutab allapoole klotsi
ISR(TIMER3_COMPA_vect) {
	row++;
}

/*ISR(TIMER0_COMPA_vect) {
	arv++;
	if (arv == 16) {
		arv = 0;
	}
}*/

ISR(TIMER0_COMPA_vect) {
	arv++;
	if (arv == 16) {
		arv = 0;
	}
}

ISR(INT0_vect) {
	TCNT3 = 0;
}

int main(void) {
	// 1. bait - U1 - tulp
	// 2. bait - U2 - tulp
	// 3. bait - U3 - rida
	// 1. number - PORTF5
	// 2. number - PORTF6
	// 3. number - PORTF7
	// 4. number - PORTF0
	// 5. number - PORTF1
	// 6. number - PORTF4
	// numbri 1. bait - vasakpoolne
	// numbri 2. bait - parempoolne
	
   HC595Init();
   
   MCUCR|= (1 << JTD);
   MCUCR|= (1 << JTD);
   DDRF = 0xFF;
   PORTF = (1 << PORTF5)|(1 << PORTF0);
   
   DDRD &= (~((0 << PORTD0)|(0 << PORTD1)|(0 << PORTD2)|(0 << PORTD3)));
   PORTD = (1 << PORTD0)|(1 << PORTD1)|(1 << PORTD2)|(1 << PORTD3);
   
    uint8_t field[16][4] = {{0b00000001,0,0,1},
							{0b00000010,0,0,1},
							{0b00000100,0,0,1},
							{0b00001000,0,0,1},
							{0b00010000,0,0,1},
							{0b00100000,0,0,1},
							{0b01000000,0,0,1},
							{0b10000000,0,0,1},
							{0b00000001,0,0,2},
							{0b00000010,0,0,2},
							{0b00000100,0,0,2},
							{0b00001000,0,0,2},
							{0b00010000,0,0,2},
							{0b00100000,0,0,2},
							{0b01000000,0,0,2},
							{0b10000000,0,0,2}};
	uint8_t digits[10]={
		0b00111111,
		0b00000110,
		0b01011011,
		0b01001111,
		0b01100110,
		0b01101101,
		0b01111101,
		0b00000111,
		0b01111111,
		0b01101111
	};
	
	uint8_t bnr = 0;
	uint8_t blocks[7][2] = {{0b00111100,0},{0b00011000,0b00011000},{0b00011100,0b00000100},{0b00011100,0b00010000},{0b00001100,0b00011000},{0b00111000,0b00010000},{0b00011000,0b00001100}};
	uint8_t byte0 = blocks[bnr][0];
	uint8_t byte1 = blocks[bnr][1];
								
	uint8_t number[2] = {digits[0],digits[0]};
	DigitWrite(number);
	uint8_t skoor1 = 0;
	uint8_t check = 0;
	uint8_t level = 0;
   
   timer0_init();
   timer3_init();
	
   EIMSK |= (1 << INT0); // Enable INT0 External Interrupt
   EICRA |= (1 << ISC01);     // Falling edge triggers INT0
   
    while(1) {
		if (level == 0) {
			cli();
			gravity = 30000;
			for (uint8_t i = 0; i < 16; i++) {
				field[i][field[i][3]] = 0;
			}
			while (1) {
				if((~PIND) & (1 << PORTD3)) {
					level = 1;
					sei();
					break;
				}
				else {
					continue;
				}
			}
		}
		// jõuab lõppu
		if (row > 15) {
			row = 0;
			// tekitada järgmine klots
			bnr++;
			if (bnr > 6) {
				bnr = 0;
			}
			byte0 = blocks[bnr][0];
			byte1 = blocks[bnr][1];
		}
		
		// eemaldab klotsi eelmiselt kohalt
		if (row > 0) {
			field[row][field[row][3]] &= ~byte1;
			field[row-1][field[row-1][3]] &= ~(byte0);
		}
		
		// lisab klotsi uuele reale
		field[row][field[row][3]] |= byte0;
		field[row+1][field[row+1][3]] |= byte1;
		
		// paneb klotsi paigale, kui otse allpool on juba klots
		if ((field[row+2][field[row+2][3]] & byte1) != 0) {
			row = 0;
			check = 1;
			bnr++;
			if (bnr > 6) {
				bnr = 0;
			}
			byte0 = blocks[bnr][0];
			byte1 = blocks[bnr][1];
		}
		else if (((byte0 & ~byte1) & field[row+1][field[row+1][3]]) != 0) {
			row = 0;
			check = 1;
			bnr++;
			if (bnr > 6) {
				bnr = 0;
			}
			byte0 = blocks[bnr][0];
			byte1 = blocks[bnr][1];
		}
		
		// kui rida täis, siis eemaldab
		if (check == 1) {
			for (uint8_t i = 0; i < 16; i++) {
				if (field[i][field[i][3]] == 255) {
					field[i][field[i][3]] = 0;
					for (uint8_t j = i; j > 0; j--) {
						field[j][field[j][3]] = field[j-1][field[j-1][3]];
					}
					skoor1++;
					if (skoor1 > 9) {
						skoor1 = 0;
						if (level < 10) {
							level++;
							gravity = gravity - 3000;
						}
					}
				}
			}
			check = 0;
		}
		number[0] = digits[level];
		number[1] = digits[skoor1];
		
		// kontrollib, kas mäng on läbi
		for (uint8_t i = 0; i < 16; i++) {
			if (field[i][field[i][3]] != 0) {
				if (i == 15) {
					level = 0;
					skoor1 = 0;
				}
			}
			else {
				break;
			}
		}
		
		// näitab väljakut
		HC595Write(field[arv]);
		// näitab skoori
		DigitWrite(number);
		
		// liigutab vasakule
		if((~PIND) & (1 << PORTD1)) {
			if (!((byte0 & 0b10000000) | (byte1 & 0b10000000))) {
				// kontrolli vaja, et vasakul ei oleks 
				// nihutan byte'i vasakule
				// ANDin seda inverteeritud byte'iga
				// ANDin seda field[row]iga
				if (((((byte0 << 1) & ~byte0) & field[row][field[row][3]]) & (((byte1 << 1) & ~byte1) & field[row+1][field[row+1][3]])) == 0) {
					field[row][field[row][3]] &= ~byte0;
					field[row+1][field[row+1][3]] &= ~byte1;
					byte0 = byte0 << 1;
					byte1 = byte1 << 1;
					field[row][field[row][3]] |= byte0;
					field[row+1][field[row+1][3]] |= byte1;
					HC595Write(field[arv]);
					//delay koos vilgutamisega
					for(uint16_t i = 400; i != 0; i--) {
						HC595Write(field[arv]);
						for(uint16_t j = 250; j != 0; j--) {
							asm volatile("nop");
						}
					}
				}
			}
		}
		// liigutab paremale
		if((~PIND) & (1 << PORTD2)) {
			if (!((byte0 & 0b00000001) | (byte1 & 0b00000001))) {
				// kontrolli vaja, et paremal ei oleks 
				if (((((byte0 >> 1) & ~byte0) & field[row][field[row][3]]) & (((byte1 >> 1) & ~byte1) & field[row+1][field[row+1][3]])) == 0) {
					field[row][field[row][3]] &= ~byte0;
					field[row+1][field[row+1][3]] &= ~byte1;
					byte0 = byte0 >> 1;
					byte1 = byte1 >> 1;
					field[row][field[row][3]] |= byte0;
					field[row+1][field[row+1][3]] |= byte1;
					HC595Write(field[arv]);
					for(uint16_t i = 400; i != 0; i--) {
						HC595Write(field[arv]);
						for(uint16_t j = 250; j != 0; j--) {
							asm volatile("nop");
						}
					}
				}
			}
		}
		// peaks klotsi pöörama (praegu muudab klotsi lühemaks)
		else if((~PIND) & (1 << PORTD3)) {
			field[row][field[row][3]] &= ~byte0;
			field[row+1][field[row+1][3]] &= ~byte1;
			byte0 = 0b00010000;
			byte1 = 0b00010000;
		}
		// liigutab alla
		if((~PIND) & (1 << PORTD0)) {
			OCR3A = 3000;
		}
		else {
			OCR3A = gravity;
		}
	}
}
