
/***********************************************************************************************
*	  -								Zukunftstag 2014 (UltraBeamXPE (C))		                   *
*								   ____________________________  			                   *
*																			                   *
*										- Taschenlampe 						                   *
*																			                   *
* 							  Mitwirkende : Hardo Grubitz , Jan Homann						   *
*																							   *
*    Funktionen : Auto Off (ca. 4 min ( +- 1 %.) , Licht dimmen , SOS Funktion (Morse Code)    *
*																							   *
*                                                                                              *
*										Stand : 26.03.2014                                     *
***********************************************************************************************/

/***********************************************************************************************
*																							   *
*	Stromaufname im IDLE Mode       = ca. 770 µA ( 8 MHz ) kein großer Unterschied zu 4 MHz	   *
*	Stromaufname im Power_Pown_Mode = ca. 20 µA  ( 8 MHz ) kein großer Unterschied zu 4 MHz    *
*																							   *
************************************************************************************************/

#define F_CPU 8000000						// Interner RC Oszillator. Frequenz = 8000000 Hz ( 4 ms. Einschwingzeit )

#define ABSCHALTZEIT	240  				// ABSCHALTZEIT der HighPowerLED´s ( Der TIMER0 läuft ca. jede 10ms in den Interrupt ) 24000 = 4 Min
#define HELLIGKEIT		3000				// Wert für den Überlauf ( OCR1A )

#define key_SOS			PD3					// Taste SOS
#define key_ON			PD2					// Taste ON
#define key_UP			PD1					// Taste UP
#define key_DOWN		PD0					// Taste Down

#define TIMER1_START     TCCR1B |=   (1<<CS10);
#define TIMER1_STOP      TCCR1B &= ~ (1<<CS10);

#define XPE_1_AN		PORTB |= (1<<PB0)	// HighPowerLED 1 an
#define XPE_2_AN		PORTB |= (1<<PB1)	// HighPowerLED 2 an
#define XPE_3_AN		PORTB |= (1<<PB2)	// HighPowerLED 3 an
#define XPE_4_AN		PORTB |= (1<<PB3)	// HighPowerLED 4 an

#define XPE_1_AUS		PORTB &= ~(1<<PB0)	// HighPowerLED 1 aus
#define XPE_2_AUS		PORTB &= ~(1<<PB1)	// HighPowerLED 2 aus 
#define XPE_3_AUS		PORTB &= ~(1<<PB2)	// HighPowerLED 3 aus 
#define XPE_4_AUS		PORTB &= ~(1<<PB3)	// HighPowerLED 4 aus

#define INT0_ENABLE		TIMSK  |= ((1<<OCIE1A) | (1<<TOIE1));						// Interrupt "INT0" aktivieren
#define INT0_DISABLE	TIMSK  &= ~((1<<OCIE1A) | (1<<TOIE1));						// Interrupt "INT0" deaktivieren

#define INT1_ENABLE  	GIMSK  |= (1<<INT1);					// Interrupt "INT1" aktivieren
#define INT1_DISABLE 	GIMSK  &= ~(1<<INT1);					// Interrupt "INT1" ktivieren

#define ALARM_AN		PORTB |=  (1<<PB4)	// Summer an
#define ALARM_AUS		PORTB &= ~(1<<PB4)	// Summer aus

#include <avr/io.h>							// Library für die I/O (Ports)
#include <avr/sleep.h>						// Library für den Schlaf Modus
#include <avr/interrupt.h>					// Library für die Interrupts
#include <util/delay.h>						// Library für Delay
#include <stdint.h>							// Benötigt Library für Uint16_t Werte

volatile uint16_t Auto_Off;					// Hier zählt der Timer jede 10 ms die Variable hoch, ( Volatile = Darf durch Interrupt verändert werden & der Compiler Optimiert die Variable nicht weg )
volatile uint8_t keystatus;					// Verhindert das eine Taste mehrmals abgefragt wird, wenn sie fest gehalten wird
volatile uint16_t Ueberlauf;				// Variable gegen den "überlauf / Unterlauf" der Helligkeit
volatile uint16_t Helligkeit_Speicher;
volatile uint8_t Debounce, function_flg, key_state, key_press, key_rpt;


void SOS();									// Main wird bekannt gegeben, dass ein Unterprogramm existiert
uint8_t get_key_rpt( uint8_t key_mask );
uint8_t get_key_press( uint8_t key_mask );


int main(void)
{
	DDRB |= (1<<PB0) | (1<<PB1) | (1<<PB2) | (1<<PB3);
	
    OCR0A   = 77;							// Legt den Wert ZUM Überlaufes fest ( ca. jede 10 ms.)
	TCCR0A |= (1<<WGM01);					// Compare Match (Normal Mode) keine Funktion am PIN &&    TIFR |= 0x01;			// Clear Interrupt Flag ( Wird nur für Polling benötigt! Nicht für Interrupt )
    TIMSK  |= (1<<OCIE0A);					// Compare Match Interrupt enable
    TCCR0B |= ((1<<CS02) | (1<<CS00));		// Prescaler auf 1024 setzen (F_CPU/1024)
	
	OCR1A   = HELLIGKEIT;					// Legt den Wert des Überlaufes fest
	TCCR1A |= 0x00;							// Compare Match (Normal Mode) keine Funktion am PIN
	TIFR   |= 0x01;							// Clear Interrupt Flag ( Wird nur für Polling benötigt! Nicht für Interrupt )
	TIMSK  |= ((1<<OCIE1A) | (1<<TOIE1));							// Compare Match Interrupt enable
	TCCR1B |= (1<<CS10);					// Prescaler auf 0 setzen (F_CPU)
	TIMER1_STOP;
	
	GIMSK  |= ((1<<INT0) | (1<<INT1));		// Interrupt "INT0 & INT1" aktivieren
	
	sei();									// Interrupts global aktivieren
			
	Auto_Off = ABSCHALTZEIT;
	OCR1A = 800;

		
while(1)
{

	
	if ((Auto_Off == ABSCHALTZEIT) || (!(TCCR1B & (1<<CS10))))			// Ist Auto_Off = ABSCHALTZEIT, dann schlafen!
	{	
		
		Auto_Off=0;
		TIMER1_STOP;	
		TCNT1 = 0;			
		XPE_1_AUS; XPE_2_AUS; XPE_3_AUS; XPE_4_AUS;
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sleep_mode();							// Setzt den µC (nach Batteriewechsel) in den sleep_mode
	}
	
    
	}// Ende While
	
}// Ende Main


ISR(TIMER0_COMPA_vect) // Sorgt dafür das die Akkus / Batterien geschont werden! (Abschaltung nach ca. 10 min.) 
{
	
	Auto_Off ++; // Jede 10ms Auto_Off um 1 erhöhen!	

	debounce_key();
	
	static uint8_t delay_cnt;
			
	if ((function_flg & 0x01) == 0x01)
	{
		delay_cnt++;
				
		if (delay_cnt >= 0x01)
		{
			ALARM_AN;
		}
				
		if (delay_cnt >= 0x0B)
		{
			delay_cnt = 0x00;
			ALARM_AUS;
			function_flg &= ~(0x01);
		}
	}
	
	if (PIND & (1<<PIND2))
	{
		INT0_ENABLE;
	}

		Auto_Off= 0x00;
		

		Auto_Off=0;


	

} // Ende ISR (TIMER0_COMPA_vect)


ISR(INT0_vect) // HighPowerLED´s ein / aus schalten! // #define TIMER1_START     TCCR1B |=   (1<<CS10);
{	
	Auto_Off = 0; // Muss auf "0" gesetzt werden, sonst geht das Licht erst nach Tastendruck durch "key_UP" | "key_DOWN" aus		
	INT0_DISABLE;

	TIMER1_START;


	TCNT1= 0;

} // Ende ISR (INT0_vect)


	ISR(INT1_vect) // Wird die Taste "key_SOS" gedrückt, läuft das Unterprogramm ( SOS(); ) ab
	{
		SOS();
	} // Ende ISR (INT1_vect)


	ISR(TIMER1_COMPA_vect) // Ist der OCR1A Wert erreicht, werden die HighPowerLED´s aus geschaltet
	{
		XPE_1_AUS;
		XPE_2_AUS;
		XPE_3_AUS;
		XPE_4_AUS;
	} // Ende ISR (TIMER1_COMPA_vect)


	ISR(TIMER1_OVF_vect) // Schaltet nach jedem Überlauf ( Overflow ) die HighPowerLED´s an
	{	
		XPE_1_AN;
		XPE_2_AN;
		XPE_3_AN;
		XPE_4_AN;	
	} // Ende ISR (TIMER1_OVF_vect)


/***************************************************************************************************
*																								   *
*						Ab hier beginnen die Sonderfunktionen ( Effekte )						   *
*																								   *
***************************************************************************************************/

void SOS() // Hier ist das Unterprogramm "SOS();"
{	
	TIMER1_STOP;
	while(1)
	{	
		
	uint8_t SOS_Counter;
		
	for (SOS_Counter = 0 ; SOS_Counter < 3 ; SOS_Counter++) // 3 x kurz
	{
		ALARM_AN ;
		XPE_1_AN ; XPE_2_AN ; XPE_3_AN ; XPE_4_AN ;
		_delay_ms(150);
		ALARM_AUS ;
		XPE_1_AUS ; XPE_2_AUS ; XPE_3_AUS ; XPE_4_AUS ;
		_delay_ms(150);
			
	if (!(PIND & (1<<key_ON)))
	{

		ALARM_AN;XPE_1_AN;XPE_2_AN;XPE_3_AN;XPE_4_AN;
		_delay_ms(1500);
		ALARM_AUS;XPE_1_AUS;XPE_2_AUS;XPE_3_AUS;XPE_4_AUS;
		_delay_ms(1000);
		return;
	}

		}// Ende for SOS_Counter
		
			_delay_ms(1000);
		
	for (SOS_Counter = 0 ; SOS_Counter < 3 ; SOS_Counter++) // 3 x lang
	{
		ALARM_AN ;
		XPE_1_AN ; XPE_2_AN ; XPE_3_AN ; XPE_4_AN;
		_delay_ms(850);
		ALARM_AUS ;
		XPE_1_AUS ; XPE_2_AUS ; XPE_3_AUS ; XPE_4_AUS;
		_delay_ms(850);
			
	}// Ende for SOS_Counter	
	
}// Ende While (SOS)

}// Ende SOS
	
void debounce_key(void)
{
		static uint8_t ct0, ct1, rpt;
		uint8_t i;
		
		i = key_state ^ ~PIND;					// key changed ?
		ct0 = ~( ct0 & i );						// reset or count ct0
		ct1 = ct0 ^ (ct1 & i);					// reset or count ct1
		i &= ct0 & ct1;							// count until roll over ?
		key_state ^= i;							// then toggle debounced state
		key_press |= key_state & i;				// 0->1: key press detect
		
		if( (key_state & (0b00001111)) == 0 )	// check repeat function
		rpt = 5;								// start delay
		if(--rpt == 0 )
		{
			rpt = 5;                            // repeat delay
			key_rpt |= key_state & (0b00001111);
		}
		
		
		
		if (get_key_rpt((1<<PD0)))
		{
			OCR1A = OCR1A - 6553; // Überlauf um "13107" erhöhen
		}
		
		

		if (get_key_rpt((1<<PD1)))
		{
			OCR1A = OCR1A + 6553; // Überlauf um "13107" erhöhen
		}
		
		
}

int8_t get_key_short( uint8_t key_mask )
{
	cli();                                          // read key state and key press atomic !
	return get_key_press( ~key_state & key_mask );
}

uint8_t get_key_press( uint8_t key_mask )
{
	cli();                                          // read and clear atomic !
	key_mask &= key_press;                          // read key(s)
	key_press ^= key_mask;                          // clear key(s)
	sei();
	return key_mask;
}

uint8_t get_key_rpt( uint8_t key_mask )
{
	cli();                                          // read and clear atomic !
	key_mask &= key_rpt;                            // read key(s)
	key_rpt ^= key_mask;                            // clear key(s)
	sei();
	return key_mask;
}