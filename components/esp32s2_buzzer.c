#include "esp32s2_buzzer.h"
#include "esp32s2_buff_operations.h"
#include "string.h"
#include "esp_log.h"
#include <rom/ets_sys.h>

uint8_t note_name[NUM_NOTES] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C' };
uint16_t note_freq[NUM_NOTES] = {NOTE_c, NOTE_d, NOTE_e, NOTE_f, NOTE_g, NOTE_a, NOTE_b, NOTE_C};

uint8_t tone1_notes[] = "c1d1f1d1a1";
#define TONE1_SIZE	sizeof(tone1_notes)
#define TONE1_TEMPO	200

uint8_t tone2_notes[] = "C1C1C1 2C1 1C1 2C1C1C1";
#define TONE2_SIZE	sizeof(tone2_notes)
#define TONE2_TEMPO	130

#define soft_delay_ms(x) vTaskDelay(x/portTICK_PERIOD_MS);

#define _1usecond 90//4
#define _1msecond 8500

void msdelay(int dur){
  volatile int i;
  for(int j=0;i<dur;j++){
    for(i=0;i<_1msecond;i++){
    }  
  }
}

void usdelay(){
  volatile int i;
  for(i=0;i<_1usecond;i++){
  }  
}

void test_tone(){
	toggle_buzzer();
	usdelay();	
}

/** 
 ** Convert to the note to frequency and genrate that frequency through
 ** Buzzer for given time
 ** Arguments :
 **  note - note to play
 **  dur  - duration of note in mSec
 ** Reurns :
 **  None
 **/

void
play_tone(uint8_t note, uint16_t dur){
	uint16_t note_time, freq_counter = 0;
	uint32_t ms_duration;
	note_time = note_to_dur(note);
	if(note_time != 0 ){	
		ms_duration = esp_log_timestamp();
		while((esp_log_timestamp() - ms_duration) < dur){
            toggle_buzzer();
			freq_counter = 0;
			while(freq_counter < note_time){
				freq_counter++;
                usdelay();
            }
		}
	}	
    buzzer_control(0);
}

/** 
 ** Play tone 1
 ** Arguments :
 **  None
 ** Reurns :
 **  None
 **/

void play_tone1(){
	uint16_t duration, i;
	for (i = 0; i < TONE1_SIZE; i += 2){
		duration = (tone1_notes[i+1] - 48) * TONE1_TEMPO;
		if (tone1_notes[i] == ' '){
			soft_delay_ms(duration);
		}
		else{
			play_tone(tone1_notes[i], duration);
		}
		soft_delay_ms(TONE1_TEMPO / 10);              // brief pause between notes
	}
}

/** 
 ** Play tone 2
 ** Arguments :
 **  None
 ** Reurns :
 **  None
 **/
void play_tone2(){
	uint16_t duration, i;
	for (i = 0; i < TONE2_SIZE; i++){
        duration = (tone2_notes[i+1] - 48) * TONE2_TEMPO;  
		if (tone2_notes[i] == ' '){
			soft_delay_ms(duration);
		}
		else{
			play_tone(tone2_notes[i], duration);
		}
		soft_delay_ms(TONE2_TEMPO / 10); 
	}
}

/** 
 ** Generates mutiple beep sounds
 ** Arguments :
 **  beep_count - number of beeps
 **  beep_dur   - beep duration in mS
 ** Reurns :
 **  None
 **/
void play_beep(uint8_t beep_count, uint16_t beep_dur){
	uint8_t beep_cnt;
	for(beep_cnt = 0; beep_cnt < beep_count; beep_cnt++){
		play_tone('C', beep_dur);
		soft_delay_ms(100);
	}	
}

/** 
 ** Generates mutiple beep sounds
 ** Arguments :
 **  beep_count - number of beeps
 **  beep_dur   - beep duration in mS
 ** Reurns :
 **  None
 **/
void beep(uint8_t beep_count, uint16_t beep_dur){
	play_beep(beep_count, beep_dur);
}
/** 
 ** Convert the note to time
 ** Arguments :
 **  n - note to be converted
 ** Reurns :
 **  0 - invalid note
 **  no-zero - time corresponding to note
 **/
uint16_t note_to_dur(uint8_t n){
	uint8_t i;
	for (i = 0; i < NUM_NOTES; i++){
		if (note_name[i] == n){
			return(note_freq[i]);
		}
	}
	return 0;
}

// void 
// test_notes()
// {
// 	uint8_t rx_note, rx_tempo;
// 	while(1)
// 	{
// 		if(dat_buff_available() > 1)
// 		{
// 			rx_note = dat_buff_read();
// 			rx_tempo = dat_buff_read() - 48;

// 			if (rx_note == ' ')          // is this a rest? 
// 			{
// 				soft_delay_ms(((uint16_t)rx_tempo * TONE1_TEMPO));  // then pause for a moment
// 			}
// 			else
// 			{
// 				play_tone(rx_note, ((uint16_t)rx_tempo * TONE1_TEMPO));
// 			}
// 			soft_delay_ms(TONE1_TEMPO / 10); 
// 		}
// 	}

// }