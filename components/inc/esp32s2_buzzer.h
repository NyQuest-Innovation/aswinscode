#ifndef _esp32s2_buzzer_h_
#define _esp32s2_buzzer_h_

#include "esp32s2_main.h"
#include "esp32s2_gpio.h"

// C1C1C1 2C1 1C1 2C1C1C1

// Note  frequency in hertz

#define c_freq	2620
#define d_freq	2940
#define e_freq	3300
#define f_freq	3490
#define g_freq	3920
#define a_freq	4400
#define b_freq	4940
#define C_freq 	5230

#define NUM_NOTES	8


#define FREQ_2_TIME(x)	((1000000 / (x)) / 2)

#define NOTE_c	FREQ_2_TIME(c_freq)
#define NOTE_d	FREQ_2_TIME(d_freq)
#define NOTE_e	FREQ_2_TIME(e_freq)
#define NOTE_f	FREQ_2_TIME(f_freq)
#define NOTE_g	FREQ_2_TIME(g_freq)
#define NOTE_a	FREQ_2_TIME(a_freq)
#define NOTE_b	FREQ_2_TIME(b_freq)
#define NOTE_C	FREQ_2_TIME(C_freq)
#define MUTE	0

#define LONG_BEEP		500			// ms

#warning "SHORT_BEEP changed from 100 to 300"
#define SHORT_BEEP		300			// ms

extern void usdelay();
extern void play_tone(uint8_t note, uint16_t dur);
extern uint16_t note_to_dur(uint8_t n);
extern void  play_tone1();
extern void  play_tone2();
extern void play_beep(uint8_t beep_count, uint16_t beep_dur);
extern void beep(uint8_t beep_count, uint16_t beep_dur);
extern void test_notes();

#endif