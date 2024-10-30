#include "esp32s2_data_log.h"
#include "esp32s2_algorithm.h"

#if ((LOG_END_ADDR > 60000))
#error "Memory overflow"
#endif
logging_struct_t log_struct;
uint8_t  flush_idx = 0;

#define TAG "inside data log"

extern uint8_t i2cFlag;

/*
 * @Info:   Flush the data in the ring buffer
 * @Param:  None
 * @Return: None
 */
void turtle_log_flush(){
	log_struct.log_head = LOG_START_ADDR;
	log_struct.log_tail = log_struct.log_head;
	log_struct.log_available = 0;
}

/*
 * @Info: 	Restore the tail of EEPROM after reset
 * @Param:	None
 * @Return: None
 */
void 
turtle_log_idx_restore(){
    if (i2c_eeprom_read_int32(VALID_IDX_ADDR) == VALID_IDX_VALUE) {
        i2c_eeprom_read_buff(EE_LOG_IDX_START_ADDR, (uint8_t *) & log_struct, sizeof (log_struct));
        flush_idx = 1;
        if (((log_struct.log_head % SUMMARY_LEN) != 0) || ((log_struct.log_tail % SUMMARY_LEN) != 0)) {
            log_struct.log_head = LOG_START_ADDR;
            log_struct.log_tail = log_struct.log_head;
            log_struct.log_available = 0;
            flush_idx = 0;
        }
    } 
    else {
        log_struct.log_head = LOG_START_ADDR;
        log_struct.log_tail = log_struct.log_head;
        log_struct.log_available = 0;
        flush_idx = 0;
    }
}

/*
 * @Info:   Save log head and tail before reset
 * @Param:  None
 * @Return: None
 */
void turtle_log_idx_save(){
    i2c_eeprom_write_buff(EE_LOG_IDX_START_ADDR, (uint8_t *) & log_struct, sizeof (log_struct));
    vTaskDelay(5);
    i2c_eeprom_write_int32(VALID_IDX_ADDR, VALID_IDX_VALUE);
}

/*
 * @Info:   Flush saved log head and tail before reset
 * @Param:  None
 * @Return: None
 */
void turtle_log_idx_flush(){
    vTaskDelay(5);
    i2c_eeprom_write_int32(VALID_IDX_ADDR, 0);
    flush_idx = 0;
}

/*
 * @Info:   Write data into eeprom
 * @Param:  Data buffer
 * @Return: None
 */
void turtle_log_put(uint8_t *log_in_buff){
	uint16_t _log_head_offset, _log_tail_offset;
	_log_head_offset = (log_struct.log_head + SUMMARY_LEN) % LOG_END_ADDR;
	if(_log_head_offset == log_struct.log_tail){
		_log_tail_offset = (log_struct.log_tail + SUMMARY_LEN) % LOG_END_ADDR;
		log_struct.log_tail = _log_tail_offset;
	}
	else{
		log_struct.log_available++;
	}
	ESP_LOGI(TAG,"Writing to address %d",log_struct.log_head);
	i2c_eeprom_write_buff(log_struct.log_head, log_in_buff, SUMMARY_LEN); 
	log_struct.log_head = _log_head_offset;
}

/*
 * @Info:   Retrive data from EEPROM
 * @Param:  Data and packet count
 * @Retrun: None
 */
void turtle_log_get(uint8_t *log_out_buff,uint8_t pos){
	uint16_t _log_tail_offset, record_count, record_idx = 0;
	_log_tail_offset = log_struct.log_tail+(pos*SUMMARY_LEN);
	for(record_count = 0; record_count < RECORDS_PER_PACKET; record_count++){
		ESP_LOGI(TAG,"Reading address %d",_log_tail_offset);
		i2c_eeprom_read_buff(_log_tail_offset, &log_out_buff[record_idx], SUMMARY_LEN); 
		_log_tail_offset = (_log_tail_offset + SUMMARY_LEN) % LOG_END_ADDR;
		record_idx += SUMMARY_LEN;
	}
}

/*
 * @@Info:  Update the tail of data which has been sent to server
 * @Param:  None
 * @Return: None
 */
void turtle_log_update_tail(){
	extern uint8_t packet_length;
	uint16_t _log_tail_offset;
	if(log_struct.log_head != log_struct.log_tail){
		_log_tail_offset = (log_struct.log_tail + packet_length*(SUMMARY_LEN * RECORDS_PER_PACKET)) % LOG_END_ADDR;
		log_struct.log_tail = _log_tail_offset;
		if(log_struct.log_available >= packet_length*RECORDS_PER_PACKET){
			log_struct.log_available -= packet_length*RECORDS_PER_PACKET;
		}
	}	
}

/*
 * @Info:   Returns the number of data packets available in the memory
 * @Param:  None
 * @Return: None
 */
uint16_t turtle_log_available(){
	return (log_struct.log_available);
}

/*
 * @Info:   Log the algorithm data to the EEPROM
 * @Param:  None
 * @Return: None 
 */
void log_to_eeprom(){
	check_time();
	ESP_LOGI(TAG,"Inside log to eeprom function\n");
	algo_param.date_time[0] = date_time->tm_mday;   //Set day
	algo_param.date_time[1] = 1+(date_time->tm_mon);    //Set month
	algo_param.date_time[2] = date_time->tm_year;   //set year
	algo_param.date_time[3] = date_time->tm_hour;   //Set Hour
	algo_param.date_time[4] = date_time->tm_min;    //Set minute	
	algo_param.date_time[5] = date_time->tm_sec;
	ESP_LOGI(TAG,"Updated time\n");
	while(i2cFlag){};
	vTaskDelay(10/portTICK_RATE_MS);
	i2cFlag=1;
	turtle_log_put((uint8_t *)&algo_param);
	i2cFlag=0; 
}