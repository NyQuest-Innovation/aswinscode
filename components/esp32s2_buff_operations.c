#include "esp32s2_buff_operations.h"
#include "esp32s2_socket.h"

uint32_t uart_timeout_val = STREAM_TIMEOUT_SHORT;

uint32_t time_milli(void){
    time_t current_time;
    time(&current_time);
    return current_time;
}
/** 
 ** set the uart data receive timeout
 ** Arguments :
 **  _uart_time_out - timeout in mSec
 ** Reurns :
 **  None
 **/
void set_uart_timeout(uint32_t _uart_time_out){
	uart_timeout_val = _uart_time_out;
}

/** 
 ** Update uart receive data in a queue
 ** if the queue is full dequeue one element and enqueue the new element
 ** Arguments :
 **  c - byte to update in the queue
 ** Reurns :
 **  None
 **/
void debug_buff_write(uint8_t c){
  uint8_t i; 
  i = (recv_buf_head + 1) % RECV_BUFF_SIZE;
  if (i == recv_buf_tail) // if buff is full increase tail by one
  {
    recv_buf_tail = (recv_buf_tail + 1) % RECV_BUFF_SIZE;
  }
  recv_buf[recv_buf_head] = c;
  recv_buf_head = i;
}

/** 
 ** Get the number of bytes available in the queue
 ** Arguments :
 **  None
 ** Reurns :
 **  The number of bytes in the queue
 **/
uint8_t
debug_buff_available(){
	uint8_t buff_size = (RECV_BUFF_SIZE + recv_buf_head - recv_buf_tail) % RECV_BUFF_SIZE;
	ESP_LOGI("UART","Buffer size %d",buff_size);
	return buff_size;
}

/** 
 ** Dequeue one byte from the queue and update the tail
 ** Arguments :
 **  None
 ** Reurns :
 **  the dequeued element
 **/
uint8_t
debug_buff_read(){
    ESP_LOGI("UART","Inside buffer read");
	uint8_t c;
	c = recv_buf[recv_buf_tail];
	if(recv_buf_head != recv_buf_tail)
	{
	   recv_buf_tail = (recv_buf_tail + 1) % RECV_BUFF_SIZE;
	}
	return c;
}

/** 
 ** wait for data in the uart for timeout in mSec
 ** Dequeue one byte from the queue and update the tail
 ** Arguments :
 **  c - pointer to store the received data
 ** Reurns :
 **  0 - No data received within timeout
 **  1 - Data received
 **/
uint8_t
debug_buff_timed_read(uint8_t *c){
    ESP_LOGI("UART","Inside timed read");
	uint32_t _start_millis,_last_millis;
	_start_millis = time_milli();
    ESP_LOGI("UART","Start time %d",_start_millis);
	do{  
		if(debug_buff_available()){
            ESP_LOGI("UART","Buffer available");
			*c = debug_buff_read();
			return 1;
		}
        _last_millis = time_milli();
        ESP_LOGI("UART","elapsed time %d",_last_millis - _start_millis);
	}while((_last_millis - _start_millis) < 5);
    ESP_LOGI("UART","Time elapsed inside timed read");
	return 0;
}


/** 
 ** Dequeue one byte from the queue and not update the tail
 ** Arguments :
 **  None
 ** Reurns :
 **  the dequeued element
 **/
uint8_t
debug_buff_peek(){
	return(recv_buf[recv_buf_tail]);
}

/** 
 ** wait for data in the uart for timeout in mSec
 ** Dequeue one byte from the queue and not update the tail
 ** Arguments :
 **  c - pointer to store the received data
 ** Reurns :
 **  0 - No data received within timeout
 **  1 - Data received
 **/
uint8_t
debug_buff_timed_peek(uint8_t *c){
	uint32_t _start_millis;
	_start_millis = time_milli();
	do{
		if(debug_buff_available()){
			*c = debug_buff_peek();
			return 1;
		}
	}while((time_milli() - _start_millis ) < uart_timeout_val);
	return 0;
}

/** 
 ** Flush the content of the queue
 ** Arguments :
 **  None
 ** Reurns :
 **  None
 **/
void 
debug_buff_flush()
{
	// don't reverse this or there may be problems if the RX interrupt
	// occurs after reading the value of rx_buffer_head but before writing
	// the value to rx_buffer_tail; the previous value of rx_buffer_head
	// may be written to rx_buffer_tail, making it appear as if the buffer
	// don't reverse this or there may be problems if the RX interrupt
	// occurs after reading the value of rx_buffer_head but before writing
	// the value to rx_buffer_tail; the previous value of rx_buffer_head
	// may be written to rx_buffer_tail, making it appear as if the buffer
	// were full, not empty.

	recv_buf_head = recv_buf_tail;
}

/** 
 ** wait for line of data ends with LF or CR in the uart for timeout in mSec
 ** Dequeue the line from the queue and update the tail
 ** Arguments :
 **  c - pointer to store the received data
 ** Reurns :
 **  0 - No data received within timeout
 **  1 - Data received
 **/
uint8_t
debug_buff_read_line(uint8_t *rx_buff)
{
	uint8_t _index = 0;
	uint8_t c, c1;
	while(_index < ( UART_STREAM_RX_BUFF_SIZE - 1) ){
		c = debug_buff_timed_read(&c1);
		if (c == 0 || c1 == 0x0A || c1 == 0x0D) 
		{
			break;
		}
	    rx_buff[_index++] = c1;
	}
	rx_buff[_index] = '\0';
	return _index;
}


/** 
 ** Search for string present in the queue for a timeout of mSec
 ** Arguments :
 **  search_str - Search string
 ** Reurns :
 **  0 - String is not present
 **  1 - string is present
 **/
uint8_t
debug_port_search(const uint8_t *search_str){
	uint8_t _index = 0, search_str_len;
	uint8_t c;
	search_str_len = strlen((const char *)search_str);
	if( *search_str == 0)
		return 1;   // return true if target is a null string
	while( debug_buff_timed_peek(&c) != 0 ){
		if(c==search_str[_index]){
			debug_buff_timed_read(&c);
			if(++_index >= search_str_len){
                // return true if all chars in the target match
				return 1;
			}
		}
		else{
			if(_index == 0){
				debug_buff_timed_read(&c);
			}
			_index = 0;  // reset index if any char does not match
		}
	}
	return 0;
}
