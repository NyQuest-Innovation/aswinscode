#include "esp32s2_24lc512.h"
#include "esp32s2_i2c_protocol.h"

#define TAG "inside 24lc512.c"

/*
 * @Info:   Select the I2C memory to communicate with
 * @Param:  I2C memory address
 * @Return: None
 */
void select_i2c_memory(uint8_t i2c_memory_addr){
	i2c_slave_addr = i2c_memory_addr;
}

/*
 * @Info:   Check EEPROM communication busy 
 * @Param:  None
 * @Return: None
 */

esp_err_t ee24lc256_busy(void){
    select_i2c_memory(eeprom_addr);
    return i2c_master_detect_slave();
}

/*
 * @Info:   Read a byte from EEPROM
 * @Param:  Memory location to read
 * @Return: 0 if I2C communication fails
 */
uint8_t ee24lc256_read_byte(uint16_t memory_location){
    select_i2c_memory(eeprom_addr);
    uint8_t data_rd=0;
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_slave_addr | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, memory_location>>8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, memory_location, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGI("I2C_Test","memory location error");
        return 0;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_slave_addr | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_rd, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK){
        ESP_LOGI("I2C_Test","data read error");
        return 0;
    }
    return data_rd;
}

/*
 * @Info:   Write bytes into EEPROM
 * @Param:  Memory location to write into and data to be written into
 * @Return: 0 if I2C communication fails
 */

esp_err_t ee24lc256_write_byte(uint16_t mem_addr, uint8_t mem_data){
    ESP_LOGI(TAG,"Reached EEPROM byte write function");
    select_i2c_memory(eeprom_addr);
    uint8_t write_data_buff[3]={mem_addr>>8,mem_addr,mem_data};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_slave_addr) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, write_data_buff, 3, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/*
 * @Info:   Write series of data into EEPROM
 * @Param:  Memory address, memory buffer of data and memory length
 * @Return: 0 if INA communication fails
 */
esp_err_t ee24lc256_write_page(uint16_t mem_addr, uint8_t *mem_buff, uint8_t mem_len){
    select_i2c_memory(eeprom_addr);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_slave_addr) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, mem_addr>>8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, mem_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, mem_buff, mem_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/*
 * @Info:   Read a array of data from the EEPROM
 * @Param:  Memory address to read from, buffer to store the data, data length to be read
 * @Return: 0 if INA communication fails
 */

int8_t ee24lc256_read_buff(uint16_t mem_addr, uint8_t *mem_buff, uint8_t mem_len){
    ESP_LOGI(TAG,"Read EEPROM Buffer");
    select_i2c_memory(eeprom_addr);
    uint8_t mem_count=0;
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_slave_addr | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, mem_addr>>8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, mem_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGI("I2C_Test","memory location error");
        return 0;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_slave_addr | READ_BIT, ACK_CHECK_EN);
    for(mem_count = 0; mem_count < (mem_len - 1); mem_count++){
        i2c_master_read_byte(cmd, &mem_buff[mem_count],ACK_VAL);
	}

    i2c_master_read_byte(cmd, &mem_buff[mem_count],NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK){
        ESP_LOGI("I2C_Test","data read error");
        return 0;
    }
    return ret;
}

/*
 * @Info:   Write a array of data into the EEPROM
 * @Param:  Memory address to write into, buffer of data to be written, data length to be written
 * @Return: 0 if INA communication fails
 */
esp_err_t ee24lc256_write_buff(uint16_t mem_addr, uint8_t *mem_buff, uint8_t mem_len){
    ESP_LOGI(TAG,"Reached EEPROM buffer write function");
    esp_err_t status = ESP_FAIL;
	uint16_t first_page_space, first_page_size, num_writes, write_size, pages, byte_written;
	first_page_space = MEM_PAGE_LEN - (mem_addr % MEM_PAGE_LEN);
	if(first_page_space > mem_len){
		first_page_size = mem_len;
		num_writes = 1;
	}
	else{
		first_page_size = first_page_space;
		num_writes = ((mem_len - first_page_size) / MEM_PAGE_LEN) + 2;
	}
	byte_written = 0;
	for(pages = 0; pages < num_writes; pages++){
		if(pages == 0){
			write_size = first_page_size;
		}
		else{
			write_size = (mem_len - byte_written) % MEM_PAGE_LEN;
		}
        ESP_LOGI(TAG,"Writing to address location %d",(mem_addr + byte_written));
        ESP_LOGI(TAG,"Number of bytes to be written %d",write_size);
        vTaskDelay(100/portTICK_PERIOD_MS);
		status = ee24lc256_write_page((mem_addr + byte_written), &mem_buff[byte_written], write_size);
        if(status==ESP_OK){
            byte_written += write_size; 
        }
        else{
            ESP_LOGI(TAG,"Error occurred while writting");
            pages--;
        }
	}
	return status;
}

/*
 * @Info:   Write 16bit data into the EEPROM
 * @Param:  Memory address to write into, data to be written
 * @Return: None
 */
void ee24lc256_write_int16(uint16_t mem_addr, uint16_t int16_dat){
	uint8_t *dat_ptr;
	dat_ptr = (uint8_t *)&int16_dat;
	ee24lc256_write_buff(mem_addr, dat_ptr, 2);
}

/*
 * @Info:   Read 16bit data from the EEPROM
 * @Param:  Memory address to read from
 * @Return: 16bit value
 */
uint16_t ee24lc256_read_int16(uint16_t mem_addr){
	uint16_t int16_dat;
	uint8_t *dat_ptr;
	dat_ptr = (uint8_t *)&int16_dat;
	ee24lc256_read_buff(mem_addr, dat_ptr, 2);
	return (int16_dat);
}

/*
 * @Info:   Write 16bit data into the EEPROM
 * @Param:  Memory address to write into, data to be written
 * @Return: None
 */
void ee24lc256_write_int32(uint16_t mem_addr, uint32_t int32_dat){
	uint8_t *dat_ptr;

	dat_ptr = (uint8_t *)&int32_dat;
	ee24lc256_write_buff(mem_addr, dat_ptr, 4);
}

/*
 * @Info:   Read 32bit data from the EEPROM
 * @Param:  Memory address to read from
 * @Return: 32bit value
 */
uint32_t ee24lc256_read_int32(uint16_t mem_addr){
	uint32_t int32_dat;
	uint8_t *dat_ptr;
	dat_ptr = (uint8_t *)&int32_dat;
	ee24lc256_read_buff(mem_addr, dat_ptr, 4);
	return (int32_dat);
}

/*
 * @Info:   Write float data into the EEPROM
 * @Param:  Memory address to write into, data to be written
 * @Return: None
 */
void ee24lc256_write_float(uint16_t mem_addr, float float_dat){
	uint8_t *dat_ptr;
	dat_ptr = (uint8_t *)&float_dat;
	ee24lc256_write_buff(mem_addr, dat_ptr, 4);
}

/*
 * @Info:   Read flaot data from the EEPROM
 * @Param:  Memory address to read from
 * @Return: flaot value
 */
float ee24lc256_read_float(uint16_t mem_addr){
	float float_dat;
	uint8_t *dat_ptr;
	dat_ptr = (uint8_t *)&float_dat;
	ee24lc256_read_buff(mem_addr, dat_ptr, 4);
	return (float_dat);
}

/*
 * @Info:  Check if memory is available
 * @Param: None
 * @Return: 1 if memory is available; 0 if memory is not available
 */
uint8_t check_memory(){
	vTaskDelay(100/ portTICK_RATE_MS);
	if(ee24lc256_busy() != ESP_OK){
	    return 0;
	}
	else{
	    return 1;
	}
}
#if 0
    /*Busy check*/
    esp_err_t a = ee24lc256_busy();
    if(a==ESP_OK){
      ESP_LOGI("I2C_Test","Memory detected");
    }
    else{
      ESP_LOGI("I2C_Test","Error %d\n",a);
    }

    /* Buffer read/write function */
    uint8_t i2c_write_buffer[5]={0x11,0x22,0x33,0x44,0x55};
    uint8_t i2c_read_buffer[5]={0,0,0,0,0};
    ee24lc256_write_page(0x0000,i2c_write_buffer,5);
    vTaskDelay(10/ portTICK_RATE_MS);
    ee24lc256_read_buff(0x0000,i2c_read_buffer,5);
    for(int i=0;i<5;i++)
    {
      ESP_LOGI("I2C_Test","Read value %x\n",i2c_read_buffer[i]);
    }

    /* Read and write byte test function */
    if(ee24lc256_write_byte(0x0000,0xAD)==ESP_OK){
      ESP_LOGI("I2C_Test","Write Successful");
      ESP_LOGI("I2C_Test","Read value %d\n",ee24lc256_read_byte(0x0000));
    }
    else{
      ESP_LOGI("I2C_Test","Write Error");
    }

    /* Float function test code */
    ee24lc256_write_float(0x0000,14.4);
    vTaskDelay(10 / portTICK_RATE_MS);
    ESP_LOGI("I2C_Test","Float value %f",ee24lc256_read_float(0x0000));
#endif