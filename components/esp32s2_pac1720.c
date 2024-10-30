#include "esp23s2_pac1720.h"
#include "esp32s2_i2c_protocol.h"

esp_err_t pac1720_busy(){
	esp_err_t _pac1720_stat;
	uint8_t num_retry = 0;
	while(1){
		vTaskDelay(1/ portTICK_RATE_MS);
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PAC1720_ADDR) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        _pac1720_stat = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
		num_retry++;
		if((num_retry > 5 ) || (_pac1720_stat == 0)){
			break;
		}
	}
	return (_pac1720_stat);
}

void init_pac1720(){
	pac1720_busy();
	pac1720_write_reg8(PAC1720_CH1_VSENSE_SAMP_CONFIG_ADDR,(PAC1720_VSENSE_SAMP_80MS | PAC1720_VSENSE_AVG_8 | PAC1720_VSENSE_RANGE_40MV));
	pac1720_busy();
	pac1720_write_reg8(PAC1720_CH2_VSENSE_SAMP_CONFIG_ADDR,(PAC1720_VSENSE_SAMP_80MS | PAC1720_VSENSE_AVG_8 | PAC1720_VSENSE_RANGE_40MV));
	pac1720_busy();
	pac1720_write_reg8(PAC1720_V_SOURCE_SAMP_CONFIG_ADDR,(PAC1720_CH2_VSOURCE_SAMP_20MS | PAC1720_CH2_VSOURCE_AVG_8 | PAC1720_CH1_VSOURCE_SAMP_20MS | PAC1720_CH2_VSOURCE_AVG_8));
}

int8_t pac1720_write_reg16(uint8_t reg_addr, uint16_t reg_val){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PAC1720_ADDR) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, (uint8_t *) &reg_addr, 1, ACK_CHECK_EN);
    i2c_master_write(cmd, (uint8_t *) &reg_val, 2, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t _pac1720_stat = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	return (_pac1720_stat);
}

int8_t pac1720_write_reg8(uint8_t reg_addr, uint8_t reg_val){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PAC1720_ADDR) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, (uint8_t *) &reg_addr, 1, ACK_CHECK_EN);
    i2c_master_write(cmd, (uint8_t *) &reg_val, 1, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t _pac1720_stat = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	return (_pac1720_stat);
}

int8_t pac1720_read_reg16(uint8_t reg_addr, uint16_t *_reg_val){
    int _pac1720_stat,mem_count;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, PAC1720_ADDR | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    _pac1720_stat = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (_pac1720_stat != ESP_OK) {
        ESP_LOGI("I2C_Test","memory location error");
        return 0;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_slave_addr | READ_BIT, ACK_CHECK_EN);
    for(mem_count = 0; mem_count < (2 - 1); mem_count++)
	{
        i2c_master_read_byte(cmd, (uint8_t *) &_reg_val[mem_count],ACK_VAL);
	}
    i2c_master_read_byte(cmd, (uint8_t *) &_reg_val[mem_count],NACK_VAL);
    i2c_master_stop(cmd);
    _pac1720_stat = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(_pac1720_stat != ESP_OK){
        ESP_LOGI("I2C_Test","data read error");
        return 0;
    }
	return (_pac1720_stat);
}


int8_t pac1720_read_reg8(uint8_t reg_addr, uint8_t *_reg_val){
    int _pac1720_stat;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, PAC1720_ADDR | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    _pac1720_stat = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (_pac1720_stat != ESP_OK) {
        ESP_LOGI("I2C_Test","memory location error");
        return 0;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, PAC1720_ADDR | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, (uint8_t *) &_reg_val, NACK_VAL);
    i2c_master_stop(cmd);
	_pac1720_stat = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(_pac1720_stat != ESP_OK){
        ESP_LOGI("I2C_Test","data read error");
        return 0;
    }
	return (_pac1720_stat);
 }


int8_t pac720_read_buff(uint16_t reg_addr, uint8_t *reg_buff, uint8_t reg_len){
    int _pac1720_stat,mem_count;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, PAC1720_ADDR | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    _pac1720_stat = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (_pac1720_stat != ESP_OK){
        ESP_LOGI("I2C_Test","memory location error");
        return 1;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, PAC1720_ADDR | READ_BIT, ACK_CHECK_EN);
    for(mem_count = 0; mem_count < (reg_len - 1); mem_count++){
        i2c_master_read_byte(cmd,(uint8_t *) &reg_buff[mem_count],ACK_VAL);
	}
    i2c_master_read_byte(cmd,(uint8_t *) &reg_buff[mem_count],NACK_VAL);
    i2c_master_stop(cmd);
    _pac1720_stat = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(_pac1720_stat != ESP_OK){
        ESP_LOGI("I2C_Test","data read error");
        return 1;
    }
 	return (_pac1720_stat);
}