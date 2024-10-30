#include "esp32s2_ina226.h"
#include "esp32s2_socket.h"
#include "esp32s2_algorithm.h"

uint8_t i2c_ina226_addr = ina_bat_addr;

/*
 * @Info:   Select the INA slave to communicate with
 * @Param:  INA slave address
 * @Return: None
 */
void select_i2c_ina226(uint8_t i2c_slave_ina226_addr){
	i2c_ina226_addr = i2c_slave_ina226_addr;
}

/*
 * @Info:   Check if INA are busy
 * @Param:  None
 * @Return: None
 */
esp_err_t ina226_busy(){
    select_i2c_ina226(ina_bat_addr);
    return i2c_master_detect_slave();
}

/*
 * @Info:   Write to INA registers
 * @Param:  Register address to write into, Value to be written into the register 
 * @Return: 0 if INA write fails
 */
int8_t ina226_write_reg(uint8_t reg_addr, uint16_t reg_val){
    uint8_t write_data_buff[3]={reg_addr,((reg_val >> 8) & 0xFF),(reg_val & 0xFF)};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_ina226_addr) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, write_data_buff, 3, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* 
 * @Info:   Reading from INA registers 
 * @Param:  Register address to read from, pointer to a variable to store the value
 * @Return: 0 if the INA read fails
 */
int8_t ina226_read_reg(uint8_t reg_addr, int16_t *reg_val){
    uint8_t mem_count=0;
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_ina226_addr | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGI("I2C_Test","memory location error");
        return 0;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_ina226_addr | READ_BIT, ACK_CHECK_EN);
    for(mem_count = 0; mem_count < 2; mem_count++){
        i2c_master_read_byte(cmd, (uint8_t *)&reg_val[mem_count],ACK_VAL);
	}
    i2c_master_read_byte(cmd, (uint8_t *)&reg_val[mem_count],NACK_VAL);
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
 * @Info:   Initialize the INA register 
 * @Paramn: Battery calibration value, solar calibration value
 * @Return: None
 */
void init_ina226(uint16_t bat_i_cal, uint16_t sol_i_cal){
	vTaskDelay(10);
	select_i2c_ina226(ina_bat_addr);
	ina226_write_reg(INA226_REG_CALIBRATION_ADDR, bat_i_cal);
	vTaskDelay(10);
	ina226_write_reg(INA226_REG_CONFIG_ADDR, (INA226_AVERAGES_128 | INA226_BUS_CONV_TIME_4156US | INA226_SHUNT_CONV_TIME_4156US | INA226_MODE_SHUNT_BUS_CONT));
	vTaskDelay(10);
	select_i2c_ina226(ina_sol_addr);
	ina226_write_reg(INA226_REG_CALIBRATION_ADDR, sol_i_cal);
	vTaskDelay(10);
	ina226_write_reg(INA226_REG_CONFIG_ADDR, (INA226_AVERAGES_128 | INA226_BUS_CONV_TIME_4156US | INA226_SHUNT_CONV_TIME_4156US | INA226_MODE_SHUNT_BUS_CONT));
}
