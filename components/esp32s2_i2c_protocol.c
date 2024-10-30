#include "esp32s2_i2c_protocol.h"
 
i2c_port_t i2c_num; 
i2c_slave_addr_t i2c_slave_addr;

void init_i2c(void)
{   
    i2c_num = I2C_MASTER;
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000};
    i2c_param_config(I2C_MASTER, &i2c_config);
    i2c_driver_install(I2C_MASTER, I2C_MODE_MASTER, 0, 0, 0);
}

esp_err_t i2c_master_detect_slave(void)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_slave_addr) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
