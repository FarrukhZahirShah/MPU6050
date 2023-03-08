#include <driver/i2c.h>
#include <esp_log.h>

#define MPU6050_ADDRESS 0x68

#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1 0x6B

class MPU6050
{
public:
    MPU6050(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin);

    bool initialize();

    bool read_accelerometer(int16_t *x, int16_t *y, int16_t *z);

    bool read_gyroscope(int16_t *x, int16_t *y, int16_t *z);

    bool write_register(uint8_t reg_address, uint8_t data);

    bool read_register(uint8_t reg_address, uint8_t *data);

    float get_pitch();

    float get_roll();

private:
    i2c_port_t i2c_port_;
    gpio_num_t sda_pin_;
    gpio_num_t scl_pin_;
    uint8_t device_address_;
    int16_t accelerometer_x_offset_;
    int16_t accelerometer_y_offset_;
    int16_t accelerometer_z_offset_;
};

MPU6050::MPU6050(i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin) {
    i2c_port_ = i2c_port;
    device_address_ = MPU6050_ADDRESS;
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(i2c_port_, &i2c_config);
    i2c_driver_install(i2c_port_, I2C_MODE_MASTER, 0, 0, 0);
}

bool MPU6050::initialize()
{
    if (write_register(MPU6050_PWR_MGMT_1, 0x00) == false)
    {
        return false;
    }
    if (write_register(MPU6050_SMPLRT_DIV, 0x07) == false)
    {
        return false;
    }
    if (write_register(MPU6050_CONFIG, 0x00) == false)
    {
        return false;
    }
    if (write_register(MPU6050_GYRO_CONFIG, 0x08) == false)
    {
        return false;
    }
    if (write_register(MPU6050_ACCEL_CONFIG, 0x00) == false)
    {
        return false;
    }

    int16_t x, y, z;
    for (int i = 0; i < 1000; i++)
    {
        read_accelerometer(&x, &y, &z);
        accelerometer_x_offset_ += x;
        accelerometer_y_offset_ += y;
        accelerometer_z_offset_ += z;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    accelerometer_x_offset_ /= 1000;
    accelerometer_y_offset_ /= 1000;
    accelerometer_z_offset_ /= 1000;

    return true;
}

bool MPU6050::read_accelerometer(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buffer[6];
    if (!read_register(MPU6050_ACCEL_XOUT_H, buffer))
    {
        return false;
    }
    *x = (int16_t)(buffer[0] << 8 | buffer[1]) - accelerometer_x_offset_;
    *y = (int16_t)(buffer[2] << 8 | buffer[3]) - accelerometer_y_offset_;
    *z = (int16_t)(buffer[4] << 8 | buffer[5]) - accelerometer_z_offset_;

    return true;
}

bool MPU6050::read_gyroscope(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buffer[6];
    if (!read_register(MPU6050_ACCEL_XOUT_H, buffer))
    {
        return false;
    }
    *x = (int16_t)(buffer[0] << 8 | buffer[1]);
    *y = (int16_t)(buffer[2] << 8 | buffer[3]);
    *z = (int16_t)(buffer[4] << 8 | buffer[5]);
    return true;
}

bool MPU6050::write_register(uint8_t reg_address, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, device_address_ << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_address, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE("MPU6050", "Failed to write register. Error=%d", ret);
        return false;
    }
    return true;
}

bool MPU6050::read_register(uint8_t reg_address, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, device_address_ << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_address, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, device_address_ << 1 | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE("MPU6050", "Failed to read register. Error=%d", ret);
        return false;
    }
    return true;
}

float MPU6050::get_pitch()
{
    int16_t x, y, z;
    read_accelerometer(&x, &y, &z);
    float pitch = atan2(x, sqrt(y * y + z * z)) * 180 / M_PI;
    return pitch;
}

float MPU6050::get_roll()
{
    int16_t x, y, z;
    read_accelerometer(&x, &y, &z);
    float roll = atan2(y, sqrt(x * x + z * z)) * 180 / M_PI;
    return roll;
}
