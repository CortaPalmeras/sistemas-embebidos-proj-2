#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#define CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BUF_SIZE (128)      // buffer size
#define TXD_PIN 1           // UART TX pin
#define RXD_PIN 3           // UART RX pin
#define UART_NUM UART_NUM_0 // UART port number
#define BAUD_RATE 115200    // Baud rate
#define M_PI 3.14159265358979323846
#define REDIRECT_LOGS 1

#define I2C_MASTER_SCL_IO GPIO_NUM_22 // GPIO pin
#define I2C_MASTER_SDA_IO GPIO_NUM_21 // GPIO pin
#define I2C_MASTER_FREQ_HZ 10000
#define BME_ESP_SLAVE_ADDR 0x76
#define WRITE_BIT 0x0
#define READ_BIT 0x1
#define ACK_CHECK_EN 0x0
#define EXAMPLE_I2C_ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1
#define NVS_NAMESPACE "storage"

/*
 * Esto se copio y pego del archivo main del proyecto comunicacion serial.
 * Permite configurar la conexion entre el computador y controlador para poder enviar
 */
static int uart1_printf(const char* str, va_list ap) {
    char* buf;
    vasprintf(&buf, str, ap);
    uart_write_bytes(UART_NUM_1, buf, strlen(buf));
    free(buf);
    return 0;
}

// Setup of UART connections 0 and 1, and try to redirect logs to UART1 if asked
static void uart_setup() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Redirect ESP log to UART1
    if (REDIRECT_LOGS) {
        esp_log_set_vprintf(uart1_printf);
    }
}

// Read UART_num for input with timeout of 1 sec
int serial_read(char* buffer, int size) {
    int len = uart_read_bytes(UART_NUM, buffer, size, pdMS_TO_TICKS(1000));
    return len;
}

esp_err_t sensor_init(void) {
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL; // 0
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

esp_err_t bme_i2c_read(i2c_port_t i2c_num, uint8_t* data_addres, uint8_t* data_rd, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_addres, size, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);

    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }

    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t bme_i2c_write(i2c_port_t i2c_num, uint8_t* data_addres, uint8_t* data_wr, size_t size) {
    uint8_t size1 = 1;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_addres, size1, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ------------ BME 688 ------------- //
uint8_t calc_gas_wait(uint16_t dur) {
    // Fuente: BME688 API
    // https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c#L1176
    uint8_t factor = 0;
    uint8_t durval;

    if (dur >= 0xfc0) {
        durval = 0xff; /* Max duration*/
    } else {
        while (dur > 0x3F) {
            dur = dur >> 2;
            factor += 1;
        }

        durval = (uint8_t)(dur + (factor * 64));
    }

    return durval;
}

uint8_t calc_res_heat(uint16_t temp) {
    // Fuente: BME688 API
    // https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c#L1145
    uint8_t heatr_res;
    uint8_t amb_temp = 25;

    uint8_t reg_par_g1 = 0xED;
    uint8_t par_g1;
    bme_i2c_read(I2C_NUM_0, &reg_par_g1, &par_g1, 1);

    uint8_t reg_par_g2_lsb = 0xEB;
    uint8_t par_g2_lsb;
    bme_i2c_read(I2C_NUM_0, &reg_par_g2_lsb, &par_g2_lsb, 1);
    uint8_t reg_par_g2_msb = 0xEC;
    uint8_t par_g2_msb;
    bme_i2c_read(I2C_NUM_0, &reg_par_g2_msb, &par_g2_msb, 1);
    uint16_t par_g2 = (int16_t)(CONCAT_BYTES(par_g2_msb, par_g2_lsb));

    uint8_t reg_par_g3 = 0xEE;
    uint8_t par_g3;
    bme_i2c_read(I2C_NUM_0, &reg_par_g3, &par_g3, 1);

    uint8_t reg_res_heat_range = 0x02;
    uint8_t res_heat_range;
    uint8_t mask_res_heat_range = (0x3 << 4);
    uint8_t tmp_res_heat_range;

    uint8_t reg_res_heat_val = 0x00;
    uint8_t res_heat_val;

    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t heatr_res_x100;

    if (temp > 400) {
        temp = 400;
    }

    bme_i2c_read(I2C_NUM_0, &reg_res_heat_range, &tmp_res_heat_range, 1);
    bme_i2c_read(I2C_NUM_0, &reg_res_heat_val, &res_heat_val, 1);
    res_heat_range = (mask_res_heat_range & tmp_res_heat_range) >> 4;

    var1 = (((int32_t)amb_temp * par_g3) / 1000) * 256;
    var2 = (par_g1 + 784) * (((((par_g2 + 154009) * temp * 5) / 100) + 3276800) / 10);
    var3 = var1 + (var2 / 2);
    var4 = (var3 / (res_heat_range + 4));
    var5 = (131 * res_heat_val) + 65536;
    heatr_res_x100 = (int32_t)(((var4 / var5) - 250) * 34);
    heatr_res = (uint8_t)((heatr_res_x100 + 50) / 100);

    return heatr_res;
}

int bme_get_chipid(void) {
    uint8_t reg_id = 0xd0;
    uint8_t tmp;

    bme_i2c_read(I2C_NUM_0, &reg_id, &tmp, 1);
    printf("Valor de CHIPID: %2X \n\n", tmp);

    if (tmp == 0x61) {
        printf("Chip BME688 reconocido.\n\n");
        return 0;
    } else {
        printf("Chip BME688 no reconocido. \nCHIP ID: %2x\n\n", tmp); // %2X
    }

    return 1;
}

int bme_softreset(void) {
    uint8_t reg_softreset = 0xE0, val_softreset = 0xB6;

    esp_err_t ret = bme_i2c_write(I2C_NUM_0, &reg_softreset, &val_softreset, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf("%s: Error en softreset\n", esp_err_to_name(ret));
        return 1;
    } else {
        printf("\nSoftreset: OK\n\n");
    }
    return 0;
}

void bme_forced_mode(void) {
    /*
    Fuente: Datasheet[19]
    https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=19

    Para configurar el BME688 en forced mode los pasos son:

    1. Set humidity oversampling to 1x     |-| 0b001 to osrs_h<2:0>
    2. Set temperature oversampling to 2x  |-| 0b010 to osrs_t<2:0>
    3. Set pressure oversampling to 16x    |-| 0b101 to osrs_p<2:0>

    4. Set gas duration to 100 ms          |-| 0x59 to gas_wait_0
    5. Set heater step size to 0           |-| 0x00 to res_heat_0
    6. Set number of conversion to 0       |-| 0b0000 to nb_conv<3:0> and enable gas measurements
    7. Set run_gas to 1                    |-| 0b1 to run_gas<5>

    8. Set operation mode                  |-| 0b01  to mode<1:0>

    */

    // Datasheet[33]
    uint8_t ctrl_hum = 0x72;
    uint8_t ctrl_meas = 0x74;
    uint8_t gas_wait_0 = 0x64;
    uint8_t res_heat_0 = 0x5A;
    uint8_t ctrl_gas_1 = 0x71;

    uint8_t mask;
    uint8_t prev;
    // Configuramos el oversampling (Datasheet[36])

    // 1. osrs_h esta en ctrl_hum (LSB) -> seteamos 001 en bits 2:0
    uint8_t osrs_h = 0b001;
    mask = 0b00000111;
    bme_i2c_read(I2C_NUM_0, &ctrl_hum, &prev, 1);
    osrs_h = (prev & ~mask) | osrs_h;

    // 2. osrs_t esta en ctrl_meas MSB -> seteamos 010 en bits 7:5
    uint8_t osrs_t = 0b01000000;
    // 3. osrs_p esta en ctrl_meas LSB -> seteamos 101 en bits 4:2 [Datasheet:37]
    uint8_t osrs_p = 0b00010100;
    uint8_t osrs_t_p = osrs_t | osrs_p;
    // Se recomienda escribir hum, temp y pres en un solo write

    // Configuramos el sensor de gas

    // 4. Seteamos gas_wait_0 a 100ms
    uint8_t gas_duration = calc_gas_wait(100);

    // 5. Seteamos res_heat_0
    uint8_t heater_step = calc_res_heat(300);

    // 6. nb_conv esta en ctrl_gas_1 -> seteamos bits 3:0
    uint8_t nb_conv = 0b00000000;
    // 7. run_gas esta en ctrl_gas_1 -> seteamos bit 5
    uint8_t run_gas = 0b00100000;
    uint8_t gas_conf = nb_conv | run_gas;

    bme_i2c_write(I2C_NUM_0, &gas_wait_0, &gas_duration, 1);
    bme_i2c_write(I2C_NUM_0, &res_heat_0, &heater_step, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_hum, &osrs_h, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_meas, &osrs_t_p, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_gas_1, &gas_conf, 1);

    // Seteamos el modo
    // 8. Seteamos el modo a 01, pasando primero por sleep
    uint8_t mode = 0b00000001;
    uint8_t tmp_pow_mode;
    uint8_t pow_mode = 0;

    esp_err_t ret;
    do {
        ret = bme_i2c_read(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);

        if (ret == ESP_OK) {
            // Se pone en sleep
            pow_mode = (tmp_pow_mode & 0x03);
            if (pow_mode != 0) {
                tmp_pow_mode &= ~0x03;
                ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);
            }
        }
    } while ((pow_mode != 0x0) && (ret == ESP_OK));

    tmp_pow_mode = (tmp_pow_mode & ~0x03) | mode;
    ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

int bme_check_forced_mode(void) {
    uint8_t ctrl_hum = 0x72;
    uint8_t ctrl_meas = 0x74;
    uint8_t gas_wait_0 = 0x64;
    uint8_t res_heat_0 = 0x5A;
    uint8_t ctrl_gas_1 = 0x71;

    uint8_t tmp, tmp2, tmp3, tmp4, tmp5;

    bme_i2c_read(I2C_NUM_0, &ctrl_hum, &tmp, 1);
    bme_i2c_read(I2C_NUM_0, &gas_wait_0, &tmp2, 1);
    bme_i2c_read(I2C_NUM_0, &res_heat_0, &tmp3, 1);
    bme_i2c_read(I2C_NUM_0, &ctrl_gas_1, &tmp4, 1);
    bme_i2c_read(I2C_NUM_0, &ctrl_meas, &tmp5, 1);

    return (tmp == 0b001 && tmp2 == 0x59 && tmp3 == 0x00 && tmp4 == 0b100000 && tmp5 == 0b01010101);
}

void bme_get_mode(void) {
    uint8_t reg_mode = 0x74;
    uint8_t tmp;

    bme_i2c_read(I2C_NUM_0, &reg_mode, &tmp, 1);

    tmp = tmp & 0x3;

    printf("Valor de BME MODE: %2X \n\n", tmp);
}

/**
 * @brief Funcion que calcula la FFT de un arreglo y guarda el resultado inplace
 *
 * @param array Arreglo de elementos sobre los que se quiere calcular la FFT
 * @param size Tamano del arreglo
 * @param array_re Direccion del arreglo donde se guardara la parte real. Debe ser de tamano size
 * @param array_im Direccion del arreglo donde se guardara la parte imaginaria. Debe ser de tamano
 * size
 */
void calcularFFT(float* array, int size, float* array_re, float* array_im) {
    for (int k = 0; k < size; k++) {
        float real = 0;
        float imag = 0;

        for (int n = 0; n < size; n++) {
            float angulo = 2 * M_PI * k * n / size;
            float cos_angulo = cos(angulo);
            float sin_angulo = -sin(angulo);

            real += cos_angulo * array[n];
            imag += sin_angulo * array[n];
        }
        real /= size;
        imag /= size;
        array_re[k] = real;
        array_im[k] = imag;
    }
}

// realiza el calculo de la temperatura y setea t_fine
int16_t bme_temp_celsius(const uint32_t temp_adc, int32_t* t_fine_ptr) {
    // Datasheet[23]
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=23

    // Se obtienen los parametros de calibracion
    uint8_t addr_par_t1_lsb = 0xE9, addr_par_t1_msb = 0xEA;
    uint8_t addr_par_t2_lsb = 0x8A, addr_par_t2_msb = 0x8B;
    uint8_t addr_par_t3_lsb = 0x8C;

    uint16_t par_t1;
    uint16_t par_t2;
    uint16_t par_t3;

    uint8_t par[5];
    bme_i2c_read(I2C_NUM_0, &addr_par_t1_lsb, par, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t1_msb, par + 1, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t2_lsb, par + 2, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t2_msb, par + 3, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t3_lsb, par + 4, 1);

    par_t1 = (par[1] << 8) | par[0];
    par_t2 = (par[3] << 8) | par[2];
    par_t3 = par[4];

    int64_t var1;
    int64_t var2;
    int64_t var3;
    int16_t temp_comp;
    int32_t t_fine;

    var1 = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
    var2 = (var1 * (int32_t)par_t2) >> 11;
    var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)par_t3 << 4)) >> 14;
    t_fine = var2 + var3;
    temp_comp = ((t_fine * 5) + 128) >> 8;

    *t_fine_ptr = t_fine;

    return temp_comp;
}

// prepara las variables para el calculo de la temperatura
int32_t bme_read_temp(int32_t* t_fine_ptr) {
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=23
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=41
    uint8_t forced_temp_addr[] = {0x22, 0x23, 0x24};

    uint8_t tmp;
    int32_t temp_adc = 0;

    bme_i2c_read(I2C_NUM_0, &forced_temp_addr[0], &tmp, 1);
    temp_adc = temp_adc | ((int32_t)tmp << 12);
    bme_i2c_read(I2C_NUM_0, &forced_temp_addr[1], &tmp, 1);
    temp_adc = temp_adc | ((int32_t)tmp << 4);
    bme_i2c_read(I2C_NUM_0, &forced_temp_addr[2], &tmp, 1);
    temp_adc = temp_adc | ((int32_t)tmp >> 4);

    return bme_temp_celsius(temp_adc, t_fine_ptr);
}

// retorna la presion atmosferica en pascal
uint32_t bme_pres_pascal(const uint32_t press_raw, const int32_t t_fine) {
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=24

    uint8_t addr_par_p1_lsb = 0X8E, addr_par_p1_msb = 0x8F;
    uint8_t addr_par_p2_lsb = 0x90, addr_par_p2_msb = 0x91;
    uint8_t addr_par_p3_lsb = 0x92;
    uint8_t addr_par_p4_lsb = 0x94, addr_par_p4_msb = 0x95;
    uint8_t addr_par_p5_lsb = 0x96, addr_par_p5_msb = 0x97;
    uint8_t addr_par_p6_lsb = 0x99;
    uint8_t addr_par_p7_lsb = 0x98;
    uint8_t addr_par_p8_lsb = 0x9C, addr_par_p8_msb = 0x9D;
    uint8_t addr_par_p9_lsb = 0x9E, addr_par_p9_msb = 0x9F;
    uint8_t addr_par_p10_lsb = 0xA0;

    uint16_t par_p1;
    uint16_t par_p2;
    uint16_t par_p3;
    uint16_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    uint16_t par_p7;
    uint16_t par_p8;
    uint16_t par_p9;
    uint16_t par_p10;

    uint8_t par[16];
    bme_i2c_read(I2C_NUM_0, &addr_par_p1_lsb, par, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p1_msb, par + 1, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p2_lsb, par + 2, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p2_msb, par + 3, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p3_lsb, par + 4, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p4_lsb, par + 5, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p4_msb, par + 6, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p5_lsb, par + 7, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p5_msb, par + 8, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p6_lsb, par + 9, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p7_lsb, par + 10, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p8_lsb, par + 11, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p8_msb, par + 12, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p9_lsb, par + 13, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p9_msb, par + 14, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p10_lsb, par + 15, 1);

    par_p1 = (par[1] << 8) | par[0];
    par_p2 = (par[3] << 8) | par[2];
    par_p3 = par[4];
    par_p4 = (par[6] << 8) | par[5];
    par_p5 = (par[8] << 8) | par[7];
    par_p6 = par[9];
    par_p7 = par[10];
    par_p8 = (par[12] << 8) | par[11];
    par_p9 = (par[14] << 8) | par[13];
    par_p10 = par[15];

    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t press_comp;

    var1 = ((int32_t)t_fine >> 1) - 64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)par_p4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)par_p3 << 5)) >> 3) +
           (((int32_t)par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)par_p1) >> 15;
    press_comp = 1048576 - press_raw;
    press_comp = (uint32_t)((press_comp - (var2 >> 12)) * ((uint32_t)3125));
    if (press_comp >= (1 << 30))
        press_comp = ((press_comp / (uint32_t)var1) << 1);
    else
        press_comp = ((press_comp << 1) / (uint32_t)var1);
    var1 = ((int32_t)par_p9 * (int32_t)(((press_comp >> 3) * (press_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(press_comp >> 2) * (int32_t)par_p8) >> 13;
    var3 = ((int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) *
            (int32_t)par_p10) >>
           17;
    press_comp = (int32_t)(press_comp) + ((var1 + var2 + var3 + ((int32_t)par_p7 << 7)) >> 4);

    return (uint32_t)press_comp;
}

// calcula e imprime el la presion
uint32_t bme_read_pres(const int32_t t_fine) {
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=24
    uint8_t tmp;
    uint8_t forced_pres_addr[] = {0x1F, 0x20, 0x21};

    uint32_t pres_adc = 0;

    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=41
    bme_i2c_read(I2C_NUM_0, forced_pres_addr, &tmp, 1);
    pres_adc = tmp << 12;
    bme_i2c_read(I2C_NUM_0, forced_pres_addr + 1, &tmp, 1);
    pres_adc = pres_adc | ((int32_t)tmp << 4);
    bme_i2c_read(I2C_NUM_0, forced_pres_addr + 2, &tmp, 1);
    pres_adc = pres_adc | ((int32_t)tmp >> 4);

    return bme_pres_pascal(pres_adc, t_fine);
}

uint32_t bme_hum_percent(const uint32_t hum_adc, const int32_t temp_comp) {
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=26
    uint8_t addr_par_h1_lsb = 0xE2, addr_par_h1_msb = 0xE3;
    uint8_t addr_par_h2_msb = 0xE1;
    uint8_t addr_par_h3_lsb = 0xE4;
    uint8_t addr_par_h4_lsb = 0xE5;
    uint8_t addr_par_h5_lsb = 0xE6;
    uint8_t addr_par_h6_lsb = 0xE7;
    uint8_t addr_par_h7_lsb = 0xE8;

    uint8_t par[8];
    bme_i2c_read(I2C_NUM_0, &addr_par_h1_lsb, par, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h1_msb, par + 1, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h2_msb, par + 2, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h3_lsb, par + 3, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h4_lsb, par + 4, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h5_lsb, par + 5, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h6_lsb, par + 6, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h7_lsb, par + 7, 1);

    uint16_t par_h1 = (par[0] & 0xF) | (par[1] << 4);
    uint16_t par_h2 = (par[0] >> 4) | (par[2] << 4);
    uint16_t par_h3 = par[3];
    uint16_t par_h4 = par[4];
    uint16_t par_h5 = par[5];
    uint16_t par_h6 = par[6];
    uint16_t par_h7 = par[7];

    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t var6;
    int32_t temp_scaled;
    int32_t hum_comp;

    temp_scaled = (int32_t)temp_comp;
    var1 = (int32_t)hum_adc - (int32_t)((int32_t)par_h1 << 4) - (
                                  ((temp_scaled * (int32_t)par_h3) / ((int32_t)100)) >> 1);
    var2 = ((int32_t)par_h2 *
            (((temp_scaled * (int32_t)par_h4) / ((int32_t)100)) +
             (((temp_scaled * ((temp_scaled * (int32_t)par_h5) / ((int32_t)100))) >> 6) /
              ((int32_t)100)) +
             ((int32_t)(1 << 14)))) >>
           10;
    var3 = var1 * var2;
    var4 = (((int32_t)par_h6 << 7) + ((temp_scaled * (int32_t)par_h7) / ((int32_t)100))) >> 4;
    var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    var6 = (var4 * var5) >> 1;
    hum_comp = (var3 + var6) >> 12;
    hum_comp = (((var3 + var6) >> 10) * ((int32_t)1000)) >> 12;

    return (uint32_t)hum_comp;
}

uint32_t bme_read_hum(const int32_t temp_comp) {
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=26
    uint8_t tmp;
    uint8_t forced_hum_addr[] = {0x25, 0x26};

    uint32_t hum_adc = 0;

    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=41
    bme_i2c_read(I2C_NUM_0, &forced_hum_addr[0], &tmp, 1);
    hum_adc = (int32_t)tmp << 8;
    bme_i2c_read(I2C_NUM_0, &forced_hum_addr[1], &tmp, 1);
    hum_adc = hum_adc | tmp;

    return bme_hum_percent(hum_adc, temp_comp);
}

uint32_t bme_gas_ohm(const uint16_t gas_adc, const uint8_t gas_range) {
    uint32_t calc_gas_res;
    uint32_t gas_res;

    uint32_t var1 = UINT32_C(262144) >> gas_range;
    int32_t var2 = (int32_t)gas_adc - INT32_C(512);
    var2 *= INT32_C(3);
    var2 = INT32_C(4096) + var2;
    calc_gas_res = (UINT32_C(10000) * var1) / (uint32_t)var2;
    gas_res = calc_gas_res * 100;

    return gas_res;
}

uint32_t bme_read_gas(void) {
    uint8_t tmp;
    uint8_t forced_gas_addr[] = {0x2C, 0x2D};

    uint16_t gas_adc;
    uint8_t gas_range;

    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=41
    bme_i2c_read(I2C_NUM_0, forced_gas_addr, &tmp, 1);
    gas_adc = tmp << 2;
    bme_i2c_read(I2C_NUM_0, forced_gas_addr + 1, &tmp, 1);
    gas_adc = gas_adc | (tmp >> 6);
    gas_range = tmp & 0xf;

    return bme_gas_ohm(gas_adc, gas_range);
}

void obtener5Peaks(float* datos, float* resultados, uint32_t sample_size) {
    for (int i = 0; i < sample_size; i++) {
        for (int j = 0; j < 5; j++) {
            if (datos[i] > resultados[j]) {
                for (int k = 4; k > j; k--) {
                    resultados[k] = resultados[k - 1];
                }
                resultados[j] = datos[i];
                break;
            }
        }
    }
}

void bme_read_data(const uint32_t sample_size) {
    float sumatoria_temp = 0;
    float sumatoria_pres = 0;
    float sumatoria_hum = 0;
    float sumatoria_gas = 0;

    float* valores_temp = malloc(sizeof(float) * sample_size);
    float* valores_pres = malloc(sizeof(float) * sample_size);
    float* valores_hum = malloc(sizeof(float) * sample_size);
    float* valores_gas = malloc(sizeof(float) * sample_size);

    float peaks5_temp[5];
    float peaks5_pres[5];
    float peaks5_hum[5];
    float peaks5_gas[5];

    for (int i = 0; i < 5; ++i) {
        bme_forced_mode();
        int32_t t_fine;
        int32_t temp_comp = bme_read_temp(&t_fine);
        bme_read_pres(t_fine);
        bme_read_hum(temp_comp);
        bme_read_gas();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    for (int i = 0; i < sample_size; ++i) {
        int32_t t_fine;

        bme_forced_mode();
        int32_t temp_comp = bme_read_temp(&t_fine);
        uint32_t pres_comp = bme_read_pres(t_fine);
        uint32_t hum_comp = bme_read_hum(temp_comp);
        uint32_t gas_comp = bme_read_gas();

        float normalized_temp = (float)temp_comp / 100;
        float normalized_pres = (float)pres_comp / 1000;
        float normalized_hum = (float)hum_comp / 1000;
        float normalized_gas = (float)gas_comp;

        valores_temp[i] = normalized_temp;
        valores_pres[i] = normalized_pres;
        valores_hum[i] = normalized_hum;
        valores_gas[i] = normalized_gas;

        float data[4];
        data[0] = normalized_temp;
        data[1] = normalized_pres;
        data[2] = normalized_hum;
        data[3] = normalized_gas;
        uart_write_bytes(UART_NUM, (const char*)data, sizeof(float) * 4);

        sumatoria_temp += normalized_temp * normalized_temp;
        sumatoria_pres += normalized_pres * normalized_pres;
        sumatoria_hum += normalized_hum * normalized_hum;
        sumatoria_gas += normalized_gas * normalized_gas;

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Envio de RMS de las 4 variables de medicion
    float data[4];
    data[0] = sqrt(sumatoria_temp / sample_size);
    data[1] = sqrt(sumatoria_pres / sample_size);
    data[2] = sqrt(sumatoria_hum / sample_size);
    data[3] = sqrt(sumatoria_gas / sample_size);
    uart_write_bytes(UART_NUM, (const char*)data, sizeof(float) * 4);

    // Envio de peaks de las 4 variables de medicion
    obtener5Peaks(valores_temp, peaks5_temp, sample_size);
    obtener5Peaks(valores_pres, peaks5_pres, sample_size);
    obtener5Peaks(valores_hum, peaks5_hum, sample_size);
    obtener5Peaks(valores_gas, peaks5_gas, sample_size);

    uart_write_bytes(UART_NUM, (const char*)peaks5_temp, sizeof(float) * 5);
    uart_write_bytes(UART_NUM, (const char*)peaks5_pres, sizeof(float) * 5);
    uart_write_bytes(UART_NUM, (const char*)peaks5_hum, sizeof(float) * 5);
    uart_write_bytes(UART_NUM, (const char*)peaks5_gas, sizeof(float) * 5);

    // envio de las transformadas de fourier
    float* resultado_fft_real = malloc(sizeof(float) * sample_size);
    float* resultado_fft_imag = malloc(sizeof(float) * sample_size);

    calcularFFT(valores_temp, sample_size, resultado_fft_real, resultado_fft_imag);
    uart_write_bytes(UART_NUM, (const char*)resultado_fft_real, sizeof(float) * sample_size);
    uart_write_bytes(UART_NUM, (const char*)resultado_fft_imag, sizeof(float) * sample_size);

    calcularFFT(valores_pres, sample_size, resultado_fft_real, resultado_fft_imag);
    uart_write_bytes(UART_NUM, (const char*)resultado_fft_real, sizeof(float) * sample_size);
    uart_write_bytes(UART_NUM, (const char*)resultado_fft_imag, sizeof(float) * sample_size);

    calcularFFT(valores_hum, sample_size, resultado_fft_real, resultado_fft_imag);
    uart_write_bytes(UART_NUM, (const char*)resultado_fft_real, sizeof(float) * sample_size);
    uart_write_bytes(UART_NUM, (const char*)resultado_fft_imag, sizeof(float) * sample_size);

    calcularFFT(valores_gas, sample_size, resultado_fft_real, resultado_fft_imag);
    uart_write_bytes(UART_NUM, (const char*)resultado_fft_real, sizeof(float) * sample_size);
    uart_write_bytes(UART_NUM, (const char*)resultado_fft_imag, sizeof(float) * sample_size);

    free(valores_temp);
    free(valores_pres);
    free(valores_hum);
    free(valores_gas);

    free(resultado_fft_real);
    free(resultado_fft_imag);
}

void wait_for_plus(void) {
    char handshake = '\0';
    do {
        serial_read(&handshake, 1);
    } while (handshake != '+');
}

int seek_star(void) {
    char handshake = '\0';

    while (1) {
        int rLen = serial_read(&handshake, 1);

        if (rLen == 1) {
            if (handshake == '+') {
                continue;
            } else if (handshake == '*') {
                return 1;
            } else {
                return 0;
            }
        }
    }
}

// funcion que se asegura de que el esp y el computador están sincronizados
void handshake(void) {

    while (1) {
        wait_for_plus();

        if (seek_star()) {
            break;
        } else {
            uart_write_bytes(UART_NUM, "FAIL", 4);
        }
    }

    uart_write_bytes(UART_NUM, "GOOD", 4);
}

// Función para guardar el tamaño de la ventana en NVS
int save_window_size(int size) {
    nvs_handle_t my_handle;
    esp_err_t err;
    int ret = 0;

    // Abre el espacio de nombres en NVS
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("%s: Error al abrir NVS\n", esp_err_to_name(err));
        return ret;
    }

    // Guarda el tamaño de la ventana
    err = nvs_set_i32(my_handle, "window_size", size);
    if (err != ESP_OK) {
        printf("%s: Error al guardar el tamano de la ventana\n", esp_err_to_name(err));
    } else {
        // Confirma el cambio
        err = nvs_commit(my_handle);
        if (err != ESP_OK) {
            printf("%s: Error al confirmar el tamano de la ventana\n", esp_err_to_name(err));
        } else {
            printf("%s: Tamano de ventana guardado\n", esp_err_to_name(err));
            ret = 1;
        }
    }

    // Cierra el espacio de nombres en NVS
    nvs_close(my_handle);
    return ret;
}

void init_nvs() {
    // Inicializar NVS
    esp_err_t err = nvs_flash_init();

    // Si el NVS necesita ser reformateado
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        printf("%s: NVS necesita ser reformateado", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);
}

// Función para leer el tamaño de la ventana desde NVS
int load_window_size(int default_size) {
    nvs_handle_t my_handle;
    int32_t size = default_size;

    // Abre el espacio de nombres en NVS
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);

    if (err != ESP_OK) {
        printf("%s: Error al abrir NVS\n", esp_err_to_name(err));
        return default_size;
    }

    // Lee el tamaño de la ventana
    err = nvs_get_i32(my_handle, "window_size", &size);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("%s: No se encontró el tamano de la de la muestra en el NVS\n", esp_err_to_name(err));
    } else if (err != ESP_OK) {
        printf("%s: Error al leer el tamano de la muestra\n", esp_err_to_name(err));
    }

    // Cierra el espacio de nombres en NVS
    nvs_close(my_handle);
    return size;
}

#define GET_DATA 'g'
#define CHANGE_SAMPLE_SIZE 'c'
#define GET_SAMPLE_SIZE 's'
#define QUIT 'q'

void app_main(void) {
    uart_setup(); // Uart setup
    handshake();  // sincronizar con python

    ESP_ERROR_CHECK(sensor_init());
    bme_get_chipid();
    bme_softreset();
    bme_get_mode();
    bme_forced_mode();

    init_nvs();
    uint32_t sample_size = load_window_size(50);

    while (1) {
        char command;

        // esto no quema el bme, la funcion serial read tiene un timeout
        while (serial_read((char*)&command, 1) == 0)
            continue;

        if (command == GET_DATA) {
            bme_read_data(sample_size);

        } else if (command == CHANGE_SAMPLE_SIZE) {
            uint32_t new_sample_size;
            serial_read((char*)&new_sample_size, sizeof(new_sample_size));

            if (save_window_size(new_sample_size))
                sample_size = new_sample_size;

        } else if (command == GET_SAMPLE_SIZE) {
            uart_write_bytes(UART_NUM, (char*)&sample_size, sizeof(uint32_t));

        } else if (command == QUIT) {
            esp_restart();

        } else {
            printf("FAIL");
        }
    }
}
