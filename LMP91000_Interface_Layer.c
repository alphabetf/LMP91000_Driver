#include "LMP91000_Interface_Layer.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include <stdarg.h>
#include "nrf_gpio.h"

/* lmp9100使能引脚初始化 */
void lmp91000_MENB_pin_mode(uint8_t pin)
{
		nrf_gpio_cfg_output(pin);		
}

/* 拉低使能引脚 */
void lmp91000_MENB_pin_clear(uint8_t pin)
{
		nrf_gpio_pin_clear(pin);
}

/* 拉高使能引脚 */
void lmp91000_MENB_pin_set(uint8_t pin)
{
		nrf_gpio_pin_set(pin);
}

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t lmp91000_interface_iic_init(void)
{
    return hal_i2c_master_init();
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t lmp91000_interface_iic_deinit(void)
{
    return hal_i2c_master_deinit();
}

/**
 * @brief     interface iic bus write command
 * @param[in] addr is the iic device write address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t lmp91000_interface_iic_write_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
	return hal_i2c_master_write(addr, buf, len, true); /* 注意此处最后不要发送停止位 */
}

/**
 * @brief      interface iic bus read command
 * @param[in]  addr is the iic device write address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t lmp91000_interface_iic_read_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
    return hal_i2c_master_read(addr, buf, len);
}

/**
 * @brief     interface delay ms
 * @param[in] ms
 * @note      none
 */
void lmp91000_interface_delay_ms(uint32_t ms)
{
    nrf_delay_ms(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void lmp91000_interface_debug_print(const char *const fmt, ...)
{
    char str[256];
    uint16_t len;
    va_list args;
    
    memset((char *)str, 0, sizeof(char) * 256); 
    va_start(args, fmt);
    vsnprintf((char *)str, 255, (char const *)fmt, args);
    va_end(args);
    
    len = strlen((char *)str);
	
    //(void)uart_write((uint8_t *)str, len);		/* 此处也可以调用串口将DEBUG信息输出到串口 */
		//NRF_LOG_HEXDUMP_INFO(NRF_LOG_PUSH(str), len)
		NRF_LOG_INFO("%s", NRF_LOG_PUSH(str));
}
