#ifndef __LMP91000_DRIVER_LAYER__
#define __LMP91000_DRIVER_LAYER__

#include <stdint.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <nrfx.h>

#ifdef __cplusplus
extern "C"{
#endif

#define LMP91000_I2C_ADDRESS  0X48

#define LMP91000_STATUS_REG		0x00    /* Read only status register */
#define LMP91000_LOCK_REG			0x01    /* Protection Register */
#define LMP91000_TIACN_REG		0x10    /* TIA Control Register */
#define LMP91000_REFCN_REG		0x11    /* Reference Control Register*/
#define LMP91000_MODECN_REG		0x12    /* Mode Control Register */

#define LMP91000_READY			0x01
#define LMP91000_NOT_READY	0x00

#define LMP91000_TIA_GAIN_EXT			0x00	/* default */
#define LMP91000_TIA_GAIN_2P75K		0x04
#define LMP91000_TIA_GAIN_3P5K		0x08
#define LMP91000_TIA_GAIN_7K			0x0C
#define LMP91000_TIA_GAIN_14K			0x10
#define LMP91000_TIA_GAIN_35K			0x14
#define LMP91000_TIA_GAIN_120K		0x18
#define LMP91000_TIA_GAIN_350K		0x1C

#define LMP91000_RLOAD_10OHM		0X00
#define LMP91000_RLOAD_33OHM		0X01
#define LMP91000_RLOAD_50OHM		0X02
#define LMP91000_RLOAD_100OHM		0X03 	 /* default */

#define LMP91000_REF_SOURCE_INT		0x00 /* default */
#define LMP91000_REF_SOURCE_EXT		0x80

#define LMP91000_INT_Z_20PCT		0x00
#define LMP91000_INT_Z_50PCT		0x20 	 /* default */
#define LMP91000_INT_Z_67PCT		0x40
#define LMP91000_INT_Z_BYPASS		0x60

#define LMP91000_BIAS_SIGN_NEG		0x00 /* default */
#define LMP91000_BIAS_SIGN_POS		0x10
 
#define LMP91000_BIAS_0PCT		0x00 		 /* default */
#define LMP91000_BIAS_1PCT		0x01
#define LMP91000_BIAS_2PCT		0x02
#define LMP91000_BIAS_4PCT		0x03
#define LMP91000_BIAS_6PCT		0x04
#define LMP91000_BIAS_8PCT		0x05
#define LMP91000_BIAS_10PCT		0x06
#define LMP91000_BIAS_12PCT		0x07
#define LMP91000_BIAS_14PCT		0x08
#define LMP91000_BIAS_16PCT		0x09
#define LMP91000_BIAS_18PCT		0x0A
#define LMP91000_BIAS_20PCT		0x0B
#define LMP91000_BIAS_22PCT		0x0C
#define LMP91000_BIAS_24PCT		0x0D

#define LMP91000_FET_SHORT_DISABLED			0x00 	/* default */
#define LMP91000_FET_SHORT_ENABLED			0x80
#define LMP91000_OP_MODE_DEEP_SLEEP			0x00 	/* default */
#define LMP91000_OP_MODE_GALVANIC				0x01
#define LMP91000_OP_MODE_STANDBY				0x02
#define LMP91000_OP_MODE_AMPEROMETRIC		0x03
#define LMP91000_OP_MODE_TIA_OFF				0x06
#define LMP91000_OP_MODE_TIA_ON					0x07

#define LMP91000_WRITE_LOCK			0x01 /* default */
#define LMP91000_WRITE_UNLOCK		0x00

#define NUM_TIA_BIAS 14

#define GET_TIA_GAIN_ARRAY_VALUE(index) (\
    (index) == 0 ? 2750 : 	\
    (index) == 1 ? 3500 : 	\
    (index) == 2 ? 7000 : 	\
    (index) == 3 ? 14000 : 	\
    (index) == 4 ? 35000 : 	\
    (index) == 5 ? 120000 : \
    (index) == 6 ? 350000 : \
    -1 /* Default or error value */ \
)

#define GET_TIA_BIAS_ARRAY_VALUE(index) (\
    (index) == 0 ? 0 : 	   \
    (index) == 1 ? 0.01 :  \
    (index) == 2 ? 0.02 :  \
    (index) == 3 ? 0.04 :  \
    (index) == 4 ? 0.06 :  \
    (index) == 5 ? 0.08 :  \
    (index) == 6 ? 0.1 :   \
		(index) == 7 ? 0.12 :  \
		(index) == 8 ? 0.14 :  \
		(index) == 9 ? 0.16 :  \
		(index) == 10 ? 0.18 : \
		(index) == 11 ? 0.2 :  \
		(index) == 12 ? 0.22 : \
		(index) == 13 ? 0.24 : \
    -1 /* Default or error value */ \
)

#define GET_TIA_ZERO_ARRAY_VALUE(index) (\
    (index) == 0 ? 0.2 : 	 \
    (index) == 1 ? 0.5 :   \
    (index) == 2 ? 0.67 :  \
    -1 /* Default or error value */ \
)

/**
 * @brief lmp9100 handle structure definition
 */
typedef struct lmp91000_handle_s
{
		void (*lmp91000_MENB_pin_mode)(uint8_t pin);															 /* Initialize enable pins, low and effective */
		void (*lmp91000_MENB_pin_clear)(uint8_t pin);															 /* set enable pin  */
		void (*lmp91000_MENB_pin_set)(uint8_t pin);																 /* clear enable enable */
    uint8_t (*iic_init)(void);                                                 /* point to an iic_init function address */
    uint8_t (*iic_deinit)(void);                                               /* point to an iic_deinit function address */
    uint8_t (*iic_write_cmd)(uint8_t addr, uint8_t *buf, uint16_t len);        /* point to an iic_write_cmd function address */
    uint8_t (*iic_read_cmd)(uint8_t addr, uint8_t *buf, uint16_t len);         /* point to an iic_read_cmd function address */
    void (*delay_ms)(uint32_t ms);                                             /* point to a delay_ms function address */
    void (*debug_print)(const char *const fmt, ...);                           /* point to a debug_print function address */
    uint8_t iic_addr;                                                          /* iic device address */
    uint8_t inited;                                                            /* inited flag */
		uint8_t MENB_pin_number; 																									 /* IO pin for enabling and disabling lmp91000 */
    uint8_t gain;
    uint8_t zero;
}lmp91000_handle_t;
 
uint8_t lmp91000_init(lmp91000_handle_t* handle);
uint8_t lmp91000_deinit(lmp91000_handle_t* handle);
void lmp91000_MENB_pin_init(lmp91000_handle_t* handle); 
void lmp91000_enable(lmp91000_handle_t* handle);
void lmp91000_disable(lmp91000_handle_t* handle);
bool lmp91000_isReady(lmp91000_handle_t* handle);
bool lmp91000_isLocked(lmp91000_handle_t* handle);
void lmp91000_lock(lmp91000_handle_t* handle);
void lmp91000_unlock(lmp91000_handle_t* handle);
void lmp91000_setGain(lmp91000_handle_t* handle, uint8_t user_gain);
double lmp91000_getGain(lmp91000_handle_t* handle);
void lmp91000_setRLoad(lmp91000_handle_t* handle, uint8_t load);
void lmp91000_setRefSource(lmp91000_handle_t* handle, uint8_t source);
void lmp91000_setIntZ(lmp91000_handle_t* handle, uint8_t intZ);
double lmp91000_getIntZ(lmp91000_handle_t* handle);
void lmp91000_setBiasSign(lmp91000_handle_t* handle, uint8_t sign);
void lmp91000_setBias(lmp91000_handle_t* handle, uint8_t bias);
void lmp91000_setBiasWithSign(lmp91000_handle_t* handle, uint8_t bias, int8_t sign);
void lmp91000_setFET(lmp91000_handle_t* handle, uint8_t selection);
void lmp91000_setMode(lmp91000_handle_t* handle, uint8_t mode);

/**
 * @brief     initialize lmp91000_handle_t structure
 * @param[in] HANDLE points to an lmp91000 handle structure
 * @param[in] STRUCTURE is lmp91000_handle_t
 * @note      none
 */
#define DRIVER_LMP91000_LINK_INIT(HANDLE, STRUCTURE)            memset(HANDLE, 0, sizeof(STRUCTURE))

/**
 * @brief     link iic_init function
 * @param[in] HANDLE points to an lmp91000 handle structure
 * @param[in] FUC points to an iic_init function address
 * @note      none
 */
#define DRIVER_LMP91000_LINK_IIC_INIT(HANDLE, FUC)              (HANDLE)->iic_init = FUC

/**
 * @brief     link iic_deinit function
 * @param[in] HANDLE points to an lmp91000 handle structure
 * @param[in] FUC points to an iic_deinit function address
 * @note      none
 */
#define DRIVER_LMP91000_LINK_IIC_DEINIT(HANDLE, FUC)            (HANDLE)->iic_deinit = FUC

/**
 * @brief     link iic_read_cmd function
 * @param[in] HANDLE points to an lmp91000 handle structure
 * @param[in] FUC points to an iic_read_cmd function address
 * @note      none
 */
#define DRIVER_LMP91000_LINK_IIC_READ_COMMAND(HANDLE, FUC)      (HANDLE)->iic_read_cmd = FUC

/**
 * @brief     link iic_write_cmd function
 * @param[in] HANDLE points to an lmp91000 handle structure
 * @param[in] FUC points to an iic_write_cmd function address
 * @note      none
 */
#define DRIVER_LMP91000_LINK_IIC_WRITE_COMMAND(HANDLE, FUC)     (HANDLE)->iic_write_cmd = FUC

/**
 * @brief     link delay_ms function
 * @param[in] HANDLE points to an lmp91000 handle structure
 * @param[in] FUC points to a delay_ms function address
 * @note      none
 */
#define DRIVER_LMP91000_LINK_DELAY_MS(HANDLE, FUC)              (HANDLE)->delay_ms = FUC

/**
 * @brief     link debug_print function
 * @param[in] HANDLE points to an lmp91000 handle structure
 * @param[in] FUC points to a debug_print function address
 * @note      none
 */
#define DRIVER_LMP91000_LINK_DEBUG_PRINT(HANDLE, FUC)           (HANDLE)->debug_print = FUC

/**
 * @brief     link debug_print function
 * @param[in] HANDLE points to an lmp91000 handle structure
 * @param[in] FUC points to a debug_print function address
 * @note      none
 */
#define DRIVER_LMP91000_LINK_MENB_PIN_MODE(HANDLE, FUC)           (HANDLE)->lmp91000_MENB_pin_mode = FUC

/**
 * @brief     link debug_print function
 * @param[in] HANDLE points to an lmp91000 handle structure
 * @param[in] FUC points to a debug_print function address
 * @note      none
 */
#define DRIVER_LMP91000_LINK_MENB_PIN_CLEAR(HANDLE, FUC)           (HANDLE)->lmp91000_MENB_pin_clear = FUC

/**
 * @brief     link debug_print function
 * @param[in] HANDLE points to an lmp91000 handle structure
 * @param[in] FUC points to a debug_print function address
 * @note      none
 */
#define DRIVER_LMP91000_LINK_MENB_PIN_SET(HANDLE, FUC)           (HANDLE)->lmp91000_MENB_pin_set = FUC

#ifdef __cplusplus
}
#endif

#endif /* __LMP91000_DRIVER_LAYER__ */
