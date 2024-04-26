#include "LMP91000_Driver_Layer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include <stdbool.h>

/**
 * @brief     initialize the chip
 * @param[in] *handle points to an lmp91000 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic initialization failed
 *            - 2 handle is NULL
 *            - 3 linked functions is NULL
 *            - 4 soft reset failed
 * @note      none
 */
uint8_t lmp91000_init(lmp91000_handle_t* handle)
{
    if (handle == NULL){                                                     /* check handle */
        return 2;                                                            /* return error */
    }
    if (handle->debug_print == NULL){                                        /* check debug_print */    
        return 3;                                                            /* return error */
    }
    if (handle->iic_init == NULL){                                           /* check iic_init */
        handle->debug_print("lmp91000: iic_init is null\n");                 /* iic_init is null */   
        return 3;                                                            /* return error */
    }
    if (handle->iic_deinit == NULL){                                         /* check iic_deinit */ 
        handle->debug_print("lmp91000: iic_deinit is null\n");               /* iic_deinit is null */ 
        return 3;                                                            /* return error */
    }
    if (handle->iic_read_cmd == NULL){                                       /* check iic_read_cmd */
        handle->debug_print("lmp91000: iic_read_cmd is null\n");             /* iic_read_cmd is null */
        return 3;                                                            /* return error */
    }
    if (handle->iic_write_cmd == NULL){                                      /* check iic_write_cmd */
        handle->debug_print("lmp91000: iic_write_cmd is null\n");            /* iic_write_cmd is null */
        return 3;                                                            /* return error */
    }
    if (handle->delay_ms == NULL){                                           /* check delay_ms */
        handle->debug_print("lmp91000: delay_ms is null\n");                 /* delay_ms is null */ 
        return 3;                                                            /* return error */
    }
		if (handle->lmp91000_MENB_pin_mode == NULL){                             /* check lmp91000_MENB_pin_mode */
        handle->debug_print("lmp91000: lmp91000_MENB_pin_mode is null\n");   /* lmp91000_MENB_pin_mode is null */ 
        return 3;                                                            /* return error */
    }
		if (handle->lmp91000_MENB_pin_clear == NULL){                            /* check lmp91000_MENB_pin_clear */
        handle->debug_print("lmp91000: lmp91000_MENB_pin_clear is null\n");  /* lmp91000_MENB_pin_clear is null */ 
        return 3;                                                            /* return error */
    }
		if (handle->lmp91000_MENB_pin_set == NULL){                            	 /* check lmp91000_MENB_pin_set */
        handle->debug_print("lmp91000: lmp91000_MENB_pin_set is null\n");    /* lmp91000_MENB_pin_set is null */ 
        return 3;                                                            /* return error */
    }		
		/* 初始化I2C通信接口 */
    if (handle->iic_init() != 0){                                            /* iic init */
        handle->debug_print("lmp91000: iic init failed\n");                  /* iic init failed */  
        return 1;                                                            /* return error */
    }
		lmp91000_MENB_pin_init(handle);																					 /* initialization enable pin */
		handle->iic_addr = LMP91000_I2C_ADDRESS;																 /* initialization i2c address */
		/* 其他部分初始化 */
		
    handle->inited = 1;                                                      /* flag finish initialization */
    
    return 0;                                                                /* success return 0 */
}

/**
 * @brief     close the chip
 * @param[in] *handle points to an lmp91000 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 iic deinit failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 *            - 4 soft reset failed
 * @note      none
 */
uint8_t lmp91000_deinit(lmp91000_handle_t* handle)
{
    if (handle == NULL){                                              /* check handle */
        return 2;                                                     /* return error */
    }
    if (handle->inited != 1){                                         /* check handle initialization */
        return 3;                                                     /* return error */
    }
    if (handle->iic_deinit() != 0){                                   /* iic deinit */
        handle->debug_print("lmp91000: iic deinit failed\n");         /* iic deinit failed */
        return 1;                                                     /* return error */
    }
		/* 其他deinit操作 */
		
    handle->inited = 0;                                               /* flag close */
    
    return 0;                                                         /* success return 0 */
}

/**
 * @brief      write and read bytes
 * @param[in]  *handle points to an lmp91000 handle structure
 * @param[in]  cmd is the send command
 * @param[in]  delay is the delay in ms
 * @param[out] *data points to a data buffer
 * @param[in]  len is the data length
 * @return     status code
 *             - 0 success
 *             - 1 write read failed
 * @note       none
 */
static uint8_t a_lmp91000_write_read(lmp91000_handle_t* handle, uint8_t cmd, uint16_t delay, uint8_t *data, uint16_t len)
{
    if (handle->iic_write_cmd(handle->iic_addr, &cmd, 1) != 0){            /* write command */
        return 1;                                                          /* return error */
    }
    if (delay != 0){                                                       /* if not 0 */
        handle->delay_ms(delay);                                           /* delay */
    }
    if (len != 0){                                                         /* check length */
        if (handle->iic_read_cmd(handle->iic_addr, data, len) != 0){       /* read data */    
            return 1;                                                      /* return error */
        }
    }
    
    return 0;                                                              /* success return 0 */
}

/* @param    reg: register to write to
   @param		 data: data that will be written to register */
static void lmp91000_write(lmp91000_handle_t* handle, uint8_t reg, uint8_t data)
{
		uint8_t res;
		res = a_lmp91000_write_read(handle,reg,10,NULL,0);
		if (res != 0){                                                   /* check result */
				handle->debug_print("lmp91000: write command failed.\n");    /* write command failed */  
    }
		res = a_lmp91000_write_read(handle,data,10,NULL,0);
		if (res != 0){                                                   /* check result */
				handle->debug_print("lmp91000: write command failed.\n");    /* write command failed */  
    }
}

/* @param  reg: register to read from */
static uint8_t lmp91000_read(lmp91000_handle_t* handle, uint8_t reg)
{
		uint8_t res;
		uint8_t data = 0;
    
		res = a_lmp91000_write_read(handle,reg,10,&data,1);
		if (res != 0){                                                   /* check result */
				handle->debug_print("lmp91000: read command failed.\n");     /* write command failed */                    
    }
	
    return data;
}

/* Initialize enable pins, low and effective */
void lmp91000_MENB_pin_init(lmp91000_handle_t* handle)
{
		handle->lmp91000_MENB_pin_mode(handle->MENB_pin_number);		/* 初始化使能引脚模式 */
}

/* ENABLES the LMP91000 for I2C operations
	 The device is active low */
void lmp91000_enable(lmp91000_handle_t* handle)
{
		handle->lmp91000_MENB_pin_clear(handle->MENB_pin_number);		/* 低使能 */
}

/* DISABLES the LMP91000 for I2C operations
	 The device is active low */
void lmp91000_disable(lmp91000_handle_t* handle)
{
    handle->lmp91000_MENB_pin_set(handle->MENB_pin_number);		/* 高失能 */
}

/* @return  whether or not the device is ready.
	 Reads the status register (0x00) of the LMP91000 to determine whether or not
	 the device is ready to accept I2C commands.
	 Default state is not ready */
bool lmp91000_isReady(lmp91000_handle_t* handle)
{
    return lmp91000_read(handle, LMP91000_STATUS_REG)==LMP91000_READY;
}


/* @return whether or not the TIACN and REFCN is locked for writing
	 Reads the lock register (0x01) of the LMP91000 to determine whether or not
	 the TIACN and REFCN are "write-enabled" or "read-only."
	 Deafult state is "read-only" mode */
bool lmp91000_isLocked(lmp91000_handle_t* handle)
{
    return (lmp91000_read(handle, LMP91000_LOCK_REG)&0x01)==LMP91000_WRITE_LOCK;
}

/* Writes to the lock register (0x01) of the LMP9100 to set the TIACN and REFCN
	 registers to "read-only."
	 Default state is "read-only" mode */
void lmp91000_lock(lmp91000_handle_t* handle)
{
    lmp91000_write(handle, LMP91000_LOCK_REG, LMP91000_WRITE_LOCK);
}

/* Writes to the lock register (0x01) of the LMP9100 to set the TIACN and REFCN
	 registers to "write" mode.
	 Default state is "read-only" mode */
void lmp91000_unlock(lmp91000_handle_t* handle)
{
    lmp91000_write(handle, LMP91000_LOCK_REG, LMP91000_WRITE_UNLOCK);
}

/* @param  gain: the gain to be set to
		param - value - gain resistor
		0 - 000 - External resistor
		1 - 001 - 2.75 kOhm
		2 - 010 - 3.5 kOhm
		3 - 011 - 7 kOhm
		4 - 100 - 14 kOhm
		5 - 101 - 35 kOhm
		6 - 110 - 120 kOhm
		7 - 111 - 350 kOhm
Sets the transimpedance amplifier gain. First reads the register to ensure
that the other bits are not affected. The 3 LSBs of "gain" parameter is
written to the 2nd, 3rd, and 4th bit of the TIACN register */
void lmp91000_setGain(lmp91000_handle_t* handle, uint8_t user_gain)
{
    handle->gain = user_gain;
    
    lmp91000_unlock(handle);
    uint8_t data = lmp91000_read(handle, LMP91000_TIACN_REG);
    data &= ~(7 << 2); 				/* clears bits 2-4 */
    data |= (user_gain << 2); /* writes to bits 2-4 */
    lmp91000_write(handle, LMP91000_TIACN_REG, data);
}

double lmp91000_getGain(lmp91000_handle_t* handle)
{
    if (handle->gain == 0){
			return handle->gain;
		}else{
			return GET_TIA_GAIN_ARRAY_VALUE(handle->gain);
		}
}

/* @param  load: the internal load resistor to select
		param - value - RLoad
		0 - 00 - 10 Ohm
		1 - 01 - 33 Ohm
		2 - 10 - 50 Ohm
		3 - 11 - 100 Ohm
Sets the internal RLOAD selection resistor. First reads the register to ensure
that the other bits are not affected. The 2 LSBs of "load" parameter is
written to the 0th and 1st bit of the TIACN register */
void lmp91000_setRLoad(lmp91000_handle_t* handle, uint8_t load)
{
    lmp91000_unlock(handle);
    uint8_t data = lmp91000_read(handle, LMP91000_TIACN_REG);
    data &= ~3; 	/* clears 0th and 1st bits */
    data |= load; /* writes to 0th and 1st bits */
    lmp91000_write(handle, LMP91000_TIACN_REG, data);
}

/* Unlocks the REFCN register for "write" mode. First reads the register to
ensure that the other bits are not affected. Writes a "0" to the 7th bit of
the REFCN register.
Sets the voltage reference source to supply voltage (Vdd) */
static void lmp91000_setIntRefSource(lmp91000_handle_t* handle)
{
    lmp91000_unlock(handle); /* unlocks the REFCN register for "write" mode */
    uint8_t data = lmp91000_read(handle, LMP91000_REFCN_REG);
    data &= ~(1 << 7); 			 /* clears the 7th bit */
    lmp91000_write(handle, LMP91000_REFCN_REG, data);
}

/* Unlocks the REFCN register for "write" mode. First reads the register to
ensure that the other bits are not affected. Writes a "1" to the 7th bit of
the REFCN register.
Sets the reference source of the LMP91000 to an external reference provided at
the Vref pin */
static void lmp91000_setExtRefSource(lmp91000_handle_t* handle)
{
    lmp91000_unlock(handle); /* unlocks the REFCN register for "write" mode */
    uint8_t data = lmp91000_read(handle, LMP91000_REFCN_REG);
    data |= (1 << 7); 			 /* writes a "1" to the 7th bit */
    lmp91000_write(handle, LMP91000_REFCN_REG, data);
}

/* @param  source: external vs. internal
		param - result
		0 - internal reference
		1 - external reference
Sets the voltage reference source of the LMP91000 to an internal reference or
an external reference */
void lmp91000_setRefSource(lmp91000_handle_t* handle, uint8_t source)
{
    if (source == 0){
			lmp91000_setIntRefSource(handle);
		}else{
			lmp91000_setExtRefSource(handle);
		}
}

/* @param intZ: the internal zero selection
	 param - value - result
		0 - 00 - 20%
		1 - 01 - 50%
		2 - 10 - 67%
		3 - 11 - bypassed
Unlocks the REFCN register for "write" mode. First reads the register to
ensure that the other bits are not affected. Writes to the 5th and 6th bits
of the REFCN register.
Sets the internal zero of the device, particularly the transimpedance
amplifier */
void lmp91000_setIntZ(lmp91000_handle_t* handle, uint8_t intZ)
{
    handle->zero = intZ;
    
    lmp91000_unlock(handle); /* unlocks the REFCN register for "write" mode */
    uint8_t data = lmp91000_read(handle, LMP91000_REFCN_REG);
    data &= ~(3 << 5);
    data |= (intZ << 5);
    lmp91000_write(handle, LMP91000_REFCN_REG, data);
}

double lmp91000_getIntZ(lmp91000_handle_t* handle)
{
    return GET_TIA_ZERO_ARRAY_VALUE(handle->zero);
}

static void lmp91000_setNegBias(lmp91000_handle_t* handle)
{
    lmp91000_unlock(handle);
    uint8_t data = lmp91000_read(handle, LMP91000_REFCN_REG);
    data &= ~(1 << 4); 	/* clear bit */
    lmp91000_write(handle, LMP91000_REFCN_REG, data);
}

static void lmp91000_setPosBias(lmp91000_handle_t* handle)
{
    lmp91000_unlock(handle);
    uint8_t data = lmp91000_read(handle, LMP91000_REFCN_REG);
    data |= (1 << 4);
    lmp91000_write(handle, LMP91000_REFCN_REG, data);
}

/* 0 = negative
	 1 = positive */
void lmp91000_setBiasSign(lmp91000_handle_t* handle, uint8_t sign)
{
    if (sign == 0){
			lmp91000_setNegBias(handle);
		}else{
			lmp91000_setPosBias(handle);
		}
}

void lmp91000_setBias(lmp91000_handle_t* handle, uint8_t bias)
{
    lmp91000_unlock(handle);
    uint8_t data = lmp91000_read(handle, LMP91000_REFCN_REG);
    data &= ~(0x0F); 	/* clear the first four bits so I can bit Or in the next step	*/
    data |= bias; 
    lmp91000_write(handle, LMP91000_REFCN_REG, data);
}

/* sign	0 is negative and 1 is positive */
void lmp91000_setBiasWithSign(lmp91000_handle_t* handle, uint8_t bias, int8_t sign)
{
	if(sign > 0){
		sign = 1;
	}else{
		sign = 0;
	}
	sign = (uint8_t)sign;
	if(bias > 13){
		bias = 0;
	}
	lmp91000_unlock(handle);
	uint8_t data = lmp91000_read(handle, LMP91000_REFCN_REG);
	data &= ~(0x1F); /* clear the first five bits so I can bit Or in the next step */
	data |= bias;
	data |= ((sign << 4) | bias);
	lmp91000_write(handle, LMP91000_REFCN_REG, data);
}

static void lmp91000_disableFET(lmp91000_handle_t* handle)
{
    uint8_t data = lmp91000_read(handle, LMP91000_MODECN_REG);
    data &= ~(1 << 7);
    lmp91000_write(handle, LMP91000_MODECN_REG, data);
}

static void lmp91000_enableFET(lmp91000_handle_t* handle)
{
    uint8_t data = lmp91000_read(handle, LMP91000_MODECN_REG);
    data |= (1 << 7);
    lmp91000_write(handle, LMP91000_MODECN_REG, data);
}

void lmp91000_setFET(lmp91000_handle_t* handle, uint8_t selection)
{
    if (selection == 0){
			lmp91000_disableFET(handle);
		}else{
			lmp91000_enableFET(handle);
		}
}

/* Sets the 3 LSBs of the Mode Control Register (0x12) to 0.
Places the LMP91000 in deep sleep state for power conservation. The LMP91000
consumes 0.6 uA of current in deep sleep mode */
static void lmp91000_sleep(lmp91000_handle_t* handle)
{
    uint8_t data = lmp91000_read(handle, LMP91000_MODECN_REG);
    data &= ~(0x07);
    lmp91000_write(handle, LMP91000_MODECN_REG, data);
}

/* Sets the first three bits of the Mode Control Register to 001. This enables
the LMP91000 for 2-electrode potentiometric measurements */
static void lmp91000_setTwoLead(lmp91000_handle_t* handle)
{
    uint8_t data = lmp91000_read(handle, LMP91000_MODECN_REG);
    data &= ~(0x07);
    data |= (0x01);
    lmp91000_write(handle, LMP91000_MODECN_REG, data);
}

/* Sets the 3 LSBs of the Mode Control Register (0x12) to 010.
Places the device in standby() mode which allows quick warm-up in between tests */
static void lmp91000_standby(lmp91000_handle_t* handle)
{
    uint8_t data = lmp91000_read(handle, LMP91000_MODECN_REG);
    data &= ~(0x07);
    data |= (0x02);
    lmp91000_write(handle, LMP91000_MODECN_REG, data);
}

/* Sets the first three bits of the Mode Control Register to 011. This enables
the LMP91000 for 3-electrode potentiometric measurements */
static void lmp91000_setThreeLead(lmp91000_handle_t* handle)
{
    uint8_t data = lmp91000_read(handle, LMP91000_MODECN_REG);
    data &= ~(0x07);
    data |= (0x03);
    lmp91000_write(handle, LMP91000_MODECN_REG, data);
}

static void lmp91000_measureCell(lmp91000_handle_t* handle)
{
    uint8_t data = lmp91000_read(handle, LMP91000_MODECN_REG);
    data &= ~(0x07); 	/* clears the first three bits */
    data |= (0x06);
    lmp91000_write(handle, LMP91000_MODECN_REG, data);
}

static void lmp91000_getTemp(lmp91000_handle_t* handle)
{
    uint8_t data = lmp91000_read(handle, LMP91000_MODECN_REG);
    data |= (0x07);
    lmp91000_write(handle, LMP91000_MODECN_REG, data);
}

void lmp91000_setMode(lmp91000_handle_t* handle, uint8_t mode)
{
    if (mode == 0){
			lmp91000_sleep(handle);
		}else if (mode == 1){
			lmp91000_setTwoLead(handle);
		}else if (mode == 2){
			lmp91000_standby(handle);
		}else if (mode == 3){
			lmp91000_setThreeLead(handle);
		}else if (mode == 4){
			lmp91000_measureCell(handle);
		}else if (mode == 5){
			lmp91000_getTemp(handle);
		}else {}; 	/* some error */
}
