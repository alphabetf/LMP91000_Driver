#include "LMP91000_Application_Layer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "adc_battery.h"
#include "app_timer.h"

static lmp91000_handle_t g_lmp91000_handle;        /* lmp91000 handle */
APP_TIMER_DEF(lmp91000_timer_id);							 		 /* lmp91000��ʱ����� */	

/* �����Ƕ�ʱ������ʱҪִ�еĴ��� */
static void lmp91000_timer_handler(void * p_context) 
{
		float lmp91000_res = get_lmp91000_voltage();

		NRF_LOG_INFO("lmp91000 res is " NRF_LOG_FLOAT_MARKER " C", NRF_LOG_FLOAT(lmp91000_res));
}

/* lmp91000��������ʼ�� */
uint8_t lmp91000_sensor_init(void)
{
		uint8_t res;
	
		if(g_lmp91000_handle.inited == 1){	 /* �������Ѿ���ɳ�ʼ�� */
			return 0;
		}
		/* ע��lmp91000�������� */
		DRIVER_LMP91000_LINK_INIT(&g_lmp91000_handle, lmp91000_handle_t);
		DRIVER_LMP91000_LINK_IIC_INIT(&g_lmp91000_handle, lmp91000_interface_iic_init);
		DRIVER_LMP91000_LINK_IIC_DEINIT(&g_lmp91000_handle, lmp91000_interface_iic_deinit);
		DRIVER_LMP91000_LINK_IIC_READ_COMMAND(&g_lmp91000_handle, lmp91000_interface_iic_read_cmd);
		DRIVER_LMP91000_LINK_IIC_WRITE_COMMAND(&g_lmp91000_handle, lmp91000_interface_iic_write_cmd);
		DRIVER_LMP91000_LINK_DELAY_MS(&g_lmp91000_handle, lmp91000_interface_delay_ms);
		DRIVER_LMP91000_LINK_DEBUG_PRINT(&g_lmp91000_handle, lmp91000_interface_debug_print);
		DRIVER_LMP91000_LINK_MENB_PIN_MODE(&g_lmp91000_handle, lmp91000_MENB_pin_mode);
		DRIVER_LMP91000_LINK_MENB_PIN_CLEAR(&g_lmp91000_handle, lmp91000_MENB_pin_clear);
		DRIVER_LMP91000_LINK_MENB_PIN_SET(&g_lmp91000_handle, lmp91000_MENB_pin_set);
    /* ��ʼ��lmp91000 */
    res = lmp91000_init(&g_lmp91000_handle);
    if (res != 0){
        NRF_LOG_INFO("lmp91000: init failed.\n");
        return 1;
    }
		/* �Ƿ���Ҫ��ʼ��ADC */
		saadc_driver_init();
		
//		void lmp91000_enable(lmp91000_handle_t* handle);
//void lmp91000_disable(lmp91000_handle_t* handle);
//bool lmp91000_isReady(lmp91000_handle_t* handle);
//bool lmp91000_isLocked(lmp91000_handle_t* handle);
//void lmp91000_lock(lmp91000_handle_t* handle);
//void lmp91000_unlock(lmp91000_handle_t* handle);
//void lmp91000_setGain(lmp91000_handle_t* handle, uint8_t user_gain);
//double lmp91000_getGain(lmp91000_handle_t* handle);
//void lmp91000_setRLoad(lmp91000_handle_t* handle, uint8_t load);
//void lmp91000_setRefSource(lmp91000_handle_t* handle, uint8_t source);
//void lmp91000_setIntZ(lmp91000_handle_t* handle, uint8_t intZ);
//double lmp91000_getIntZ(lmp91000_handle_t* handle);
//void lmp91000_setBiasSign(lmp91000_handle_t* handle, uint8_t sign);
//void lmp91000_setBias(lmp91000_handle_t* handle, uint8_t bias);
//void lmp91000_setBiasWithSign(lmp91000_handle_t* handle, uint8_t bias, int8_t sign);
//void lmp91000_setFET(lmp91000_handle_t* handle, uint8_t selection);
//void lmp91000_setMode(lmp91000_handle_t* handle, uint8_t mode);

		app_timer_create(&lmp91000_timer_id, APP_TIMER_MODE_REPEATED, lmp91000_timer_handler);
		app_timer_start(lmp91000_timer_id, APP_TIMER_TICKS(1000), NULL);
		NRF_LOG_INFO("lmp91000 init finsh"); 
		
		return 0;
}
