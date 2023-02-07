
#ifndef __GLOBAL_VER_H__
#define __GLOBAL_VER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* Public typedef -----------------------------------------------------------*/
typedef enum 
{
    TX_POWER_0_DBM = 0, /*  0 dBm */
    TX_POWER_Neg2_DBM,  /* -2 dBm */  
    TX_POWER_Neg4_DBM,  /* -4 dBm */
    TX_POWER_Neg8_DBM,  /* -8 dBm */
    TX_POWER_Neg15_DBM, /* -12 dBm */
    TX_POWER_Neg20_DBM, /* -20 dBm */ 
    TX_POWER_Pos2_DBM,  /* +2 dBm */
    TX_POWER_Pos3_DBM,  /* +3 dBm */
    TX_POWER_Pos4_DBM,  /* +4 dBm */
    TX_POWER_Pos6_DBM,  /* +6 dBm */    
}rf_tx_power_t;

static const uint8_t TXPWR_MAP[][3]=
{
0, 0x17, 0x13,  /*  0 dBm */
0, 0x17, 0x11,  /* -2 dBm */ 
0, 0x17, 0x0f,  /* -4 dBm */
0, 0x17, 0x0b,  /* -8 dBm */
0, 0x17, 0x06,  /* -12 dBm */
0, 0x17, 0x00,  /* -20 dBm */
0, 0x17, 0x16,  /* +2 dBm */
0, 0x17, 0x19,  /* +3 dBm */    
1, 0x17, 0x16,  /* +4 dBm */
1, 0x17, 0x19,  /* +6 dBm */
};
/* Public define ------------------------------------------------------------*/	


#ifdef MCU_LSC_SEL_LSE
#define G15_ORG              (0)
#define LSC_LSE_32768_HZ     (1) 
#else //#elif defined MCU_LSC_SEL_LSI
#define G15_ORG              (1)
#define LSC_LSI_32000_HZ     (0) 
#endif

/* Public variables ---------------------------------------------------------*/

#define N32WB02X_FLASH                  2    
#define N32WB02X_CHIP                   1

//N32WB02X_CHIP
#define N32WB02X_TYPE                   N32WB02X_CHIP 

#define HSE_CLK         1
#define HSI_CLK         0
extern uint8_t  g_clock_src; 
extern uint8_t g_phy_2mbps_support;
extern uint8_t g_auto_start_data_length_req;
extern uint32_t g_system_hsi_clk;
extern uint8_t g_sleep_sleep_enable;
extern uint8_t g_sleep_status_flag;
extern uint8_t  g_status_io_stat;
extern uint32_t g_timeoutcnt;   //TODO add global var

//32k----global var 
extern uint32_t g_lsi_accuracy;
extern uint32_t g_lsi_count_n_syscle;  
extern uint32_t g_lsi_1_syscle_cal_value;  
extern uint32_t g_lsi_1_syscle_cnt_value;
extern uint8_t  g_recalib_lsi_flag;  //0 no recalib  1: recalib


//32M----global var
extern uint32_t g_hsi_accuracy;
extern uint16_t g_calib_stat_flag;
extern uint8_t  g_recalib_hsi_flag;  //0 no recalib  1: recalib


//64M  2048  96M 3072
#define HSI_32M_VAL      1024
#define HSI_64M_VAL      2048
#define HSI_96M_VAL      3072
#define HSI_100M_VAL     3200
extern uint16_t g_cal_hsi_cnt_value;  //64M

//modem flag 
#define MODEM_CHIP      1
extern uint8_t  g_modem_flag;


extern bool g_hci_transport_onff;

//modify hci 
#define HCI_TRANS_NORMAL  0
#define HCI_TRANS_SOFT_FLOW  1
extern uint8_t g_hci_transport_type;  //0: traditional mode 1: modify mode

#define  STAGE_IDLE     0x20
#define  STAGE_TX       0x40    
#define  STAGE_RX       0x80
#define  STAGE_IDLE_DMA_LEN  3
extern uint8_t g_hci_transport_process;        //0x00: idle  0x40: tx process 0x80: rx process
extern uint16_t g_acl_tx_sch_interval;
extern uint8_t  acl_flow_ctrl_num;
extern uint8_t  acl_flow_ctrl;


extern uint8_t* g_soft_flow_tx_buf;
extern uint16_t g_soft_flow_tx_len;
extern void (*g_soft_flow_tx_callback)(void*, uint8_t);
extern void * g_soft_flow_tx_dummy;
extern uint16_t g_soft_flow_rx_len;
extern uint16_t g_length_2_data_intv;

#define  HCI_LPUART     0x0
#define  HCI_SPI        0x1    
extern uint8_t g_hci_ch_type;


extern uint32_t g_uart_irq_wakeup_time;
extern uint32_t g_spi_data_ready_2_irq_rise;

extern uint32_t dma_recv_data_time_window_size;
extern uint32_t g_uart_bps;


extern uint32_t g_saverc32kcnt;
extern uint8_t g_delay_div_param;
extern uint8_t g_delay_us_div_param;



/// Radio Drift
extern uint16_t g_lpclk_drift;  // 500ppm
///IFS
extern uint8_t  g_rx_ifs;
extern uint8_t  g_tx_ifs;

extern uint16_t g_adv_int_undirect;
extern uint16_t g_adv_int_direct;

extern uint16_t g_max_octets;
extern uint16_t g_max_time;

extern uint16_t g_company_id;
extern uint8_t g_version_sw_major;
extern uint8_t g_version_sw_minor;
extern uint8_t g_version_sw_build;

extern bool g_loop_test;                      
extern bool g_ota_test;                     
extern bool g_ota_test_mtu_247_dle;   
extern bool g_app_init_branch;
extern uint16_t g_adv_intv; 
extern uint16_t g_uart_change_delay;
extern uint8_t g_delay_recv_more_data;
extern uint16_t g_uart_irq_rise_wait_time;
extern uint8_t g_ltk_req_neg_reply;
extern uint8_t g_debug_printf;

extern uint16_t g_twrm;   
extern uint16_t g_twosc;  
extern uint16_t g_twext; 
extern uint8_t rwip_prog_delay;

extern uint16_t g_rc32k_trim;
extern uint8_t (* prf_add_profile_ptr)(void * params, uint16_t *prf_task);
extern uint16_t (*prf_get_task_from_id_ptr)(uint16_t );
extern uint16_t (*prf_get_id_from_task_ptr)(uint16_t task);
extern void (*SVC_IRQHandler_ptr)(uint8_t);
extern void (*NMI_IRQHandler_ptr)(void)   ;
extern void (*HardFault_Handler_ptr)(void);
extern void (*SysTick_IRQHandler_ptr)(void);
extern void (*PendSV_IRQHandler_ptr)(void);

extern bool g_ramcode_work_flag;
extern uint8_t  g_sch_prog_fifo_iar_branch;
extern uint8_t  g_sch_arb_insert_branch;
extern bool g_le_coded_phy_500;
#define BT_DEFAULT_BDADDR  {{0x11, 0x22, 0x33, 0x33, 0x22, 0x33}}
extern struct bd_addr g_co_default_bdaddr;
    
//extern uint8_t  BLE_NORMAL_WIN_SIZE;
//extern uint8_t  BLE_RX_DESC_NB;
//extern uint8_t  RWIP_MINIMUM_SLEEP_TIME;
extern uint16_t  SCH_SCAN_EVT_DUR_DFT;        
extern struct le_features llm_local_le_feats;

extern uint32_t g_rwip_patch_addr;       
extern uint16_t g_rwip_patch_addr_size; 
extern uint32_t g_eeprom_start_addr;       

extern uint32_t g_rwip_heap_env;    
extern uint16_t g_rwip_heap_env_size;
extern uint32_t g_rwip_heap_db;       
extern uint16_t g_rwip_heap_db_size;
extern uint32_t g_rwip_heap_msg;
extern uint32_t g_rwip_heap_msg_size;
extern uint32_t g_rwip_heap_non_ret;
extern uint32_t g_rwip_heap_non_ret_size;       

extern uint8_t*  utils_recv_buf_ptr;
extern uint16_t  utils_max_buf_len;

extern uint8_t*  g_attr_table;
extern uint16_t  ATTR_TABLE_LENGTH;

extern uint8_t* g_app_tx_buf_ptr;
extern uint16_t g_app_tx_buf_len;

extern uint8_t* g_app_bone_table_ptr;
extern uint16_t g_app_bone_table_size;
extern struct prf_task_cbs* g_cbs;

void reinit_params(void);

#ifdef __cplusplus
}
#endif

#endif /* __GLOBAL_VER_H__ */
