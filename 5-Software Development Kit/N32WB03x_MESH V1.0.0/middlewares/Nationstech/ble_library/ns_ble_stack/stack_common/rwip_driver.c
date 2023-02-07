#include "rwip_config.h"     // RW SW configuration
#include "n32wb03x.h"
#include <string.h>          // for mem* functions
#include "rwip.h"            // RW definitions
#include "rwip_int.h"        // RW internal definitions
#include "arch.h"            // Platform architecture definition
#include "em_map.h"
#include "rf.h"

#include "ke.h"
#include "ke_event.h"
#include "sch_alarm.h"       // for the half slot target ISR
#include "sch_arb.h"         // for the half us target ISR
#include "sch_prog.h"        // for the fifo/clock ISRs
#include "reg_ipcore.h"
#include "aes.h"             // AES result function
#include "rwble.h"           // for sleep and wake-up specific functions
#include "lld.h"             // for AES encryption handler
#include "global_var.h"

extern __INLINE uint32_t co_max(uint32_t a, uint32_t b);


/*
 * DEFINES
 ****************************************************************************************
 */
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/// Sleep Duration Value in periodic wake-up mode in Half slots
#define MAX_SLEEP_DURATION_PERIODIC_WAKEUP      0x0640  // 0.5s

/// Sleep Duration Value in external wake-up mode
#define MAX_SLEEP_DURATION_EXTERNAL_WAKEUP      0x7D00  //10s

/**
 * Inverse an intra-half-slot value (in half-us), from/to following formats:
 *   - A: elapsed time from the previous half-slot (in half-us)
 *   - B: remaining time to the next half-slot (in half-us)
 * The function from A to B or from B to A.
 *  ____________________________________________________________________________________________________
 *     Half-slot N-1            |             Half-slot N              |             Half-slot N+1
 *  ____________________________|______________________________________|________________________________
 *                              |<---------- A ---------->|<---- B --->|
 *  ____________________________|______________________________________|________________________________
 */
#define HALF_SLOT_INV(x)  (HALF_SLOT_SIZE - x - 1)

#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)



/**
 ****************************************************************************************
 * @brief Converts a duration in lp cycles into a duration in half us.
 *
 * The function converts a duration in lp cycles into a duration is half us, according to the
 * low power clock frequency (32768Hz or 32000Hz).
 *
 * To do this the following formula are applied:
 *
 *   Tus = (x*30.517578125)*2 = (30*x + x/2 + x/64 + x/512)*2 = (61*x + (x*8 + x)/256) for a 32.768kHz clock or
 *   Tus = (x*31.25)*2        = (31*x + x/4) * 2              = (62*x + x/2)           for a 32kHz clock
 *
 * @param[in]     lpcycles    duration in lp cycles
 * @param[in|out] error_corr  Insert and retrieve error created by truncating the LP Cycle Time to a half us (32kHz: 1/2 half us | 32.768kHz: 1/256 half-us)
 *
 * @return duration in half us
 ****************************************************************************************
 */
__STATIC uint32_t rwip_lpcycles_2_hus(uint32_t lpcycles, uint32_t *error_corr)
{
    uint32_t res;
 
#if (G15_ORG)  //org
    *error_corr = lpcycles + *error_corr;
    res = *error_corr >> 1;
    *error_corr = *error_corr - (res << 1);

    //12 lpcycles    312.5us   12 *62 *   800/1000
    //res = (62 * lpcycles + res)*g_lsi_1_syscle_cnt_value/g_lsi_1_syscle_cal_value;
    float  tmep = (float)g_lsi_1_syscle_cnt_value/1000.0;
    res = (uint32_t)((62 * lpcycles + res)*tmep);

#else
    
    #if (LSC_LSI_32000_HZ)
    *error_corr = lpcycles + *error_corr;
    res = *error_corr >> 1;
    *error_corr = *error_corr - (res << 1);
    res = 62 * lpcycles + res;
    #elif (LSC_LSE_32768_HZ)
    *error_corr = (lpcycles << 3) + lpcycles + *error_corr;
    res = *error_corr >> 8;
    *error_corr = *error_corr - (res << 8);
    res = 61 * lpcycles + res;
    #endif  //LSC_LSE_32768_HZ

#endif

    return(res);
}

/**
 ****************************************************************************************
 * @brief Converts a duration in half slots into a number of low power clock cycles.
 * The function converts a duration in half slots into a number of low power clock cycles.
 * Sleep clock runs at either 32768Hz or 32000Hz, so this function divides the value in
 * slots by 10.24 or 10 depending on the case.
 * To do this the following formulae are applied:
 *
 *   N = x * 10.24 = (1024 * x)/100 for a 32.768kHz clock or
 *   N = x * 10                     for a 32kHz clock
 *
 * @param[in] hs_cnt    The value in half slot count
 *
 * @return The number of low power clock cycles corresponding to the slot count
 *
 ****************************************************************************************
 */
 

__STATIC int32_t rwip_slot_2_lpcycles(int32_t hs_cnt)
{
    uint32_t lpcycles;
    
#if (G15_ORG)  //org   
    
    //1 hslot   10 lpcycle       312.5   *  1000/800 
    lpcycles = hs_cnt * 10 * g_lsi_1_syscle_cal_value/g_lsi_1_syscle_cnt_value;
    lpcycles--;

#else
    
    #if (LSC_LSI_32000_HZ)
    //1 hslot   10 lpcycle       312.5   *  1000/800 
    lpcycles = hs_cnt * 10;
    #elif (LSC_LSE_32768_HZ)
    lpcycles = (hs_cnt << 10)/100;
    #endif //LSC_LSE_32768_HZ
    
    lpcycles--;
    
#endif
    
    return(lpcycles);
}


/**
 ****************************************************************************************
 * @brief Converts a duration in us into a duration in lp cycles.
 *
 * The function converts a duration in us into a duration is lp cycles, according to the
 * low power clock frequency (32768Hz or 32000Hz).
 *
 * @param[in] us    duration in us
 *
 * @return duration in lpcycles
 ****************************************************************************************
 */

//TODO  just used one time  no need more check 
__STATIC uint32_t rwip_us_2_lpcycles(uint32_t us)
{
    uint32_t lpcycles;
    
#if (G15_ORG)  //org    
    #if 0
    lpcycles = ((us * 32) + (999)) / 1000;
    #else
    lpcycles = (us *32* g_lsi_1_syscle_cal_value/g_lsi_1_syscle_cnt_value + (999)) / 1000;
    #endif 
#else
     
    #if (LSC_LSI_32000_HZ)
    lpcycles = ((us * 32) + (999)) / 1000;
    #elif (LSC_LSE_32768_HZ)
    lpcycles = ((us * 32768) + (999999)) / 1000000;
    #endif  //LSC_LSE_32768_HZ
    
#endif
    
    return(lpcycles);
}


/**
 ****************************************************************************************
 * @brief Wake-up from Core sleep.
 *
 * Compute and apply the clock correction according to duration of the deep sleep.
 ****************************************************************************************
 */
__STATIC void rwip_wakeup(void)
{
    uint16_t fintetime_correction;
    // duration in half us
    uint32_t dur_hus;
    // duration in half slot
    uint32_t dur_hslot;
    // Get the number of low power sleep period
    uint32_t slp_period = ip_deepslstat_get();

    // Sleep is over now
    rwip_prevent_sleep_clear(RW_DEEP_SLEEP);

    // Prevent going to deep sleep until a slot interrupt is received
    rwip_prevent_sleep_set(RW_WAKE_UP_ONGOING);

    // Compensate the base time counter and fine time counter by the number of slept periods
    dur_hus = rwip_lpcycles_2_hus(slp_period, &(rwip_env.sleep_acc_error));
    // Compute the sleep duration (based on number of low power clock cycles)
    dur_hslot = dur_hus / HALF_SLOT_SIZE;

    // retrieve halfslot sleep duration
    fintetime_correction = (HALF_SLOT_SIZE-1) - (dur_hus - dur_hslot*HALF_SLOT_SIZE);
//    //printf("44=%x\r\n",dur_hus / HALF_SLOT_SIZE);
    // The correction values are then deduced from the sleep duration in us
    ip_clkncntcorr_pack(/*absdelta*/ 1, /*clkncntcorr*/ dur_hus / HALF_SLOT_SIZE);

    // The correction values are then deduced from the sleep duration in us
    ip_finecntcorr_setf(fintetime_correction);

    // Start the correction
    ip_deepslcntl_deep_sleep_corr_en_setf(1);

    // Enable the RWBT slot interrupt
    ip_intcntl1_clknintsrmsk_setf(0);
    ip_intcntl1_clknintmsk_setf(1);
    ip_intack1_clear(IP_CLKNINTACK_BIT);

}


uint8_t rwip_sleep(void)  
{
    uint8_t sleep_res = RWIP_ACTIVE;
     
    do
    {
        if(g_sleep_sleep_enable == 0)                       //sleep condition param  0: not sleep 1: allow to sleep 
        {
            break;
        }
        
        #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
        int32_t sleep_duration;
        rwip_time_t current_time;
        #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

        /************************************************************************
         **************            CHECK KERNEL EVENTS             **************
         ************************************************************************/
        
        // Check if some kernel processing is ongoing (during wakeup, kernel events are not processed)
        if (((rwip_env.prevent_sleep & RW_WAKE_UP_ONGOING) == 0) && !ke_sleep_check())
        {
            break;
        }

        // Processor sleep can be enabled
        sleep_res = RWIP_CPU_SLEEP;

        /************************************************************************
         **************              CHECK RW FLAGS                **************
         ************************************************************************/
        // First check if no pending procedure prevent from going to sleep
        if (rwip_env.prevent_sleep != 0)
        {
            break;
        }

        #if (BLE_EMB_PRESENT || BT_EMB_PRESENT)

        /************************************************************************
         **************           Retrieve Current time            **************
         ************************************************************************/
        current_time = rwip_time_get();
        // Consider 2 half-slots for clock correction (worst case: 1 half-slot before correction, 1 half-slot after correction)
        current_time.hs += 2;
        
        // Remove 1 more slot because next slot will be started at end of function
        if((HALF_SLOT_INV(current_time.hus)) < rwip_env.sleep_algo_dur)
        {
            current_time.hs += 1;
        }
        // Be sure that we don't exceed the clock wrapping time
        current_time.hs &= RWIP_MAX_CLOCK_TIME;

        /************************************************************************
         ******* COMPUTE SLEEP TIME ACCORDING TO 1 MS AND HALF SLOT TIMER ******
         ************************************************************************/

        // put sleep duration to maximum value
        sleep_duration = (rwip_env.ext_wakeup_enable) ? MAX_SLEEP_DURATION_EXTERNAL_WAKEUP : MAX_SLEEP_DURATION_PERIODIC_WAKEUP;

        // check if 1ms timer is active
        if(rwip_env.timer_1ms_target.hs != RWIP_INVALID_TARGET_TIME)
        {
            int32_t duration = CLK_DIFF(current_time.hs, rwip_env.timer_1ms_target.hs);
            // update sleep duration to minimum value
            sleep_duration = co_min_s(sleep_duration, duration);
        }

        // check if Half slot timer is active
        if(rwip_env.timer_hs_target != RWIP_INVALID_TARGET_TIME)
        {
            int32_t duration = CLK_DIFF(current_time.hs, rwip_env.timer_hs_target);
            // update sleep duration to minimum value
            sleep_duration = co_min_s(sleep_duration, duration);
        }

        // check if Half us timer is active
        if(rwip_env.timer_hus_target != RWIP_INVALID_TARGET_TIME)
        {
            int32_t duration = CLK_DIFF(current_time.hs, rwip_env.timer_hus_target);
            // update sleep duration to minimum value
            sleep_duration = co_min_s(sleep_duration, duration);
        }

        // A timer ISR is not yet handled or will be raised soon
        // note the sleep duration could be negative, that's why it's useful to check if a minimum requirement is ok
        // at least one half slot.
        if(sleep_duration <= RWIP_MINIMUM_SLEEP_TIME)
        {
            break;
        }
		 if((RCC->OSCFCSR & 0x01))
		{
			uint32_t count_value=0;
			count_value = RCC->OSCFCLSICNT; 
			g_lsi_1_syscle_cnt_value = (count_value / g_lsi_count_n_syscle) + (count_value % g_lsi_count_n_syscle) / (g_lsi_count_n_syscle/2);   
		}
        
        /************************************************************************
         **************           CHECK SLEEP TIME                 **************
         ************************************************************************/
       
        ////printf("sleep_duration = %x\r\n", sleep_duration);
        sleep_duration = rwip_slot_2_lpcycles(sleep_duration);
        // check if sleep duration is sufficient according to wake-up delay
       
       // //printf("sleep_duration = %x\r\n", sleep_duration);
        if(sleep_duration < rwip_env.lp_cycle_wakeup_delay + 1)
        {
            break;
        }

        sleep_res = RWIP_DEEP_SLEEP;
        
        /************************************************************************
         **************          PROGRAM CORE DEEP SLEEP           **************
         ************************************************************************/
        // Program wake-up time
        ip_deepslwkup_set(sleep_duration);

        // Prevent re-entering sleep, until it effectively sleeps and wakes up
        rwip_prevent_sleep_set(RW_DEEP_SLEEP);

        /************************************************************************
         **************               SWITCH OFF RF                **************
         ************************************************************************/
        rwip_rf.sleep();
        #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
       

    } while(0);

    return sleep_res;
}

#define REG32(addr)          (*(volatile uint32_t *)(addr))
void llhwc_phy_prerx_flash(void)
{
    REG32(0x40011004) = 0x20; 
    REG32(MODEM_BASE +0x5C *4) = 0x03;    
    REG32(MODEM_BASE +0xff *4) = 0x01;
    REG32(MODEM_BASE +0x56 *4) = 0x05;	
    REG32(MODEM_BASE +0xff *4) = 0x00;
    REG32(MODEM_BASE +0xd4 *4) = 0x0a; 

    REG32(MODEM_BASE +0x104 *4) = 0x00027e12;  
    REG32(MODEM_BASE +0x10A *4) = 0x00037e12;
    REG32(MODEM_BASE +0x110 *4) = 0x00027e12;    
    REG32(MODEM_BASE +0x11c *4) = 0x00a0c202;
    REG32(MODEM_BASE +0x11d *4) = 0x00bFC302;
    REG32(MODEM_BASE +0x11e *4) = 0x00080B02;
    REG32(MODEM_BASE +0x11f *4) = 0x00885902;
    REG32(MODEM_BASE +0x120 *4) = 0x00007d02;
    REG32(MODEM_BASE +0x121 *4) = 0x00845b02;
    REG32(MODEM_BASE +0x122 *4) = 0x00845d02;
    REG32(MODEM_BASE +0x123 *4) = 0x00805f02;
    REG32(MODEM_BASE +0x124 *4) = 0x28a46e02;	 
    REG32(MODEM_BASE +0x125 *4) = 0x14805a02;
    REG32(MODEM_BASE +0x126 *4) = 0x08480002;
    REG32(MODEM_BASE +0x127 *4) = 0x50080a02;	 
    REG32(MODEM_BASE +0x128 *4) = 0x008c5902;	
    REG32(MODEM_BASE +0x129 *4) = 0x14137012;   	 
    REG32(MODEM_BASE +0x134 *4) = 0x00885902;
    REG32(MODEM_BASE +0x135 *4) = 0x00000b12;
    REG32(MODEM_BASE +0x138 *4) = 0x00a0c202;
    REG32(MODEM_BASE +0x139 *4) = 0x00bFC302;
    REG32(MODEM_BASE +0x13A *4) = 0x00080B02;
    REG32(MODEM_BASE +0x13B *4) = 0x00885902;
    REG32(MODEM_BASE +0x13C *4) = 0x00007d02; 
    REG32(MODEM_BASE +0x13D *4) = 0x00845b02; 
    REG32(MODEM_BASE +0x13E *4) = 0x00845d02; 
    REG32(MODEM_BASE +0x13F *4) = 0x00805f02; 
    REG32(MODEM_BASE +0x140 *4) = 0x28246e02;   
    REG32(MODEM_BASE +0x141 *4) = 0x14805a02; 
    REG32(MODEM_BASE +0x142 *4) = 0x08AC0002; 
    REG32(MODEM_BASE +0x143 *4) = 0x78060a02;  
    REG32(MODEM_BASE +0x144 *4) = 0x008c5912;      
    REG32(MODEM_BASE +0x14D *4) = 0x00045D02;
    REG32(MODEM_BASE +0x14E *4) = 0x00885902;
    REG32(MODEM_BASE +0x14F *4) = 0x00000b12;
}


/**
 * @brief  set rf tx power
 * @param  
 * @return 
 * @note   
 */
rf_tx_power_t g_rf_tx_power = TX_POWER_0_DBM;
void rf_tx_power_set(rf_tx_power_t pwr)
{
    if( pwr > TX_POWER_Pos6_DBM)
    {
        //error
        return;
    }
    
    g_rf_tx_power = pwr;
    if(TXPWR_MAP[g_rf_tx_power][0])
    {
        REG32(0x40007014) &= ~(0x300);
        REG32(0x40007014) |= 0x200;  
        REG32(0x40007020) &= ~(0x3f << 14);
        REG32(0x40007020) |=  (0x2e << 14);
    }
    else
    {
        REG32(0x40007014) &= ~(0x300);
        REG32(0x40007020) &= ~(0x3f << 14);
        REG32(0x40007020) |=  (0x04 << 14);
    }
    REG32(0x4002255C) = TXPWR_MAP[g_rf_tx_power][1];
    REG32(0x4002258C) = TXPWR_MAP[g_rf_tx_power][2];
}

void rwip_slp_isr(void)
{
    // Check interrupt status and call the appropriate handlers
    uint32_t irq_stat      = ip_intstat1_get();
    if (irq_stat & IP_SLPINTSTAT_BIT)
    {
        // ack Sleep wakeup interrupt
        ip_intack1_slpintack_clearf(1);
        //when Baseband wake up then we need initialize the modem register
        //rf_set_modem_register(); //TODO 
        //llhwc_phy_initialization();
        llhwc_phy_prerx_flash();
        
        //rf tx power update after sleep
        REG32(0x4002255C) = TXPWR_MAP[g_rf_tx_power][1];
        REG32(0x4002258C) = TXPWR_MAP[g_rf_tx_power][2];
        //Handle wake-up
        rwip_wakeup();
        g_sleep_status_flag = 0;
    }
}


/**
 ****************************************************************************************
 * @brief Handles crypto event (to provide results out of interrupt context
 ****************************************************************************************
 */
__STATIC void rwip_crypt_evt_handler(void)
{
    uint8_t aes_result[KEY_LEN];

    // Clear event
    ke_event_clear(KE_EVENT_AES_END);

    // Load AES result
    em_rd(aes_result, EM_ENC_OUT_OFFSET, KEY_LEN);

    // inform AES result handler
    aes_result_handler(CO_ERROR_NO_ERROR, aes_result);
}

void rwip_driver_init(uint8_t init_type)
{

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)

    switch (init_type)
    {
        case RWIP_INIT:
        {
            // Register AES event
            ke_event_callback_set(KE_EVENT_AES_END, &rwip_crypt_evt_handler);

            // ensure that we will never enter in deep sleep
            rwip_prevent_sleep_set(RW_PLF_DEEP_SLEEP_DISABLED);
        }
        break;

        case RWIP_RST:
        {
            // Do nothing
        }
        // No break

        case RWIP_1ST_RST:
        {
            //uint8_t length;
            uint8_t sleep_enable = 0;
            uint8_t ext_wakeup_enable;
            #if (BT_DUAL_MODE)
            uint8_t diag_cfg[PARAM_LEN_DIAG_DM_HW];
            #endif // (BT_DUAL_MODE)

            // initialize environment
            rwip_env.prevent_sleep     = 0;
            // clear target timer
            rwip_env.timer_1ms_target.hs = RWIP_INVALID_TARGET_TIME;
            rwip_env.timer_1ms_target.hus = 0;
            rwip_env.timer_hs_target   = RWIP_INVALID_TARGET_TIME;
            rwip_env.timer_hus_target  = RWIP_INVALID_TARGET_TIME;

            // Reset the IP core
            ip_rwdmcntl_master_soft_rst_setf(1);
            while(ip_rwdmcntl_master_soft_rst_getf());

            // Enable default common interrupts
            ip_intcntl1_set(IP_FIFOINTMSK_BIT | IP_CRYPTINTMSK_BIT | IP_SWINTMSK_BIT | IP_SLPINTMSK_BIT);



            // Activate deep sleep feature if enabled in NVDS and in reset mode
            //length = PARAM_LEN_SLEEP_ENABLE;
            //if(rwip_param.get(PARAM_ID_SLEEP_ENABLE, &length, &sleep_enable) != PARAM_OK)
            //{
                sleep_enable = 1;
            //}
            // check is sleep is enabled
            if(sleep_enable != 0)
            {
                uint16_t twext, twosc, twrm;
                // Set max sleep duration depending on wake-up mode
//                if(rwip_param.get(PARAM_ID_EXT_WAKEUP_ENABLE, &length, &ext_wakeup_enable) != PARAM_OK)
//                {
                ext_wakeup_enable = 1;
//                }
                rwip_env.ext_wakeup_enable = (ext_wakeup_enable != 0) ? true : false;

                // Set max sleep duration depending on wake-up mode
//                length = sizeof(rwip_env.sleep_algo_dur);
//                if(rwip_param.get(PARAM_ID_SLEEP_ALGO_DUR, &length, (uint8_t*) &rwip_env.sleep_algo_dur) != PARAM_OK)
//                {
                    // set a default duration: 200 us ==> 400 half us
                    //TODO CHECK
                rwip_env.sleep_algo_dur = 500;
//                }
                // Initialize sleep parameters
                rwip_env.sleep_acc_error   = 0;

                // Get TWrm from NVDS
//            length = sizeof(uint16_t);
//            if (rwip_param.get(PARAM_ID_RM_WAKEUP_TIME, &length, (uint8_t*)&twrm) != PARAM_OK)
//            {
                    // Set default values : 625 us
//                    twrm = SLEEP_RM_WAKEUP_DELAY;
////                }
//                // Get TWosc from NVDS
/////               length = sizeof(uint16_t);
////                if (rwip_param.get(PARAM_ID_OSC_WAKEUP_TIME, &length, (uint8_t*)&twosc) != PARAM_OK)
////                {
//                    // Set default values : 5 ms
//                    twosc = SLEEP_OSC_NORMAL_WAKEUP_DELAY;
////                }

//                // Get TWext from NVDS
////                length = sizeof(uint16_t);
////                if (rwip_param.get(PARAM_ID_EXT_WAKEUP_TIME, &length, (uint8_t*)&twext) != PARAM_OK)
////                {
//                    // Set default values : 5 ms
//                    twext = SLEEP_OSC_EXT_WAKEUP_DELAY;
//                }
                twrm  = rwip_us_2_lpcycles(g_twrm);
                twosc = rwip_us_2_lpcycles(g_twosc);
                twext = rwip_us_2_lpcycles(g_twext);

                // Program register
                ip_enbpreset_pack(twext, twosc, twext);

                // Configure wake up delay to the highest parameter
                twext = co_max(twext, twrm);
                twext = co_max(twext, twosc);

                // Store wake-up delay in lp cycles
                rwip_env.lp_cycle_wakeup_delay = twext;

                // Set the external wakeup parameter
                ip_deepslcntl_extwkupdsb_setf(!rwip_env.ext_wakeup_enable);
            }
            else
            {
                // ensure that we will never enter in deep sleep
                rwip_prevent_sleep_set(RW_PLF_DEEP_SLEEP_DISABLED);
            }

            // Set prefetch time and anticipated prefetch abort time
            ip_timgencntl_pack((IP_PREFETCHABORT_TIME_US << 1), (IP_PREFETCH_TIME_US << 1));

            // Get programming delay from parameters
//            length = PARAM_LEN_PROG_DELAY;
//            if(rwip_param.get(PARAM_ID_PROG_DELAY, &length, &rwip_prog_delay) != PARAM_OK)
//            {
//                rwip_prog_delay = IP_PROG_DELAY_DFT;
//            }
        }
        break;

        default:
        {
            // Do nothing
        }
        break;
    }

    #elif (BLE_HOST_PRESENT)
    // initialize environment
    rwip_env.prevent_sleep     = 0;
    rwip_env.timer_10ms_target = RWIP_INVALID_TARGET_TIME;
    // enable timer
    timer_enable(true);
    #endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)
}

