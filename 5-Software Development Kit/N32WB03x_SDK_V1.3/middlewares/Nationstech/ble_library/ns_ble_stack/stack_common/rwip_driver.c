/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file rwip_driver.c
 * @author Nations Firmware Team
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
#include "rwip_config.h"     // RW SW configuration
#include "global_func.h"
#include "ns_ble.h"
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


#ifdef TRIM_RADIO_FREQUENCY_ENABLE
#define TRIM_STORED_MARK            0xAA55 
#define TRIM_STORE_ADDR3            0x3000 
#define TRIM_READ_CMD_CODE_LEN      0x140
extern const unsigned char  TRIM_READ_CMD_CODE[] ;
typedef uint32_t (*trim_read_cmd_func_t)(uint32_t,uint8_t*,uint32_t);
typedef struct{
    uint16_t mark;
    uint8_t  user_mac[6];
    uint32_t f_cal;
}trim3_stored_t;
/* call initial and after sleep,  */ 
void radio_freq_cal_from_trim(void)
{
    
    static uint32_t f_act = 0xffffffff;
    static uint32_t modem_reg_bc,modem_reg_bd,modem_reg_be; 
    
    trim3_stored_t trim3_read;
    
    if(f_act == 0xffffffff)
    {
        //read radio trim frim otp3
        uint32_t ramcode[TRIM_READ_CMD_CODE_LEN/4 +1 ];
        trim_read_cmd_func_t trim_read_cmd_func = (trim_read_cmd_func_t)((uint8_t*)&ramcode[0] + 0x11);
        memcpy((void*)ramcode,(const void*)TRIM_READ_CMD_CODE,TRIM_READ_CMD_CODE_LEN);
        (*trim_read_cmd_func)(TRIM_STORE_ADDR3, (uint8_t*)&trim3_read, sizeof(trim3_stored_t));
        
        //confirm otp3 valid 
        if(trim3_read.mark == TRIM_STORED_MARK)
        {
            f_act = trim3_read.f_cal;
            uint32_t  delta_k;
            int32_t   delta_fref;
            float      err;

            delta_fref = 8000000 - f_act;
            err = (float)delta_fref / (float)f_act;

            if(delta_fref > 0)
            {
                delta_k = (uint32_t)(9838592 * err);
                modem_reg_bc = (delta_k >> 16) & 0x1;
                modem_reg_bd = (delta_k >> 8) & 0xff;
                modem_reg_be = delta_k & 0xff;
            }
            else if(delta_fref < 0)
            {
                delta_k = 131072 + 9838592 * err;
                modem_reg_bc = ((delta_k >> 16) & 0x1) | 0x2;
                modem_reg_bd = (delta_k >> 8) & 0xff;
                modem_reg_be = delta_k & 0xff;        
            }
        }
        else{
            // invalid  
            f_act  = 0xfffffffe;
        }
    }
    
    if(f_act < 8008000 && f_act > 7002000 )
    {
        REG32(MODEM_BASE + 0x2F0 ) = modem_reg_bc;   //0xbc * 4
        REG32(MODEM_BASE + 0x2F4 ) = modem_reg_bd;   //0xbd * 4
        REG32(MODEM_BASE + 0x2F8 ) = modem_reg_be;   //0xbe * 4
    }
}
#endif
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
__STATIC __INLINE uint32_t rwip_lpcycles_2_hus(uint32_t lpcycles, uint32_t *error_corr)
{
    uint32_t res;

    *error_corr = lpcycles + *error_corr;
    res = *error_corr >> 1;
    *error_corr = *error_corr - (res << 1);

    float  tmep = (float)g_lsi_1_syscle_cnt_value/1000.0;
    res = (uint32_t)((62 * lpcycles + res)*tmep);

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
__STATIC __INLINE int32_t rwip_slot_2_lpcycles(int32_t hs_cnt)
{
    uint32_t lpcycles;
    
    //1 hslot   10 lpcycle       312.5   *  1000/800 
//    lpcycles = hs_cnt * 10 * 1000/g_lsi_1_syscle_cnt_value;
    lpcycles = hs_cnt * 10000 /g_lsi_1_syscle_cnt_value;
    lpcycles--;
    
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
//        if((HALF_SLOT_INV(current_time.hus)) < rwip_env.sleep_algo_dur)
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
        
        if(app_env.lsc_cfg != BLE_LSC_LSE_32768HZ)
        {
            if((RCC->OSCFCSR & 0x01))
            {
                uint32_t count_value=0;
                count_value = RCC->OSCFCLSICNT; 
                g_lsi_1_syscle_cnt_value = (count_value / g_lsi_count_n_syscle) + (count_value % g_lsi_count_n_syscle) / (g_lsi_count_n_syscle/2);   
            }
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
__STATIC uint8_t modem_setmode[2] = {0xa0,0x00};
__STATIC __INLINE void llhwc_setmode_reg_config(void)
{
    if(app_env.scan_actv_idx != 0xff || 
       scan_env.scan_enable) 
    {
        REG32(MODEM_BASE +0xc2 *4) = modem_setmode[0];
        REG32(MODEM_BASE +0xc7 *4) = modem_setmode[1];
        //setmode1m from 0x104 to 0x109
        REG32(MODEM_BASE +0x104 *4) = 0x00027e12; 
        //setmode2m from 0x10a to 0x10f
        REG32(MODEM_BASE +0x10A *4) = 0x00037e12;
        //setmodelr from 0x110 to 0x11f
        REG32(MODEM_BASE +0x110 *4) = 0x00027e12;        
    }
    else{
        //setmode1m from 0x104 to 0x109
        REG32(MODEM_BASE +0x104 *4) = 0x00027e02;
        REG32(MODEM_BASE +0x105 *4) = 0x00a0c202;
        REG32(MODEM_BASE +0x106 *4) = 0x0000c712;
        
        //setmode2m from 0x10a to 0x10f
        REG32(MODEM_BASE +0x10A *4) = 0x00037e02;
        REG32(MODEM_BASE +0x10B *4) = 0x00a0c202;
        REG32(MODEM_BASE +0x10C *4) = 0x0080c712; 
        //setmodelr from 0x110 to 0x11f
        REG32(MODEM_BASE +0x110 *4) = 0x00027e02;
        REG32(MODEM_BASE +0x111 *4) = 0x00e0c202;
        REG32(MODEM_BASE +0x112 *4) = 0x0000c712;
    }
}

void llhwc_modem_setmode(uint8_t phy)
{
    if(phy == GAP_PHY_2MBPS)
    {
        modem_setmode[0] = 0xa0;
        modem_setmode[1] = 0x80;
    }
    else if(phy == GAP_PHY_CODED || 
            phy == GAP_PHY_500KBPS)
    {
        modem_setmode[0] = 0xe0;
        modem_setmode[1] = 0x00; 
    }
    else //if(phy == GAP_PHY_1MBPS)
    {
        modem_setmode[0] = 0xa0;
        modem_setmode[1] = 0x00;
    }        

    llhwc_setmode_reg_config();

}

void llhwc_phy_prerx_flash(void)
{
	//reduce pll spur (only valid for version b)
    REG32(0x40011004) = 0x20;

	//enable rssi notch filter
    REG32(MODEM_BASE +0xff *4) = 0x01;
    REG32(MODEM_BASE +0x56 *4) = 0x05;	
    REG32(MODEM_BASE +0xff *4) = 0x00;
    REG32(MODEM_BASE +0xd4 *4) = 0x0a; 
    
	llhwc_setmode_reg_config();

	//PRETX from 0x11c to 0x12b
    REG32(MODEM_BASE +0x11c *4) = 0x00f85902; //manually control vcoendly
	REG32(MODEM_BASE +0x11d *4) = 0x00855b02; //turn on vdd_ana
    REG32(MODEM_BASE +0x11e *4) = 0x00805a02; //enable bias current
    REG32(MODEM_BASE +0x11f *4) = 0x00855d02; //turn on vdd_fsynvco
    REG32(MODEM_BASE +0x120 *4) = 0x50805f02; //turn on vdd_adc and delay 20us
    REG32(MODEM_BASE +0x121 *4) = 0x78080002; //turn on pll and delay 30us
    REG32(MODEM_BASE +0x122 *4) = 0x00080a02; //enable iqdiv tx mode
	REG32(MODEM_BASE +0x123 *4) = 0x50ac6e02; //turn on vdd_rffe and vdd_pa and delay 20us
	REG32(MODEM_BASE +0x124 *4) = 0x00fc5902; //manually control vcoendly
    REG32(MODEM_BASE +0x125 *4) = 0x00c80002; //turn on tx
    REG32(MODEM_BASE +0x126 *4) = 0x14137012; //turn on paen and delay 5us

	//POSTTX from 0x12c to 0x137
	REG32(MODEM_BASE +0x12c *4) = 0x14037002;
	REG32(MODEM_BASE +0x131 *4) = 0x00286e02;
	REG32(MODEM_BASE +0x133 *4) = 0x00055d02;
	REG32(MODEM_BASE +0x134 *4) = 0x00055b12;
	
	//PRERX from 0x138 to 0x147
    REG32(MODEM_BASE +0x138 *4) = 0x00f85902; //manually control vcoendly
    REG32(MODEM_BASE +0x139 *4) = 0x00855b02; //turn on vdd_ana
    REG32(MODEM_BASE +0x13A *4) = 0x00805a02; //enable bias current
    REG32(MODEM_BASE +0x13B *4) = 0x00855d02; //turn on vdd_fsynvco
    REG32(MODEM_BASE +0x13C *4) = 0x50805f02; //turn on vdd_adc and delay 20us
    REG32(MODEM_BASE +0x13D *4) = 0x78080002; //turn on pll and delay 30us	 
    REG32(MODEM_BASE +0x13E *4) = 0x00060a02; //enable iqdiv rx mode
    REG32(MODEM_BASE +0x13F *4) = 0x502c6e02; //turn on vdd_rffe and delay 20us
    REG32(MODEM_BASE +0x140 *4) = 0x00bfc302; //turn on rxdata and clk
    REG32(MODEM_BASE +0x141 *4) = 0x00fc5902; //manually control vcoendly
    REG32(MODEM_BASE +0x142 *4) = 0x00ac0012; //turn on rx

	//POSTRX from 0x148 to 0x153
	REG32(MODEM_BASE +0x14C *4) = 0x00286e02;
	REG32(MODEM_BASE +0x14E *4) = 0x00055d02;
    REG32(MODEM_BASE +0x14F *4) = 0x00055b12;
    
    #ifdef TRIM_RADIO_FREQUENCY_ENABLE
    radio_freq_cal_from_trim();
    #endif
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
        //Handle wake-up
        rwip_wakeup();        
        //when Baseband wake up then we need initialize the modem register
        //rf_set_modem_register(); //TODO 
        //llhwc_phy_initialization();
        llhwc_phy_prerx_flash();

        //rf tx power update after sleep
        REG32(0x4002255C) = TXPWR_MAP[g_rf_tx_power][1];
        REG32(0x4002258C) = TXPWR_MAP[g_rf_tx_power][2];

        g_sleep_status_flag = 0;
    }
}



