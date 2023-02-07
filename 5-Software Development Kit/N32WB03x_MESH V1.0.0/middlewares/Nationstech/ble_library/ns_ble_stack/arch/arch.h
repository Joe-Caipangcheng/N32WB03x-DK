/**
 ****************************************************************************************
 *
 * @file arch.h
 *
 * @brief This file contains the definitions of the macros and functions that are
 * architecture dependent.  The implementation of those is implemented in the
 * appropriate architecture directory.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */


#ifndef _ARCH_H_
#define _ARCH_H_

/**
 ****************************************************************************************
 * @defgroup REFIP
 * @brief Reference IP Platform
 *
 * This module contains reference platform components - REFIP.
 *
 *
 * @{
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @defgroup DRIVERS
 * @ingroup REFIP
 * @brief Reference IP Platform Drivers
 *
 * This module contains the necessary drivers to run the platform with the
 * RW BT SW protocol stack.
 *
 * This has the declaration of the platform architecture API.
 *
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>        // standard integer definition
#include "compiler.h"      // inline functions


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */
/// Macro to read a platform register
#define REG_PL_RD(addr)              (*(volatile uint32_t *)(addr))

/// Macro to write a platform register
#define REG_PL_WR(addr, value)       (*(volatile uint32_t *)(addr)) = (value)

/// Macro to read a common ip register
#define REG_IP_RD(addr)              (*(volatile uint32_t *)(addr))

/// Macro to write a common ip register
#define REG_IP_WR(addr, value)       (*(volatile uint32_t *)(addr)) = (value)

/// Macro to read a BLE register
#define REG_BLE_RD(addr)             (*(volatile uint32_t *)(addr))

/// Macro to write a BLE register
#define REG_BLE_WR(addr, value)      (*(volatile uint32_t *)(addr)) = (value)

/// Macro to read a BLE control structure field (16-bit wide)
#define EM_BLE_RD(addr)              (*(volatile uint16_t *)(addr))

/// Macro to write a BLE control structure field (16-bit wide)
#define EM_BLE_WR(addr, value)       (*(volatile uint16_t *)(addr)) = (value)

/// Macro to read a BT register
#define REG_BT_RD(addr)              (*(volatile uint32_t *)(addr))

/// Macro to write a BT register
#define REG_BT_WR(addr, value)       (*(volatile uint32_t *)(addr)) = (value)

/// Macro to read a BT control structure field (16-bit wide)
#define EM_BT_RD(addr)               (*(volatile uint16_t *)(addr))

/// Macro to write a BT control structure field (16-bit wide)
#define EM_BT_WR(addr, value)        (*(volatile uint16_t *)(addr)) = (value)

/// Macro to read accelerometer register
#define REG_ACC_RD(addr)             (acc_reg_rd((uint8_t)addr))
/// Macro to write accelerometer register
#define REG_ACC_WR(addr, value)      (acc_reg_wr((uint8_t)addr, (uint8_t)value))

/// Macro to read RTC register
#define REG_RTC_RD(addr)             (rtc_reg_rd((uint8_t)addr))
/// Macro to write RTC register
#define REG_RTC_WR(addr, value)      (rtc_reg_wr((uint8_t)addr, (uint8_t)value))

/// Macro to read a EM field (16-bit wide)
#define EM_RD(addr)                  (*(volatile uint16_t *)(addr))

/// Macro to write a EM field (16-bit wide)
#define EM_WR(addr, value)           (*(volatile uint16_t *)(addr)) = (value)
/*
 * CPU WORD SIZE
 ****************************************************************************************
 */
/// ARM is a 32-bit CPU
#define CPU_WORD_SIZE   4

/*
 * CPU Endianness
 ****************************************************************************************
 */
/// ARM is little endian
#define CPU_LE          1

/*
 * DEBUG configuration
 ****************************************************************************************
 */
#if defined(CFG_DBG)
#define PLF_DEBUG          0///1
#else //CFG_DBG
#define PLF_DEBUG          0
#endif //CFG_DBG


/*
 * NVDS
 ****************************************************************************************
 */

/// NVDS
#ifdef CFG_NVDS
#define PLF_NVDS             1
#else // CFG_NVDS
#define PLF_NVDS             0
#endif // CFG_NVDS


/*
 * UART
 ****************************************************************************************
 */

/// UART
#define PLF_UART                                        1    //  0  //
/*uart enables dma*/
#define UART_DMA_INT_EN                                   0    // 1    //

/*SPIS*/
#define PLF_SPIS                                     1   // 0    //
/*SPI enables dma*/
#define SPI_DMA_INT_EN                               1    // 0    //

#define PLF_MCDL                                     1

#define  EIF_API_UART                                0x55
#define  EIF_API_SPI                                 0x66
/*
 * DEFINES
 ****************************************************************************************
 */

/// Possible errors detected by FW
#define    RESET_NO_ERROR         0x00000000
#define    RESET_MEM_ALLOC_FAIL   0xF2F2F2F2

/// Reset platform and stay in ROM
#define    RESET_TO_ROM           0xA5A5A5A5
/// Reset platform and reload FW
#define    RESET_AND_LOAD_FW      0xC3C3C3C3

/// Exchange memory size limit
#define    EM_SIZE_LIMIT          0x4000 //TODO/Ambiq/32K
/*
 * EXPORTED FUNCTION DECLARATION
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Compute size of SW stack used.
 *
 * This function is compute the maximum size stack used by SW.
 *
 * @return Size of stack used (in bytes)
 ****************************************************************************************
 */
uint16_t get_stack_usage(void);

/**
 ****************************************************************************************
 * @brief Re-boot FW.
 *
 * This function is used to re-boot the FW when error has been detected, it is the end of
 * the current FW execution.
 * After waiting transfers on UART to be finished, and storing the information that
 * FW has re-booted by itself in a non-loaded area, the FW restart by branching at FW
 * entry point.
 *
 * Note: when calling this function, the code after it will not be executed.
 *
 * @param[in] error      Error detected by FW
 ****************************************************************************************
 */
void platform_reset(uint32_t error);

void assert_err(const char* condition, const char* file, int line);

void eif_assert_param(int param0, int param1, const char* file, int line);


void assert_warn(int param0, int param1, const char* file, int line);

#if PLF_DEBUG
/**
 ****************************************************************************************
 * @brief Print the assertion error reason and loop forever.
 *
 * @param condition C string containing the condition.
 * @param file C string containing file where the assertion is located.
 * @param line Line number in the file where the assertion is located.
 ****************************************************************************************
 */
void assert_err(const char* condition, const char* file, int line);

/**
 ****************************************************************************************
 * @brief Print the assertion error reason and loop forever.
 * The parameter value that is causing the assertion will also be disclosed.
 *
 * @param param0 parameter value 0.
 * @param param1 parameter value 1.
 * @param file C string containing file where the assertion is located.
 * @param line Line number in the file where the assertion is located.
 ****************************************************************************************
 */
void assert_param(int param0, int param1, const char* file, int line);

/**
 ****************************************************************************************
 * @brief Print the assertion warning reason.
 *
 * @param param0 parameter value 0.
 * @param param1 parameter value 1.
 * @param file C string containing file where the assertion is located.
 * @param line Line number in the file where the assertion is located.
 ****************************************************************************************
 */
void assert_warn(int param0, int param1, const char* file, int line);


/**
 ****************************************************************************************
 * @brief Dump data value into FW.
 *
 * @param data start pointer of the data.
 * @param length data size to dump
 ****************************************************************************************
 */
void dump_data(uint8_t* data, uint16_t length);
#endif //PLF_DEBUG


/*
 * ASSERTION CHECK
 ****************************************************************************************
 */
#if PLF_DEBUG
/// Assertions showing a critical error that could require a full system reset
#define ASSERT_ERR(cond)                              \
    do {                                              \
        if (!(cond)) {                                \
            assert_err(#cond, __FILE__, __LINE__);  \
        }                                             \
    } while(0)

/// Assertions showing a critical error that could require a full system reset
#define ASSERT_INFO(cond, param0, param1)             \
    do {                                              \
        if (!(cond)) {                                \
            eif_assert_param((int)param0, (int)param1, __FILE__, __LINE__);  \
        }                                             \
    } while(0)

/// Assertions showing a non-critical problem that has to be fixed by the SW
#define ASSERT_WARN(cond, param0, param1)             \
    do {                                              \
        if (!(cond)) {                                \
            assert_warn((int)param0, (int)param1, __FILE__, __LINE__); \
        }                                             \
    } while(0)

#define DUMP_DATA(data, length) \
    dump_data((uint8_t*)data, length)

#else
#define ASSERT_ERR(cond)                              \
//    do {                                              \
//        if (!(cond)) {                                \
//            assert_err(#cond, __FILE__, __LINE__);  \
//        }                                             \
//    } while(0)

/// Assertions showing a critical error that could require a full system reset
#define ASSERT_INFO(cond, param0, param1)             \
//    do {                                              \
//        if (!(cond)) {                                \
//            assert_param((int)param0, (int)param1, __FILE__, __LINE__);  \
//        }                                             \
//    } while(0)

/// Assertions showing a non-critical problem that has to be fixed by the SW
#define ASSERT_WARN(cond, param0, param1)             \
//4

/// DUMP data array present in the SW.
#define DUMP_DATA(data, length)
#endif //PLF_DEBUG


/// Object allocated in shared memory - check linker script
#define __SHARED __attribute__ ((section("shram")))

// required to define GLOBAL_INT_** macros as inline assembly. This file is included after
// definition of ASSERT macros as they are used inside ll.h
//#include "ll.h"     // ll definitions
/// @} DRIVERS

void SystemInit (void);

#endif // _ARCH_H_
