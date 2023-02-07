#ifndef NORDIC_COMMON_H__
#define NORDIC_COMMON_H__




#define MSB_32(a) (((a) & 0xFF000000) >> 24)
#define LSB_32(a) ((a) & 0x000000FF)
#define MSB_16(a) (((a) & 0xFF00) >> 8)
#define LSB_16(a) ((a) & 0x00FF)
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) < (b) ? (b) : (a))
#define CONCAT_2(p1, p2)      CONCAT_2_(p1, p2)
#define CONCAT_2_(p1, p2)     p1##p2

#define CONCAT_3(p1, p2, p3)  CONCAT_3_(p1, p2, p3)
#define CONCAT_3_(p1, p2, p3) p1##p2##p3

#define STRINGIFY_(val) #val
#define STRINGIFY(val)  STRINGIFY_(val)
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
//#define SET_BIT(W, B)  ((W) |= (uint32_t)(1U << (B)))
#define CLR_BIT(W, B) ((W) &= (~(uint32_t)(1U << (B))))
#define IS_SET(W, B) (((W) >> (B)) & 1)

#define BIT_0 0x01 /**< The value of bit 0 */
#define BIT_1 0x02 /**< The value of bit 1 */
#define BIT_2 0x04 /**< The value of bit 2 */
#define BIT_3 0x08 /**< The value of bit 3 */
#define BIT_4 0x10 /**< The value of bit 4 */
#define BIT_5 0x20 /**< The value of bit 5 */
#define BIT_6 0x40 /**< The value of bit 6 */
#define BIT_7 0x80 /**< The value of bit 7 */
#define BIT_8 0x0100 /**< The value of bit 8 */
#define BIT_9 0x0200 /**< The value of bit 9 */
#define BIT_10 0x0400 /**< The value of bit 10 */
#define BIT_11 0x0800 /**< The value of bit 11 */
#define BIT_12 0x1000 /**< The value of bit 12 */
#define BIT_13 0x2000 /**< The value of bit 13 */
#define BIT_14 0x4000 /**< The value of bit 14 */
#define BIT_15 0x8000 /**< The value of bit 15 */
#define BIT_16 0x00010000 /**< The value of bit 16 */
#define BIT_17 0x00020000 /**< The value of bit 17 */
#define BIT_18 0x00040000 /**< The value of bit 18 */
#define BIT_19 0x00080000 /**< The value of bit 19 */
#define BIT_20 0x00100000 /**< The value of bit 20 */
#define BIT_21 0x00200000 /**< The value of bit 21 */
#define BIT_22 0x00400000 /**< The value of bit 22 */
#define BIT_23 0x00800000 /**< The value of bit 23 */
#define BIT_24 0x01000000 /**< The value of bit 24 */
#define BIT_25 0x02000000 /**< The value of bit 25 */
#define BIT_26 0x04000000 /**< The value of bit 26 */
#define BIT_27 0x08000000 /**< The value of bit 27 */
#define BIT_28 0x10000000 /**< The value of bit 28 */
#define BIT_29 0x20000000 /**< The value of bit 29 */
#define BIT_30 0x40000000 /**< The value of bit 30 */
#define BIT_31 0x80000000 /**< The value of bit 31 */

#define UNUSED_VARIABLE(X)  ((void)(X))
#define UNUSED_PARAMETER(X) UNUSED_VARIABLE(X)
#define UNUSED_RETURN_VALUE(X) UNUSED_VARIABLE(X)


#endif // NORDIC_COMMON_H__
