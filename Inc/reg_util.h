/*
 * reg_util.h
 *
 */

#ifndef REG_UTIL_H_
#define REG_UTIL_H_

#include <stdint.h>

/* Register bit manipulation macros */
#define REG_WRITE(reg, val)   					((reg) = (val))
#define REG_READ(reg)         					((reg))
#define REG_SET_BIT(reg,pos)   					((reg) |=  (1U << (pos)))
#define REG_CLR_BIT(reg,pos)    				((reg) &= ~(1U << (pos)))
#define REG_READ_BIT(reg,pos)    				((reg) &   (1U << (pos)))
#define REG_CLR_VAL(reg,clrmask,pos)   			((reg) &= ~((clrmask) << (pos)))
// To set a new value we have to use a mask to clear the bits and after write the new bits
// The mask value is the number of bits to be written set to 1
#define REG_SET_VAL(reg,val,setmask,pos) 		 do {\
                                                  REG_CLR_VAL(reg,setmask,pos);\
                                                  ((reg) |= ((val) << (pos))); \
                                                }while(0)

#define REG_READ_VAL(reg,rdmask,pos)           	((REG_READ(reg) >> (pos)) & (rdmask))


#define 	GPIO_Pin_0   ((uint16_t)0x0001)
#define 	GPIO_Pin_1   ((uint16_t)0x0002)
#define 	GPIO_Pin_2   ((uint16_t)0x0004)
#define 	GPIO_Pin_3   ((uint16_t)0x0008)
#define 	GPIO_Pin_4   ((uint16_t)0x0010)
#define 	GPIO_Pin_5   ((uint16_t)0x0020)
#define 	GPIO_Pin_6   ((uint16_t)0x0040)
#define 	GPIO_Pin_7   ((uint16_t)0x0080)
#define 	GPIO_Pin_8   ((uint16_t)0x0100)
#define 	GPIO_Pin_9   ((uint16_t)0x0200)
#define 	GPIO_Pin_10  ((uint16_t)0x0400)
#define 	GPIO_Pin_11  ((uint16_t)0x0800)
#define 	GPIO_Pin_12  ((uint16_t)0x1000)
#define 	GPIO_Pin_13  ((uint16_t)0x2000)
#define 	GPIO_Pin_14  ((uint16_t)0x4000)
#define 	GPIO_Pin_15  ((uint16_t)0x8000)
#define 	GPIO_Pin_All ((uint16_t)0xFFFF)

#endif /* REG_UTIL_H_ */
