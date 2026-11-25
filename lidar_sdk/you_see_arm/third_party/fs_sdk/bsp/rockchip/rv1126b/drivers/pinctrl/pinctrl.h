#ifndef __PIN_CTRL_H__
#define __PIN_CTRL_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    // Pin Output
    PIN_OUTPUT_START = 0,
    // PIN_SYNC_EXT = PIN_OUTPUT_START,        //output 帧同步信号
    PIN_MCU_MODE = PIN_OUTPUT_START,
    PIN_MCU_HW_RST,
    PIN_OUTPUT_END = PIN_MCU_HW_RST,

    // Pin Input
    PIN_INPUT_START,
    // PIN_INPUT0 = PIN_INPUT_START,           //input INT
    PIN_INPUT_END = PIN_INPUT_START,// PIN_INPUT0,

    // Pin Irq
    PIN_IRQ_START,
    // PIN_MCU_IRQ = PIN_IRQ_START,           //input INT
    // PIN_IRQ_END = PIN_MCU_IRQ,
    PIN_MAX,
} Pin_t;

int pinctrl_init(void);
void pinctrl_deinit(void);
int pinctrl_set_gpio(uint16_t io, uint8_t v);
int pinctrl_get_gpio(uint16_t io);

int mcu_int_wait_timeout(long timeout_ms);
int mcu_int_wait_forever(void);

#ifdef __cplusplus
}
#endif

#endif //__PIN_CTRL_H__
