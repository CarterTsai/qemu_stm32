/*
 * Olimex STM32 P103 Development Board
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Implementation based on
 * Olimex "STM-P103 Development Board Users Manual Rev. A, April 2008"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw/arm/stm32.h"
#include "hw/sysbus.h"
#include "hw/arm/arm.h"
#include "hw/devices.h"
#include "ui/console.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"


typedef struct {
    Stm32 *stm32;

    bool last_button_pressed;
    qemu_irq button_irq;
} Stm32P103;




static void led_irq_handler(void *opaque, int n, int level)
{
    /* There should only be one IRQ for the LED */
    assert(n == 0);

    /* Assume that the IRQ is only triggered if the LED has changed state.
     * If this is not correct, we may get multiple LED Offs or Ons in a row.
     */
    switch (level) {
        case 0:
            printf("LED Off\n");
            break;
        case 1:
            printf("LED On\n");
            break;
    }
}

static void in1_irq_handler(void *opaque, int n, int level)
{
    assert(n == 0);

    switch (level) {
        case 0:
            car_emulator(0,0,0,0);
            break;
        case 1:
            car_emulator(1,0,0,0);
            break;
    }
}

static void in2_irq_handler(void *opaque, int n, int level)
{
    assert(n == 0);
    switch (level) {
        case 0:
            car_emulator(0,0,0,0);
            break;
        case 1:
            car_emulator(0,1,0,0);
            break;
    }
}

static void in3_irq_handler(void *opaque, int n, int level)
{
    assert(n == 0);
    switch (level) {
        case 0:
            car_emulator(0,0,0,0);
            break;
        case 1:
            car_emulator(0,0,1,0);
            break;
    }
}

static void in4_irq_handler(void *opaque, int n, int level)
{
    assert(n == 0);
    switch (level) {
        case 0:
            car_emulator(0,0,0,0);
            break;
        case 1:
            car_emulator(0,0,0,1);
            break;
    }
}


int in1_count = 0;
int in2_count = 0;
int in3_count = 0;
int in4_count = 0;

void car_emulator(int in1,int in2,int in3,int in4){

    in1_count += in1;
    in2_count += in2;
    in3_count += in3;
    in4_count += in4;
    
    printf("in1 count: %d  ",in1);
    printf("in2 count: %d  ",in2);
    printf("in3 count: %d  ",in3);
    printf("in4 count: %d  \r",in4);

}



static void stm32_p103_key_event(void *opaque, int keycode)
{
    Stm32P103 *s = (Stm32P103 *)opaque;
    bool make;
    int core_keycode;

    if((keycode & 0x80) == 0) {
        make = true;
        core_keycode = keycode;
    } else {
        make = false;
        core_keycode = keycode & 0x7f;
    }

    /* Responds when a "B" key press is received.
     * Inside the monitor, you can type "sendkey b"
     */
    if(core_keycode == 0x30) {
        if(make) {
            if(!s->last_button_pressed) {
                qemu_irq_raise(s->button_irq);
                s->last_button_pressed = true;
            }
        } else {
            if(s->last_button_pressed) {
                qemu_irq_lower(s->button_irq);
                s->last_button_pressed = false;
            }
        }
    }
    return;

}


static void stm32_p103_init(QEMUMachineInitArgs *args)
{
    const char* kernel_filename = args->kernel_filename;
    qemu_irq *led_irq;
    qemu_irq *in1_irq;
    qemu_irq *in2_irq;
    qemu_irq *in3_irq;
    qemu_irq *in4_irq;
    Stm32P103 *s;

    s = (Stm32P103 *)g_malloc0(sizeof(Stm32P103));

    stm32_init(/*flash_size*/0x0001ffff,
               /*ram_size*/0x00004fff,
               kernel_filename,
               8000000,
               32768);

    DeviceState *gpio_a = DEVICE(object_resolve_path("/machine/stm32/gpio[a]", NULL));
    DeviceState *gpio_c = DEVICE(object_resolve_path("/machine/stm32/gpio[c]", NULL));
    DeviceState *gpio_d = DEVICE(object_resolve_path("/machine/stm32/gpio[d]", NULL));
    DeviceState *uart2 = DEVICE(object_resolve_path("/machine/stm32/uart[2]", NULL));

    assert(gpio_a);
    assert(gpio_c);
    assert(gpio_d);
    assert(uart2);

    /* Connect LED to GPIO C pin 12 */
    led_irq = qemu_allocate_irqs(led_irq_handler, NULL, 1);
    qdev_connect_gpio_out(gpio_c, 12, led_irq[0]);

    /*Connect PD1 to in6  */
    in1_irq = qemu_allocate_irqs(in1_irq_handler, NULL, 1);
    qdev_connect_gpio_out(gpio_d, 6, in1_irq[0]);

    /*Connect PD1 to in8  */
    in2_irq = qemu_allocate_irqs(in2_irq_handler, NULL, 1);
    qdev_connect_gpio_out(gpio_d, 8, in2_irq[0]);

    /*Connect PD1 to in10  */
    in3_irq = qemu_allocate_irqs(in3_irq_handler, NULL, 1);
    qdev_connect_gpio_out(gpio_d, 10, in3_irq[0]);

    /*Connect PD1 to in11  */
    in4_irq = qemu_allocate_irqs(in4_irq_handler, NULL, 1);
    qdev_connect_gpio_out(gpio_d, 11, in4_irq[0]);



    /* Connect button to GPIO A pin 0 */
    s->button_irq = qdev_get_gpio_in(gpio_a, 0);
    qemu_add_kbd_event_handler(stm32_p103_key_event, s);

    /* Connect RS232 to UART */
    stm32_uart_connect(
            (Stm32Uart *)uart2,
            serial_hds[0],
            STM32_USART2_NO_REMAP);
 }

static QEMUMachine stm32_p103_machine = {
    .name = "stm32-p103",
    .desc = "Olimex STM32 p103 Dev Board",
    .init = stm32_p103_init,
    DEFAULT_MACHINE_OPTIONS,
};


static void stm32_p103_machine_init(void)
{
    qemu_register_machine(&stm32_p103_machine);
}

machine_init(stm32_p103_machine_init);
