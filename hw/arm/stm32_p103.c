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
#include <math.h>
#include <time.h>
#include "hw/arm/stm32.h"
#include "hw/sysbus.h"
#include "hw/arm/arm.h"
#include "hw/devices.h"
#include "ui/console.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"

#define PI 3.14159265


/*Global variables*/
int car_direction;
float car_angle;
int irq_count = 0; //we check the PWM duty cycle every 100 irqs
int right_motor_state = 0;  //0=stop 1=forward 2=backward

int right_motor_state_record[101];
int right_motor_time_record[100];

int in1,in2,in3,in4=0;
clock_t right_forward_start,right_forward_end,right_stop_start,right_stop_end,right_backward_start,right_backward_end;
clock_t time_now,time_last;
int in1_count,in2_count,in3_count,in4_count=0;


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


void show_pwm(){

    int count=0;
    char ch1,ch2,ch3;
    ch1 = '-';
    ch2 = '^';
    ch3 = 'v';

    while(count<10){
        if ( right_motor_state_record[count] == 0 )
        {
            printf("%c",ch1);
        }
        else if ( right_motor_state_record[count] == 1 )
        {
            printf("%c",ch2);
        }
        else{
            printf("%c",ch3);
        }
        count++;
    }
    printf("\n");

}


int check_irq_count(){
    if ( irq_count == 99 ){
        show_pwm();
        irq_count = 0;
        return 1;
    }
    else{
        return 0;
    }
}



void clock_count(int state){
    if ( time_last != 0 )
        {
            time_now=clock();
            right_motor_time_record[irq_count-1]= time_now - time_last;

            time_last=clock();
            right_motor_state_record[irq_count] = state;
        }
        else{
            time_last = clock();
            right_motor_state_record[irq_count] = state;
        }
}

void in1_irq_low(){
    if ( in1 == in2 ) //stop
    {
        clock_count(0);
    }
    else{  //backward
        clock_count(2);
    }
}

void in1_irq_high(){
    if ( in1 == in2 ) //stop
    {
        clock_count(0);
    }
    else{  //backward
        clock_count(1);
    }
}

void in2_irq_low(){
    if ( in1 == in2 ) //stop
    {
        clock_count(0);
    }
    else{  //forward
        clock_count(1);
    }
}

void in2_irq_high(){
    if ( in1 == in2 ) //stop
    {
        clock_count(0);
    }
    else{  //backward
        clock_count(2);
    }
}



static void in1_irq_handler(void *opaque, int n, int level)
{
    if(check_irq_count()==1){
    }
    
    else{

        assert(n == 0);

        switch (level) {
            case 0:
                in1=0;
                in1_irq_low();
                break;
            case 1:
                in1=1;
                in1_irq_high();
                break;
        }
    }
    irq_count++;
}

static void in2_irq_handler(void *opaque, int n, int level)
{
    if(check_irq_count()==1){
    }

    else{
        assert(n == 0);
        switch (level) {
            case 0:
                in2=0;
                in2_irq_low();
                break;
            case 1:
                in2=1;
                in2_irq_high();
                break;
        }
    }
    irq_count++;
}

static void in3_irq_handler(void *opaque, int n, int level)
{
    check_irq_count();

    assert(n == 0);
    switch (level) {
        case 0:
            
            break;
        case 1:
            
            break;
    }
}

static void in4_irq_handler(void *opaque, int n, int level)
{
    check_irq_count();

    assert(n == 0);
    switch (level) {
        case 0:
            
            break;
        case 1:
            
            break;
    }
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
