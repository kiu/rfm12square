#include "rfm12square.h"

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "lib/global.h"
#include "lib/i2c/i2cmaster.h"
#include "lib/rfm12/rfm12.h"

struct LED {
    uint8_t i2c_pwm;
    uint8_t i2c_mode;

    uint32_t color;
    uint32_t color_last;

    uint8_t blink;
    uint8_t blink_last;

    uint32_t timeout;
};

struct LED led1;
struct LED led2;
struct LED led3;
struct LED led4;

void send_led(struct LED* p_led) {
    if (p_led->color != p_led->color_last) {
	p_led->color_last = p_led->color;

	uint32_t rgb = p_led->color;
	uint8_t r = rgb >> 16;
	uint8_t g = rgb >> 8;
	uint8_t b = rgb >> 0;

	i2c_start(DEVPWM+I2C_WRITE);
	i2c_write(p_led->i2c_pwm | 0b10000000);
	i2c_write(g);
	i2c_write(b);
	i2c_write(r);
	i2c_stop();
    }

    if (p_led->blink != p_led->blink_last) {
	p_led->blink_last = p_led->blink;
	i2c_start(DEVPWM+I2C_WRITE);
	i2c_write(p_led->i2c_mode);
	if (p_led->blink) {
	    i2c_write(0b00111111);
	} else {
	    i2c_write(0b00101010);
	}
	i2c_stop();
    }
}

void update_led(struct LED* p_led, uint8_t blink, uint8_t r, uint8_t g, uint8_t b) {
    uint32_t color = 0;
    color |= (uint32_t)r << 16;
    color |= (uint32_t)g << 8;
    color |= (uint32_t)b << 0;

    p_led->timeout = 0;
    p_led->color = color;
    p_led->blink = blink;

    send_led(p_led);
}

void data_received(unsigned char data[], uint8_t size) {

    if (size != 6) {
	return;
    }

    unsigned char adr = data[0];
    unsigned char led = data[1];
    unsigned char blnk = data[2];
    unsigned char r = data[3];
    unsigned char g = data[4];
    unsigned char b = data[5];

    if (adr != 0 && adr != ADDRESS) {
	return;
    }

    if (led > 4) {
	return;
    }

    if (led == 0 || led == 1) {
	update_led(&led1, blnk, r, g, b);
    }
    if (led == 0 || led == 2) {
	update_led(&led2, blnk, r, g, b);
    }
    if (led == 0 || led == 3) {
	update_led(&led3, blnk, r, g, b);
    }
    if (led == 0 || led == 4) {
	update_led(&led4, blnk, r, g, b);
    }

}

void data_receive(void) {
    unsigned char tmp[RFM12_DATA_LENGTH];
    uint8_t state = 0;

    state = 255;
    while (state != 0) {
	state = rfm12_rxstart();
	if (state != 0) {
	    rfm12_allstop();
	}
    }

    state = 255;
    while (state == 255) {
	state = rfm12_rxfinish(tmp);
	if (state == 0) { // CRC error
	    return;
	}
	if (state == 254) { // Old buffer
	    return;
	}
    }

    data_received(tmp, state);
}

void timeout_check(struct LED* p_led) {
    p_led->timeout++;
    if (p_led->timeout >= TIMEOUT_SECONDS) {
	p_led->timeout = 0;
	p_led->color = TIMEOUT_COLOR;
	p_led->blink = TIMEOUT_BLINK;
	send_led(p_led);
    }
}

ISR(TIMER1_COMPA_vect) { // 999ms
    timeout_check(&led1);
    timeout_check(&led2);
    timeout_check(&led3);
    timeout_check(&led4);
}

int main (void) {

#ifdef ORIENTATION_USB_LEFT
    led1.i2c_pwm  = 0b00001010;
    led1.i2c_mode = 0b00010110;
    led2.i2c_pwm  = 0b00000110;
    led2.i2c_mode = 0b00010101;
    led3.i2c_pwm  = 0b00000010;
    led3.i2c_mode = 0b00010100;
    led4.i2c_pwm  = 0b00001110;
    led4.i2c_mode = 0b00010111;
#endif
#ifdef ORIENTATION_USB_TOP
    led1.i2c_pwm  = 0b00001110;
    led1.i2c_mode = 0b00010111;
    led2.i2c_pwm  = 0b00001010;
    led2.i2c_mode = 0b00010110;
    led3.i2c_pwm  = 0b00000110;
    led3.i2c_mode = 0b00010101;
    led4.i2c_pwm  = 0b00000010;
    led4.i2c_mode = 0b00010100;
#endif
#ifdef ORIENTATION_USB_RIGHT
    led1.i2c_pwm  = 0b00000010;
    led1.i2c_mode = 0b00010100;
    led2.i2c_pwm  = 0b00001110;
    led2.i2c_mode = 0b00010111;
    led3.i2c_pwm  = 0b00001010;
    led3.i2c_mode = 0b00010110;
    led4.i2c_pwm  = 0b00000110;
    led4.i2c_mode = 0b00010101;
#endif
#ifdef ORIENTATION_USB_BOTTOM
    led1.i2c_pwm  = 0b00000110;
    led1.i2c_mode = 0b00010101;
    led2.i2c_pwm  = 0b00000010;
    led2.i2c_mode = 0b00010100;
    led3.i2c_pwm  = 0b00001110;
    led3.i2c_mode = 0b00010111;
    led4.i2c_pwm  = 0b00001010;
    led4.i2c_mode = 0b00010110;
#endif

    sei();

    rfm12_init();

    rfm12_setfreq(RFM12_FREQ(434.32));
    rfm12_setbandwidth(RxBW200, LNA_6, RSSI_79);
    rfm12_setbaud(19200);
    rfm12_setpower(PWRdB_0, TxBW105);

    i2c_init();

    // reset
    i2c_start(0b00000110);
    i2c_write(0xA5);
    i2c_write(0x5A);
    i2c_stop();

    _delay_ms(500);

    // Disable sleep mode, Enable blink
    i2c_start(DEVPWM+I2C_WRITE);
    i2c_write(0b10000000); // Auto increment
    i2c_write(0b00000001); // Defaults + disable sleep mode
    i2c_write(0b00100101); // Defaults + blink
    i2c_stop();

    // Configure
    i2c_start(DEVPWM+I2C_WRITE);
    i2c_write(0b10010010); // Auto increment
    i2c_write(0x7F);	// Group duty cycle
    i2c_write(0x23);	// Group frequency
    i2c_write(0b00101010);
    i2c_write(0b00101010);
    i2c_write(0b00101010);
    i2c_write(0b00101010);
    i2c_stop();

    // Init anim
    update_led(&led1, 0x00, 0xFF, 0x00, 0x00);
    _delay_ms(200);
    update_led(&led1, 0x00, 0x00, 0xFF, 0x00);
    _delay_ms(200);
    update_led(&led1, 0x00, 0x00, 0x00, 0xFF);
    _delay_ms(200);
    update_led(&led1, 0x00, 0x00, 0x00, 0x00);

    _delay_ms(200);

    update_led(&led2, 0x00, 0xFF, 0x00, 0x00);
    _delay_ms(200);
    update_led(&led2, 0x00, 0x00, 0xFF, 0x00);
    _delay_ms(200);
    update_led(&led2, 0x00, 0x00, 0x00, 0xFF);
    _delay_ms(200);
    update_led(&led2, 0x00, 0x00, 0x00, 0x00);

    _delay_ms(200);

    update_led(&led3, 0x00, 0xFF, 0x00, 0x00);
    _delay_ms(200);
    update_led(&led3, 0x00, 0x00, 0xFF, 0x00);
    _delay_ms(200);
    update_led(&led3, 0x00, 0x00, 0x00, 0xFF);
    _delay_ms(200);
    update_led(&led3, 0x00, 0x00, 0x00, 0x00);

    _delay_ms(200);

    update_led(&led4, 0x00, 0xFF, 0x00, 0x00);
    _delay_ms(200);
    update_led(&led4, 0x00, 0x00, 0xFF, 0x00);
    _delay_ms(200);
    update_led(&led4, 0x00, 0x00, 0x00, 0xFF);
    _delay_ms(200);
    update_led(&led4, 0x00, 0x00, 0x00, 0x00);

    _delay_ms(200);


    led1.timeout = 0xFFFFFFFF - 1;
    led2.timeout = 0xFFFFFFFF - 1;
    led3.timeout = 0xFFFFFFFF - 1;
    led4.timeout = 0xFFFFFFFF - 1;

    // Init Timer1B
    TCCR1B = (1<<WGM12)|(1<<CS12)|(1<<CS10); // CTC /1024 128us
    OCR1A = 7812; // 128us * 3916 = 999ms
    TIMSK |= (1<<OCIE1A); // Enable Output Compare B Interrupt

    while(1) {
	data_receive();
    }

}
