/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#define SILENT
#include <cpu_core.h>
#include <cstdint>

UnbufferedSerial serial(USBTX, USBRX, 38400);

void serial_transmit_str(const char* str) {
	while (*str) {
	    serial.write(str++, 1);
    }
}

char serial_receive() {
    char res = 0;
    serial.read(&res, 1);
    return res;
}

int serial_receive_until(char* out, char delim) {
    int bytes = 0;
    char buf = 0;

    while ((buf = serial_receive()) != delim) {
        out[bytes++] = buf;
    }

    return bytes;
}

void debugf(const char *fmt, ...) {
	char printf_buf[128] = { 0 };
	va_list args;
	int printed;

	va_start(args, fmt);
	printed = vsprintf(printf_buf, fmt, args);
	va_end(args);

    serial_transmit_str(printf_buf);
    serial_transmit_str("\n\r");
}

uint8_t memory[0xffff] = { 0 };

uint8_t cpu_fetch_byte(uint16_t addr) {
	// debugf("fetching byte at 0x%x", addr);
	return memory[addr];
}

void cpu_write_byte(uint16_t addr, uint8_t val) {
	// debugf("writing byte 0x%x at 0x%x", val, addr);
    memory[addr] = val;
}

PortOut port0_out(PortC, 0b11111111);
PortIn port0_in(PortC, 0b1111111100000000);

uint8_t cpu_io_read(uint16_t addr) {
	debugf("reading byte from io at 0x%x", addr);

    switch (addr) {
        case 0x0:
            return port0_in >> 8;
        default:
            return 0;
    }
}

void cpu_io_write(uint16_t addr, uint8_t val) {
    debugf("writing byte 0x%x to io at 0x%x", val, addr);
    switch (addr) {
        case 0x0:
            port0_out = val;
            break;
    }
}

char* parse_number(char* input, int* output) {
	int idx = 0;
	int number_system_base = 10;

	if (input[0] == '0') {
		if (input[1] == 'x') {
			number_system_base = 16;
			idx = 2;
		} else if (input[1] == 'b') {
			number_system_base = 2;
			idx = 2;
		}
	}

	int _number = 0;

	while (input[idx] != '\0') {
		if (input[idx] >= '0' && input[idx] <= '9') {
			_number = _number * number_system_base + (input[idx] - '0');
		} else if (input[idx] >= 'a' && input[idx] <= 'f') {
			_number = _number * number_system_base + (input[idx] - 'a' + 10);
		} else if (input[idx] >= 'A' && input[idx] <= 'F') {
			_number = _number * number_system_base + (input[idx] - 'A' + 10);
		} else {
			break;
		}

		idx++;
	}

	*output = _number;

	return &input[idx];
}

void programing_mode() {
    serial_transmit_str("READY\n");

    bool running = true;
	while (running) {
        char buf[0xff] = { 0 };
        serial_receive_until(buf, '\n');

        if (strcmp(buf, "PING") == 0) {
        } else if(strncmp(buf, "READ ", 5) == 0) {
            int addr;
            parse_number(&buf[5], &addr);
            char out[32] = { 0 };
            sprintf(out, "%d\n", cpu_fetch_byte(addr));
            serial_transmit_str(out);
            continue;
        } else if(strncmp(buf, "WRITE ", 6) == 0) {
            int addr;
            char* _new = parse_number(&buf[6], &addr);
            int val;
            parse_number(&_new[1], &val);
            cpu_write_byte(addr, val);
        } else if (strcmp(buf, "EXIT") == 0) {
            running = false;
        } else {
            goto error;
        }

        serial_transmit_str("OK\n");
        continue;

    error:
        serial_transmit_str("ERROR\n");
	}
}

int main() {
    programing_mode();
    core_run();
    while (true) {
    }
}
