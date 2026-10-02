"""Compile the actual SWD bit primitives against GPIO stubs with the ESP toolchain.

This checks C types/undefined shifts without requiring ESP-IDF or a connected target.
The generated assembly/object is temporary; electrical timing needs hardware tests.
"""
from pathlib import Path
import subprocess
import sys

source = Path('main/swd.c').read_text()
header = Path('include/swd.h').read_text()
types = header[header.index('typedef struct {'):header.index('} AppFSM;') + len('} AppFSM;')]
prefix = '''
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef void *SemaphoreHandle_t;
typedef int gpio_num_t;
typedef int gpio_mode_t;
typedef int gpio_pull_mode_t;
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_OUTPUT_OD,
    GPIO_FLOATING, GPIO_PULLUP_ONLY, GPIO_PULLDOWN_ONLY };
void gpio_set_direction(int, int);
void gpio_set_pull_mode(int, int);
void gpio_set_level(int, int);
int gpio_get_level(int);
void esp_rom_delay_us(uint32_t);
#define COUNT(x) (sizeof(x) / sizeof((x)[0]))
#define LOG(...) ((void)0)
'''
primitives = source[source.index('static void swd_configure_pins('):source.index('void swd_get_active_pins(')]
rdbuff = source[source.index('static uint8_t swd_read_rdbuff('):source.index('static uint8_t swd_read_ap(')]
detect = source[source.index('static uint32_t swd_detect('):source.index('static void swd_scan(')]
helpers = '''
static bool has_multiple_bits(uint32_t x) { return (x & (x - 1)) != 0; }
static uint8_t get_bit_num(uint32_t x) { return __builtin_ctz(x); }
'''
subprocess.run([sys.argv[1], '-std=c11', '-Wall', '-Wextra',
                '-Werror=implicit-function-declaration', '-Werror=shift-overflow',
                '-fsyntax-only', '-x', 'c', '-'],
               input=prefix + types + 'void swd_line_reset(AppFSM *);\n' + helpers + primitives + rdbuff + detect,
               text=True, check=True)
print('Firmware bit primitives and detection compile successfully.')
