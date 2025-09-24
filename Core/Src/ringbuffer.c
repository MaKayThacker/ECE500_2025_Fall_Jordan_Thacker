#include "ringbuffer.h"
#include <string.h>
#include <stdio.h>

/* --- Terminal characters ------------------------------------------------ */
#define CR  '\r'
#define LF  '\n'

/* --- Ring buffer state --------------------------------------------------- */
static volatile char     ring_buffer[RING_CAP];
static volatile uint16_t head_index     = 0;
static volatile uint16_t tail_index     = 0;
static volatile uint16_t buffer_count   = 0;
static volatile bool     overflow_flag  = false;

static inline uint16_t next_index(uint16_t idx) { return (uint16_t)((idx + 1) % RING_CAP); }

/* --- Add one character -------------------------------------------------- */
void ring_add(char input_char, bool *did_overflow) {
    if (buffer_count == RING_CAP) {
        tail_index = next_index(tail_index); // overwrite oldest
        overflow_flag = true;
    } else {
        buffer_count++;
    }
    ring_buffer[head_index] = input_char;
    head_index = next_index(head_index);
    if (did_overflow) *did_overflow = overflow_flag;
}

/* --- Handle backspace editing ------------------------------------------- */
void ring_backspace(void) {
    if (!buffer_count) return;
    uint16_t prev_index = (head_index == 0) ? (RING_CAP - 1) : (head_index - 1);
    char last_char = ring_buffer[prev_index];
    if (last_char != CR && last_char != LF) {
        head_index = prev_index;
        buffer_count--;
    }
}

/* --- Detect if a full line exists --------------------------------------- */
bool ring_has_line(void) {
    if (!buffer_count) return false;
    uint16_t i = tail_index;
    for (uint16_t c = 0; c < buffer_count; ++c) {
        char ch = ring_buffer[i];
        if (ch == CR || ch == LF) return true;
        i = next_index(i);
    }
    return false;
}

/* --- Pop a line into dst (strip CR/LF) ---------------------------------- */
int ring_pop_line(char *dst, size_t dst_size) {
    if (!ring_has_line() || !dst || !dst_size) return -1;

    size_t write_len = 0;
    while (buffer_count) {
        char current_char = ring_buffer[tail_index];
        tail_index = next_index(tail_index);
        buffer_count--;

        if (current_char == CR || current_char == LF) {
            if (buffer_count && ring_buffer[tail_index] == LF) {
                tail_index = next_index(tail_index);
                buffer_count--;
            }
            break; // end of line
        }
        if (write_len + 1 < dst_size) dst[write_len++] = current_char;
    }
    dst[write_len] = '\0';
    return (int)write_len;
}

/* --- Debug dump --------------------------------------------------------- */
void ring_dump(void) {
    printf("[ring] cap=%d count=%u head=%u tail=%u overflow=%d\r\n",
           (int)RING_CAP, (unsigned)buffer_count,
           (unsigned)head_index, (unsigned)tail_index, overflow_flag);
}

/* --- Normalize after parse ---------------------------------------------- */
void ring_reset_after_parse(void) {
    if (buffer_count == 0) {
        head_index = tail_index = 0;
        overflow_flag = false;
    }
}

/* --- Clear buffer ------------------------------------------------------- */
void ring_clear(void) {
    head_index = tail_index = buffer_count = 0;
    overflow_flag = false;
}

/* --- Init alias --------------------------------------------------------- */
void ring_init(void) {
    ring_clear();
}
