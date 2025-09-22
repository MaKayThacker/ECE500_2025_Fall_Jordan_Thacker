#include "ringbuffer.h"
#include <stdio.h>

/* Retarget prototypes from your project (already provided) */
int __io_putchar(int ch);

/* Internal storage (global, fixed size) */
static char     rb[RING_CAP];
static uint8_t  head;     // next write pos
static uint8_t  count;    // number of valid bytes (0..RING_CAP)

/* Small helpers */
static inline uint8_t modulo_dec(uint8_t x, uint8_t m) { return (uint8_t)((x + m - 1u) % m); }
static inline uint8_t modulo_inc(uint8_t x, uint8_t m) { return (uint8_t)((x + 1u) % m); }

void ring_init(void) {
    head  = 0u;
    count = 0u;
}

int ring_count(void) {
    return (int)count;
}

int ring_is_empty(void) {
    return (count == 0u);
}

/* Erase last displayed char on terminal: backspace, space, backspace */
static void echo_erase_one(void) {
    __io_putchar('\b');
    __io_putchar(' ');
    __io_putchar('\b');
}

/* Dump the entire ring with indices (debug view) */
void ring_dump(void) {
    printf("Ring (cap=%d, count=%d):\r\n", RING_CAP, ring_count());
    for (int i = 0; i < RING_CAP; ++i) {
        char c = rb[i];
        if (c == 0) c = '.'; // show empty as '.'
        printf("[%02d] '%c'\r\n", i, c);
    }
}

/* The required function:
   - Stores ASCII chars into ring
   - Wraps/overwrites oldest when full
   - Echoes typed characters in real time
   - Handles Backspace ('\b' or 0x7F): remove last
   - Handles Enter ('\r' or '\n'): newline + dump
*/
void ring_add(char ch) {
    /* Normalize Enter (treat LF as CR) */
    if (ch == '\n') ch = '\r';

    /* Handle Backspace (both BKSP 0x08 and DEL 0x7F) */
    if (ch == '\b' || (unsigned char)ch == 0x7F) {
        if (count > 0u) {
            /* Remove last from ring (the item before head) */
            head = modulo_dec(head, RING_CAP);
            rb[head] = 0;        // optional: clear for nicer dump
            count--;
            /* Erase on Tera Term */
            echo_erase_one();
        }
        return;
    }

    /* Handle Enter: print line, dump, and prompt */
    if (ch == '\r') {
        /* Visual newline on terminal */
        __io_putchar('\r');
        __io_putchar('\n');

        /* Show what’s stored */
        ring_dump();

        /* New prompt */
        printf("\r\n> ");
        return;
    }

    /* Printable range (space..~); ignore others */
    if ((unsigned char)ch >= 32u && (unsigned char)ch <= 126u) {
        /* Overwrite policy:
           - write at head
           - advance head
           - if already full, the oldest is lost (we just reduce net count change)
        */
        rb[head] = ch;
        head = modulo_inc(head, RING_CAP);
        if (count < RING_CAP) {
            count++;
        } else {
            /* Overwrite oldest: nothing extra to do since we don’t track tail,
               we just keep the last RING_CAP characters by moving head forward.
               The “lost” char is intentionally discarded per spec. */
        }

        /* Echo the character back in real time */
        __io_putchar(ch);
    }
}
