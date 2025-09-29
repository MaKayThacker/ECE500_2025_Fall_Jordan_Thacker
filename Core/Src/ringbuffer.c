#include "ringbuffer.h"
#include <stdio.h>

/* Retarget prototype */
int __io_putchar(int ch);

/* Storage */
static char    rb[RING_CAP];
static uint8_t head, tail, count, lines;
static uint8_t last_was_cr;

static inline uint8_t inc(uint8_t x){ return (uint8_t)((x + 1u) % RING_CAP); }
static inline uint8_t dec(uint8_t x){ return (uint8_t)((x + RING_CAP - 1u) % RING_CAP); }

void ring_init(void){
    head = tail = count = lines = last_was_cr = 0u;
    for (uint32_t i=0;i<RING_CAP;i++) rb[i]=0;
}
void ring_clear(void){ ring_init(); }
int  ring_count(void){ return (int)count; }
int  ring_is_empty(void){ return count==0u; }
int  ring_has_line(void){ return lines>0u; }

static void push_byte(char ch){
    if (count == RING_CAP) {
        if (rb[tail] == '\r' && lines) lines--;
        tail = inc(tail); count--;
    }
    rb[head]=ch; head=inc(head); count++;
}

static void echo_bs_one(void){
    __io_putchar('\b'); __io_putchar(' '); __io_putchar('\b');
}

void ring_add(char ch){
    unsigned char u = (unsigned char)ch;

    /* Backspace or DEL */
    if (u == '\b' || u == 0x7Fu){
        last_was_cr = 0u;
        if (count>0u){
            uint8_t prev = dec(head);
            if (rb[prev] != '\r'){ head=prev; rb[head]=0; count--; echo_bs_one(); }
        }
        return;
    }

    /* Normalize line endings */
    if (u == '\n'){
        if (last_was_cr){ last_was_cr=0u; return; } /* swallow LF after CR */
        u = '\r';
    }

    if (u == '\r'){
        push_byte('\r'); lines++; last_was_cr=1u;
        /* keep echo minimal so IRQ path isn’t delayed */
        __io_putchar('\r'); __io_putchar('\n');
        return;
    }

    if (u >= 32u && u <= 126u){
        last_was_cr=0u; push_byte((char)u);
        __io_putchar((int)u);
    }
}

int ring_read_line(char *dst, size_t dstlen){
    if (lines==0u || !dst || dstlen==0u) return 0;
    size_t n=0;
    while (count>0u){
        char c=rb[tail]; tail=inc(tail); count--;
        if (c=='\r'){ lines--; if (n<dstlen) dst[n]='\0'; return (int)n; }
        if (n+1<dstlen) dst[n++]=c;
    }
    lines=0u; if (n<dstlen) dst[n]='\0'; return (int)n;
}
