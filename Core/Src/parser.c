#include "parser.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>

/* --- Trim leading and trailing whitespace ------------------------------- */
static void trim_spaces(char *str) {
    while (*str && isspace((unsigned char)*str)) {
        memmove(str, str + 1, strlen(str));
    }
    size_t len = strlen(str);
    while (len && isspace((unsigned char)str[len - 1])) {
        str[--len] = '\0';
    }
}

/* --- Parse command string into code ------------------------------------- */
int parser(const char *input) {
    if (!input || !*input) return CMD_EMPTY;

    char buffer[32];
    size_t len = strnlen(input, sizeof(buffer) - 1);
    memcpy(buffer, input, len);
    buffer[len] = '\0';

    trim_spaces(buffer);
    for (char *p = buffer; *p; ++p) {
        *p = (char)toupper((unsigned char)*p);
    }

    if (strcmp(buffer, "STOP")  == 0) return CMD_STOP;
    if (strcmp(buffer, "START") == 0) return CMD_START;
    if (strcmp(buffer, "CLEAR") == 0) return CMD_CLEAR;
    return CMD_UNDEFINED;
}

/* --- Generate human-readable reply -------------------------------------- */
int parser_reply(int code, char *out, size_t out_size) {
    if (!out || !out_size) return 0;

    switch (code) {
        case CMD_STOP:     return snprintf(out, out_size, "STOP command is received in Tera Term");
        case CMD_START:    return snprintf(out, out_size, "START command is received in Tera Term");
        case CMD_CLEAR:    return snprintf(out, out_size, "CLEAR command is received in Tera Term");
        case CMD_OVERFLOW: return snprintf(out, out_size, "Error: ring buffer overflow");
        case CMD_EMPTY:    return snprintf(out, out_size, "Error: empty command");
        default:           return snprintf(out, out_size, "Error: undefined command");
    }
}
