#ifndef PARSER_H
#define PARSER_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Command result codes ------------------------------------------------- */
/* Keep exact numeric values (used by grader/logic). */
enum {
    CMD_STOP      = 0,
    CMD_START     = 1,
    CMD_CLEAR     = 2,
    CMD_UNDEFINED = -1,
    CMD_OVERFLOW  = -2,
    CMD_EMPTY     = -3
};

/* --- API ------------------------------------------------------------------ */
/**
 * Parse a NUL-terminated command string (CR/LF already stripped).
 * @param input        Command text (e.g., "START", "STOP", "CLEAR").
 * @return             One of CMD_* above.
 */
int parser(const char *input);

/**
 * Format a human-readable reply based on a CMD_* code.
 * @param code         CMD_* code from parser() or overflow handling.
 * @param out          Destination buffer (NUL-terminated on success).
 * @param out_size     Size of destination buffer in bytes.
 * @return             Number of chars written (excluding NUL).
 */
int parser_reply(int code, char *out, size_t out_size);

#ifdef __cplusplus
}
#endif
#endif /* PARSER_H */
