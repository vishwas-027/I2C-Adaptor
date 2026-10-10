/* UART command-line interface and SMBus command handlers.
 *
 * This module owns the interactive terminal state, command history, and the
 * I2C mutex. It is deliberately separate from the I2C driver: the driver
 * transfers bytes, while this module decides how commands are parsed and
 * presented to a user.
 *
 * There is no RTOS-aware line-editing library here - everything from
 * backspace handling to arrow-key history recall is hand-rolled on top of
 * raw bytes read one at a time from the UART (see console_task() at the
 * bottom of the file). Keep that in mind while reading: what looks like a
 * lot of ceremony around cursor/escape handling is standing in for what a
 * library like readline would normally give you for free. */
#include "../board/system.h"
#include "../drivers/i2c.h"
#include "../drivers/uart.h"
#include "../adapter/adapter_service.h"
#include "console.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* smbus_read always reads a fixed 32-byte response - there's no length
 * argument on the command line, so this has to match whatever the attached
 * NVMe-MI endpoint actually sends back for an unsolicited read. */
#define I2C_RX_LENGTH           32U
/* Longest line the CLI will accept, including the trailing NUL. Sized well
 * above any real command (the longest today is an smbus_write with 32 data
 * bytes) so normal use never hits Console_Execute's ability to search past
 * the end of typed input. */
#define CONSOLE_LINE_LENGTH     192U
/* Number of previous commands kept for Up/Down arrow recall. Small on
 * purpose - each slot costs a full CONSOLE_LINE_LENGTH bytes of static RAM
 * (10 * 192 = 1920 bytes here), which matters on an MCU with no heap use in
 * this module. */
#define CONSOLE_HISTORY_LENGTH  10U
/* Width of one hex-dump row: "OFFS: " (6) + 16 * " XX" (48) + " |" (2) +
 * 16 ASCII chars (16) + "|\r\n" (3) = 75, rounded up with headroom. */
#define SMBUS_DUMP_LINE_LENGTH  80U
/* Fixed-size circular history avoids dynamic memory use in the CLI. Entries
 * are written in arrival order at s_console_history_next and wrap around;
 * see Console_SaveHistory()/Console_GetHistory() for how "most recent" is
 * recovered from these two indices without ever shifting array contents. */
static char s_console_history[CONSOLE_HISTORY_LENGTH][CONSOLE_LINE_LENGTH];
/* How many of the CONSOLE_HISTORY_LENGTH slots actually hold a real command
 * so far. Saturates at CONSOLE_HISTORY_LENGTH once the buffer has wrapped at
 * least once - after that, every new entry overwrites the oldest one. */
static uint8_t s_console_history_count;
/* Index the *next* saved command will be written to (i.e. one past the most
 * recently saved entry, wrapping modulo CONSOLE_HISTORY_LENGTH). */
static uint8_t s_console_history_next;

/* Swap whatever is currently on the terminal's input line for `replacement`,
 * used when the user presses Up/Down to step through history. There is no
 * cursor-positioning escape sequence used here (no "move left N, clear to
 * end of line") - instead the terminal is walked backwards one character at
 * a time. "\b \b" is the classic three-step erase: backspace moves the
 * cursor left over the character, a space overwrites it with blank, and a
 * second backspace moves the cursor back over that blank so the next write
 * lands in the same column. This only works because it's done in a loop for
 * every character currently on the line - a single "\b \b" only erases one
 * column. */
static void Console_ReplaceLine(char *command, size_t *length, const char *replacement)
{
    size_t new_length = 0U;

    /* Erase the displayed line before redrawing an entry from history. */
    while (*length > 0U) {
        uart_puts("\b \b");
        (*length)--;
    }

    /* Copy the history entry into the live command buffer and echo it back
     * to the terminal in the same pass, so the visible line and `command[]`
     * never disagree with each other. Truncates defensively at
     * CONSOLE_LINE_LENGTH - 1 even though history entries are already
     * bounded to that size by Console_SaveHistory(). */
    while ((replacement[new_length] != '\0') &&
           (new_length < (CONSOLE_LINE_LENGTH - 1U))) {
        command[new_length] = replacement[new_length];
        new_length++;
    }
    command[new_length] = '\0';
    *length = new_length;
    uart_write(command, (uint32_t)new_length);
}

/* Append `command` to the ring buffer, called once per Enter keypress
 * (after trimming) regardless of whether the command was recognized - even
 * a typo gets recalled with Up-arrow, matching how a normal shell behaves. */
static void Console_SaveHistory(const char *command)
{
    if (command[0] == '\0') {
        return;
    }

    /* Overwrite the oldest entry once all ten slots have been used. */
    strncpy(s_console_history[s_console_history_next], command,
            CONSOLE_LINE_LENGTH - 1U);
    s_console_history[s_console_history_next][CONSOLE_LINE_LENGTH - 1U] = '\0';

    /* Advance the write cursor, wrapping back to slot 0 after the last one.
     * This single line is the entire "ring" part of the ring buffer - once
     * s_console_history_next wraps around, the next strncpy() above starts
     * silently overwriting the oldest saved command. */
    s_console_history_next = (uint8_t)((s_console_history_next + 1U) %
                                        CONSOLE_HISTORY_LENGTH);
    if (s_console_history_count < CONSOLE_HISTORY_LENGTH) {
        s_console_history_count++;
    }
}

/* Look up a history entry by "how many presses of Up ago", where offset 0
 * is the most recently saved command, offset 1 is the one before that, and
 * so on. Never call this with offset >= s_console_history_count - the
 * caller (console_task's arrow-key handling) is responsible for that bound.
 *
 * The index math walks backwards from s_console_history_next (which always
 * points one slot *past* the most recent entry) and wraps using modulo
 * arithmetic instead of a signed subtraction, because uint8_t index math
 * that goes negative would wrap to a huge unsigned value instead of -1.
 * Adding CONSOLE_HISTORY_LENGTH before subtracting keeps every intermediate
 * value non-negative so the modulo behaves the way you'd expect. */
static const char *Console_GetHistory(uint8_t offset)
{
    uint8_t index = (uint8_t)((s_console_history_next + CONSOLE_HISTORY_LENGTH -
                               1U - offset) % CONSOLE_HISTORY_LENGTH);
    return s_console_history[index];
}

/* Minimal decimal printf-substitute: no libc printf/itoa is pulled in here,
 * intentionally, to keep flash/stack footprint predictable on the MCU.
 * Digits fall out of repeated %10/divide-by-10 in *least-significant-first*
 * order (e.g. 84000000 -> 0,0,0,0,0,0,4,8), so they're stashed into a small
 * array and then walked backwards to print most-significant-first. `digits`
 * is sized for 10 characters because that's the max decimal digit count for
 * any 32-bit value (4294967295). The do/while (rather than while) guarantees
 * at least one digit is emitted, which matters for the value 0 - a
 * pre-tested while loop would print nothing for it. */
static void Console_PrintUnsigned(uint32_t value)
{
    char digits[10];
    uint8_t length = 0U;

    do {
        digits[length++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (length > 0U) {
        uart_putc(digits[--length]);
    }
}

static void Console_PrintHexByte(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    uart_putc(hex[value >> 4U]);
    uart_putc(hex[value & 0x0FU]);
}

/* Prints a static device summary followed by the full command reference.
 * The clock values are read live from SystemClock rather than hardcoded, so
 * this doubles as a quick sanity check that the board's PLL/prescaler setup
 * produced the frequencies you expect - handy after touching clock config,
 * since a wrong PCLK1 here would also mean the I2C driver's CCR/TRISE
 * calculations (which consume SystemClock.pclk1) are wrong too. */
static void Console_PrintHelp(void)
{
    uart_puts("\r\nDevice information\r\n"
              "  MCU: STM32F411CEU6 (ARM Cortex-M4F)\r\n"
              "  SYSCLK: ");
    Console_PrintUnsigned(SystemClock.sysclk / 1000000U);
    uart_puts(" MHz\r\n  HCLK: ");
    Console_PrintUnsigned(SystemClock.hclk / 1000000U);
    uart_puts(" MHz\r\n  PCLK1: ");
    Console_PrintUnsigned(SystemClock.pclk1 / 1000000U);
    uart_puts(" MHz\r\n  UART: USART2, PA2 (TX), PA3 (RX), 115200 8N1\r\n"
              "  SMBus/I2C: I2C1, PB8 (SCL), PB9 (SDA), ");
    Console_PrintUnsigned(adapter_i2c_get_bitrate() / 1000U);
    uart_puts(" kHz\r\n"
              "  LED: PC13, on after the latest successful SMBus transaction\r\n"
              "\r\nCommands\r\n"
              "  help\r\n"
              "      Show this device information and command list.\r\n"
              "  clear\r\n"
              "      Clear the terminal screen.\r\n"
              "  smbus_read 0x6A\r\n"
              "      Read and display the fixed 32-byte device response.\r\n"
              "  smbus_write 0x6A 0x01 0x12 [0x34 ...]\r\n"
              "      Write command 0x01 and 1 to 32 data bytes.\r\n"
              "  i2c_detect\r\n"
              "      Scan the I2C bus and list responding device addresses.\r\n"
              "  i2c_count\r\n"
              "      Scan the I2C bus and report only the device count (no per-address prints).\r\n"
              "  i2c_bitrate <100000|400000>\r\n"
              "      Set the I2C bus bitrate in Hz. The initial value is 100000.\r\n"
              "  i2c_slave_enable <address> <max_tx_bytes> <max_rx_bytes>\r\n"
              "      Enable addressed-slave mode; use 0 for either default maximum.\r\n"
              "  i2c_write <address> 0 [byte ...]\r\n"
              "      Send a raw controller write; flags value 0 means I2C_NO_FLAGS.\r\n"
              "  i2c_write_ext <address> 0 [byte ...]\r\n"
              "      Send a raw write and report extended status and byte count.\r\n"
              "  i2c_read_ext <address> 0 <length>\r\n"
              "      Read raw bytes and report extended status and byte count.\r\n"
              "  i2c_slave_read_ext <buffer_size>\r\n"
              "      Retrieve a queued external-master write (1 to 258 bytes).\r\n"
              "  Ctrl+C\r\n"
              "      Clear the current line, or cancel a running i2c_detect/i2c_count scan.\r\n");
}

/* Translate the low-level I2C driver's result code into the word printed
 * after a failed smbus_read/smbus_write/i2c command. Kept as a switch with
 * an explicit default (rather than a lookup table) so an unhandled new enum
 * value falls back to "error" instead of indexing off the end of an array. */
static const char *I2C_StatusString(I2C_Status_t status)
{
    switch (status) {
    case I2C_OK:               return "OK";
    case I2C_TIMEOUT:          return "timeout";
    case I2C_NACK:             return "no acknowledgement";
    case I2C_BUS_ERROR:        return "bus error";
    case I2C_ARBITRATION_LOST: return "arbitration lost";
    case I2C_BUSY:             return "bus busy";
    case I2C_ERROR:
    default:                   return "error";
    }
}

static const char *I2C_ExtendedStatusString(I2C_ExtendedStatus_t status)
{
    switch (status) {
    case I2C_EXT_STATUS_OK:            return "OK";
    case I2C_EXT_STATUS_BUS_ERROR:     return "BUS_ERROR";
    case I2C_EXT_STATUS_SLA_ACK:       return "SLA_ACK";
    case I2C_EXT_STATUS_SLA_NACK:      return "SLA_NACK";
    case I2C_EXT_STATUS_DATA_NACK:     return "DATA_NACK";
    case I2C_EXT_STATUS_ARB_LOST:      return "ARB_LOST";
    case I2C_EXT_STATUS_BUS_LOCKED:    return "BUS_LOCKED";
    case I2C_EXT_STATUS_LAST_DATA_ACK: return "LAST_DATA_ACK";
    default:                           return "UNKNOWN";
    }
}

/* Same idea as I2C_StatusString(), one layer up the stack: this reports
 * outcomes from the MCTP transport/protocol layer (packet framing, PEC,
 * NACKs surfaced through MCTP, etc.) rather than raw I2C bus conditions -
 * used by the mctp_nvme_idfy / mctp_nvme_glp command handlers. */
/* Handler for `smbus_read <addr>`. There's no command/register byte on this
 * path - it's a plain I2C master read that expects the target to just start
 * clocking out its fixed 32-byte payload as soon as it's addressed (typical
 * of an SMBus device that keeps one "current" block ready, rather than one
 * addressed by register). The output is a classic hex+ASCII dump, the same
 * layout you'd see from `xxd` or `hexdump -C`. */
static void SMBus_PrintRead(uint8_t address)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t received[I2C_RX_LENGTH];
    I2C_Status_t status;

    status = adapter_smbus_read(address, received, sizeof(received));

    if (status != I2C_OK) {
        uart_puts("SMBus read 0x");
        uart_putc(hex[address >> 4]);
        uart_putc(hex[address & 0x0FU]);
        uart_puts(": ");
        uart_puts(I2C_StatusString(status));
        uart_puts("\r\n");
        return;
    }

    uart_puts("SMBus read 0x");
    uart_putc(hex[address >> 4]);
    uart_putc(hex[address & 0x0FU]);
    uart_puts(" (32 bytes)\r\n");

    /* Walk the 32-byte buffer 16 bytes ("one dump row") at a time. Each row
     * is assembled fully into a local `line[]` buffer and written with a
     * single uart_write() rather than many small uart_puts()/uart_putc()
     * calls, so one row appears on the terminal atomically instead of
     * character-by-character. */
    for (uint16_t offset = 0U; offset < I2C_RX_LENGTH; offset += 16U) {
        char line[SMBUS_DUMP_LINE_LENGTH];
        size_t index = 0U;

        /* Row offset as a 4-digit hex label (e.g. "0000:", "0010:"). Only
         * the low nibble of each hex digit is ever non-zero for a 32-byte
         * buffer, but the shifts are written generically so this still
         * works unmodified if I2C_RX_LENGTH ever grows past 256. */
        line[index++] = hex[(offset >> 12) & 0x0FU];
        line[index++] = hex[(offset >> 8) & 0x0FU];
        line[index++] = hex[(offset >> 4) & 0x0FU];
        line[index++] = hex[offset & 0x0FU];
        line[index++] = ':';

        /* Hex byte columns: " XX" per byte, high nibble then low nibble. */
        for (uint16_t column = 0U; column < 16U; column++) {
            uint8_t value = received[offset + column];
            line[index++] = ' ';
            line[index++] = hex[value >> 4];
            line[index++] = hex[value & 0x0FU];
        }

        /* ASCII sidebar: printable bytes (space through tilde, i.e. the
         * printable 7-bit ASCII range) shown as themselves, everything else
         * - control codes, high-bit-set bytes - shown as '.' so raw binary
         * data can't corrupt the terminal display or misalign the row. */
        line[index++] = ' '; line[index++] = '|';
        for (uint16_t column = 0U; column < 16U; column++) {
            uint8_t value = received[offset + column];
            line[index++] = ((value >= (uint8_t)' ') && (value <= (uint8_t)'~'))
                        ? (char)value : '.';
        }
        line[index++] = '|'; line[index++] = '\r'; line[index++] = '\n';
        uart_write(line, (uint32_t)index);
    }
}

/* Handler for `smbus_write <addr> <command> <data...>`. Unlike smbus_read,
 * this follows the standard SMBus Block Write shape: a one-byte command
 * (selects what the write means to the target), an implicit byte count, and
 * 1-32 data bytes - the driver (I2C1_SMBus_BlockWrite) is responsible for
 * inserting the byte-count field on the wire, this function just supplies
 * the command and the raw data. */
static void SMBus_BlockWrite(uint8_t address,
                             uint8_t command,
                             const uint8_t *data,
                             uint8_t length)
{
    static const char hex[] = "0123456789ABCDEF";
    I2C_Status_t status;

    status = adapter_smbus_write(address, command, data, length);

    if (status == I2C_OK) {
        uart_puts("SMBus block write complete: address 0x");
        uart_putc(hex[address >> 4]);
        uart_putc(hex[address & 0x0FU]);
        uart_puts(", command 0x");
        uart_putc(hex[command >> 4]);
        uart_putc(hex[command & 0x0FU]);
        uart_puts(", ");
        uart_putc((char)('0' + (length / 10U)));
        uart_putc((char)('0' + (length % 10U)));
        uart_puts(" byte(s)\r\n");
    } else {
        uart_puts("SMBus block write failed: ");
        uart_puts(I2C_StatusString(status));
        uart_puts("\r\n");
    }
}

/* Non-blocking cooperative cancellation checkpoint: ^C (ASCII ETX, 0x03)
 * pressed while a command is running has nowhere to go on its own, because
 * console_task's uart_getc() isn't being called at all while a command
 * handler owns the CPU - the UART hardware has only a single-byte receive
 * register (DR) with no queue behind it, so without something actively
 * polling for it, a byte typed mid-command just sits there (or gets
 * silently overwritten by the next one) until the command finally returns
 * on its own.
 *
 * This is that active poll, meant to be called between complete,
 * self-contained units of work inside a loop that runs long enough to be
 * worth interrupting - currently only I2C_ScanCore()'s per-address probe
 * loop. It cannot interrupt a single blocking driver call already in
 * flight (e.g. one in-progress I2C1_Master_Receive()) - only the gaps
 * between separate calls, since there's no lower-level hook into those
 * driver routines to check anything mid-transaction.
 *
 * uart_try_getc() drains every byte currently waiting (there's at most one,
 * per the note above) rather than stopping at the first one, so a burst of
 * keystrokes typed during a long scan doesn't leave stale bytes for the
 * next uart_getc() call to pick up out of context once the command
 * returns. This does mean any ordinary character typed while a command is
 * running is silently discarded, not queued - there is nowhere to queue it
 * to, and it's not meaningful input anyway since no line buffer is active
 * while a command owns the CLI. */
/* Shared scan core for both i2c_detect and i2c_count. Every address probe
 * is a real bus transaction (START + address byte + STOP), so the loop's
 * cost is bus time, not print time - quiet=true only removes the per-address
 * uart_puts() calls, it does not skip any addresses. Held under one mutex
 * acquisition so the whole scan runs uninterrupted, EXCEPT that it can now
 * be cut short by Ctrl+C - see Console_CheckCancel() just above and its
 * call at the top of the loop body below. *cancelled reports back to the
 * caller whether that happened, so I2C_Detect()/I2C_Count() can print
 * "Cancelled." instead of a misleading "No devices found"/final count as if
 * the full 0x03-0x77 range had actually been covered.
 *
 * DIAGNOSTIC INSTRUMENTATION (temporary): quiet mode still prints a '.'
 * every 16 probes and the slave-mode ISR entry counters at the end,
 * carried over from root-causing the previous i2c_count hang after
 * smbus_read. Leaving this in until that fix (I2C1_ER_IRQHandler() in
 * i2c.c) is confirmed on real hardware. What each symptom means:
 *   - Dots keep appearing (with real pauses between them) -> the scan is
 *     alive and just slow; the dot count times 16, plus 0x03, tells you
 *     which address it's around when you got tired of waiting.
 *   - Dots stop appearing entirely and nothing follows for a long time ->
 *     genuine stall. Note which address that last dot corresponds to. With
 *     Ctrl+C now wired in, you no longer have to power-cycle to recover
 *     from this - see if ^C actually breaks out of it, which is itself a
 *     useful data point (a true hardware livelock like the fixed ARLO/BERR
 *     case would starve the CPU badly enough that even the cancellation
 *     check below might not run either).
 *   - "er isr entries" far above zero -> the slave-mode error ISR is firing
 *     repeatedly during the scan, which is the fingerprint of an interrupt
 *     storm rather than a simple slow bus.
 *   - "master er isr entries" above zero -> BERR or ARLO fired while a
 *     master transaction (one of these probes) was active - see the fix in
 *     I2C1_ER_IRQHandler() in i2c.c. This used to be uncounted and
 *     unhandled, which is what actually caused the immediate hang.
 *   - "ev isr entries" far above the small handful you'd expect from 117
 *     STOP->SWRST->rearm cycles -> same signal, from the event ISR side. */
static bool Console_CheckCancel(void *context)
{
    char c;
    bool cancel = false;
    (void)context;

    while (uart_try_getc(&c)) {
        if ((uint8_t)c == 0x03U) {
            cancel = true;
        }
    }
    return cancel;
}

static uint8_t I2C_ScanCore(bool quiet, bool *cancelled)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t address_bitmap[16];
    uint8_t found_count;

    (void)adapter_i2c_detect(address_bitmap, &found_count,
                             Console_CheckCancel, NULL, cancelled);

    if (!quiet) {
        for (uint8_t address = 0x03U; address <= 0x77U; address++) {
            if ((address_bitmap[address >> 3U] &
                 (uint8_t)(1U << (address & 7U))) != 0U) {
                uart_puts("Found device at 0x");
                uart_putc(hex[address >> 4]);
                uart_putc(hex[address & 0x0FU]);
                uart_puts("\r\n");
            }
        }
    }

    return found_count;
}

static void I2C_Detect(void)
{
    bool cancelled;

    uart_puts("Scanning I2C bus for devices...\r\n");
    if ((I2C_ScanCore(false, &cancelled) == 0U) && !cancelled) {
        uart_puts("No devices found\r\n");
    }
    if (cancelled) {
        uart_puts("Cancelled.\r\n");
    }
}

static void I2C_Count(void)
{
    bool cancelled;
    uint8_t found_count = I2C_ScanCore(true, &cancelled);

    if (cancelled) {
        uart_puts("Cancelled.\r\n");
        return;
    }

    uart_puts("Devices found: ");
    uart_putc((char)('0' + (found_count / 10U)));
    uart_putc((char)('0' + (found_count % 10U)));
    uart_puts("\r\n");
}

/* Pull one whitespace-separated numeric argument off the front of *cursor,
 * advancing *cursor past it (and any trailing whitespace) on success. Used
 * in a loop by every command that takes multiple numeric arguments
 * (smbus_write's data bytes, mctp_nvme_idfy/glp's up-to-four values), which
 * is why it takes/returns a cursor pointer rather than being a one-shot
 * parse of the whole line.
 *
 * Returns false for: an empty/whitespace-only remainder (no more arguments
 * to read), a token strtoul() couldn't parse as a number at all (end stayed
 * at *cursor), or a token that parsed a number but had leftover non-numeric,
 * non-whitespace junk stuck to it (e.g. "0x6Ax" - strtoul happily reads
 * "0x6A" and leaves *end pointing at the 'x', which the whitespace/NUL check
 * below catches). Every caller treats a false return as "bad command,
 * print Usage: ..." rather than silently ignoring it. */
static bool Console_ReadValue(const char **cursor, unsigned long *value)
{
    char *end;

    while ((**cursor == ' ') || (**cursor == '\t')) {
        (*cursor)++;
    }
    if (**cursor == '\0') {
        return false;
    }

    /* Base zero accepts the usual terminal forms: decimal, 0x hex, or octal. */
    *value = strtoul(*cursor, &end, 0);
    if ((end == *cursor) || ((*end != '\0') && (*end != ' ') && (*end != '\t'))) {
        return false;
    }
    while ((*end == ' ') || (*end == '\t')) {
        end++;
    }
    *cursor = end;
    return true;
}

/* Normalize the command once at Enter: whitespace inside arguments remains
 * unchanged, while leading/trailing spaces or tabs cannot affect matching. */
static void Console_TrimCommand(char *command)
{
    char *first = command;
    char *last;

    while ((*first == ' ') || (*first == '\t')) {
        first++;
    }

    last = first + strlen(first);
    while ((last > first) && ((last[-1] == ' ') || (last[-1] == '\t'))) {
        last--;
    }
    *last = '\0';

    /* Slide the trimmed text down to the start of the buffer only if leading
     * whitespace was actually skipped - memmove() (not memcpy) because the
     * source and destination ranges can legitimately overlap when `first`
     * is only a few bytes into `command`. The +1 includes the NUL just
     * written above so the copy carries the terminator with it. */
    if (first != command) {
        memmove(command, first, (size_t)(last - first) + 1U);
    }
}

/* The command dispatcher: called once per Enter keypress with the trimmed
 * line. There's no dynamic command table here - just a sequence of
 * if-blocks checked in order, each owning its own argument parsing and
 * error messages, and returning as soon as one matches. This is O(n) in the
 * number of commands rather than a hash/table lookup, but n is small (about
 * a dozen commands) and this keeps each command fully self-contained and
 * easy to add/remove without touching a separate registration table.
 *
 * Two matching styles are used, depending on whether the command takes
 * arguments:
 *   - Zero-argument commands ("help", "clear") use a plain strcmp() against
 *     the whole trimmed line - the entire input must equal the command
 *     name exactly.
 *   - Argument-taking commands use strncmp() against just the command-name
 *     prefix, PLUS an explicit check of the character right after that
 *     prefix. That second check is what stops "smbus_read" from also
 *     matching a typo like "smbus_reader" - it requires the character
 *     immediately following the prefix to be a space/tab (there must be
 *     an argument coming) or, for commands where arguments are optional
 *     (i2c_detect, i2c_count, mctp_nvme_idfy/glp), also allows '\0' (the
 *     command with no arguments at all). Without that boundary check,
 *     strncmp() alone would treat any string that merely *starts with* the
 *     right prefix as a match. */
static void Console_Execute(const char *command)
{
    const char *argument;
    unsigned long address;

    if (command[0] == '\0') {
        return;
    }

    if (strcmp(command, "help") == 0) {
        Console_PrintHelp();
        return;
    }

    if (strcmp(command, "clear") == 0) {
        /* ANSI ED(2) clears the display; CUP(1,1) moves the cursor home. */
        uart_puts("\033[2J\033[H");
        return;
    }

    /* Commands are parsed explicitly instead of using scanf to keep stack
     * and code size predictable on the MCU. smbus_read/smbus_write always
     * require an argument, so their boundary check only accepts a space or
     * tab right after the prefix (never '\0') - "smbus_read" with nothing
     * after it falls through to "Unknown command" rather than being treated
     * as a no-argument form. */
    if ((strncmp(command, "smbus_read", sizeof("smbus_read") - 1) == 0) &&
        ((command[sizeof("smbus_read") - 1] == ' ') || (command[sizeof("smbus_read") - 1] == '\t'))) {
        argument = command + (sizeof("smbus_read") - 1);
        if (!Console_ReadValue(&argument, &address) || (*argument != '\0') ||
            (address > 0x7FU)) {
            uart_puts("Usage: smbus_read 0x6A\r\n");
            return;
        }
        SMBus_PrintRead((uint8_t)address);
        return;
    }

    if ((strncmp(command, "smbus_write", sizeof("smbus_write") - 1) == 0) &&
        ((command[sizeof("smbus_write") - 1] == ' ') || (command[sizeof("smbus_write") - 1] == '\t'))) {
        uint8_t data[I2C1_SMBUS_BLOCK_MAX_LENGTH];
        uint8_t length = 0U;
        unsigned long value;
        unsigned long smbus_command;

        argument = command + (sizeof("smbus_write") - 1);
        if (!Console_ReadValue(&argument, &address) ||
            !Console_ReadValue(&argument, &smbus_command) ||
            (address > 0x7FU) || (smbus_command > 0xFFU)) {
            uart_puts("Usage: smbus_write 0x6A 0x01 0x12 [0x34 ...]\r\n");
            return;
        }

        while (*argument != '\0') {
            if (!Console_ReadValue(&argument, &value) || (value > 0xFFU) ||
                (length >= I2C1_SMBUS_BLOCK_MAX_LENGTH)) {
                uart_puts("Block write accepts 1 to 32 data bytes.\r\n");
                return;
            }
            data[length++] = (uint8_t)value;
        }

        if (length == 0U) {
            uart_puts("Block write requires at least one data byte.\r\n");
            return;
        }
        SMBus_BlockWrite((uint8_t)address, (uint8_t)smbus_command, data, length);
        return;
    }

    if ((strncmp(command, "i2c_bitrate", sizeof("i2c_bitrate") - 1) == 0) &&
        ((command[sizeof("i2c_bitrate") - 1] == ' ') ||
         (command[sizeof("i2c_bitrate") - 1] == '\t'))) {
        unsigned long bitrate;
        I2C_Status_t status;

        argument = command + (sizeof("i2c_bitrate") - 1);
        if (!Console_ReadValue(&argument, &bitrate) || (*argument != '\0') ||
            ((bitrate != (unsigned long)I2C_SPEED_STANDARD_100K) &&
             (bitrate != (unsigned long)I2C_SPEED_FAST_400K))) {
            uart_puts("Usage: i2c_bitrate <100000|400000>\r\n");
            return;
        }
        status = adapter_i2c_bitrate((uint32_t)bitrate);
        if (status != I2C_OK) {
            uart_puts("I2C bitrate change failed: ");
            uart_puts(I2C_StatusString(status));
            uart_puts("\r\n");
            return;
        }
        uart_puts("I2C bitrate: ");
        Console_PrintUnsigned(adapter_i2c_get_bitrate());
        uart_puts(" Hz\r\n");
        return;
    }

    if ((strncmp(command, "i2c_slave_enable",
                 sizeof("i2c_slave_enable") - 1) == 0) &&
        ((command[sizeof("i2c_slave_enable") - 1] == ' ') ||
         (command[sizeof("i2c_slave_enable") - 1] == '\t'))) {
        unsigned long max_tx_bytes;
        unsigned long max_rx_bytes;
        uint16_t applied_tx;
        uint16_t applied_rx;
        I2C_Status_t status;

        argument = command + (sizeof("i2c_slave_enable") - 1);
        if (!Console_ReadValue(&argument, &address) ||
            !Console_ReadValue(&argument, &max_tx_bytes) ||
            !Console_ReadValue(&argument, &max_rx_bytes) ||
            (*argument != '\0') || (address > 0x7FU) ||
            (max_tx_bytes > ADAPTER_I2C_SLAVE_DEFAULT_MAX_TX) ||
            (max_rx_bytes > ADAPTER_I2C_SLAVE_DEFAULT_MAX_RX)) {
            uart_puts("Usage: i2c_slave_enable <address> <max_tx_bytes> <max_rx_bytes>\r\n");
            return;
        }
        status = adapter_i2c_slave_enable(
            (uint8_t)address, (uint16_t)max_tx_bytes,
            (uint16_t)max_rx_bytes, &applied_tx, &applied_rx);
        if (status != I2C_OK) {
            uart_puts("I2C slave enable failed: ");
            uart_puts(I2C_StatusString(status));
            uart_puts("\r\n");
            return;
        }
        uart_puts("I2C slave enabled: address=0x");
        Console_PrintHexByte((uint8_t)address);
        uart_puts(" max_tx=");
        Console_PrintUnsigned(applied_tx);
        uart_puts(" max_rx=");
        Console_PrintUnsigned(applied_rx);
        uart_puts("\r\n");
        return;
    }

    if ((strncmp(command, "i2c_write", sizeof("i2c_write") - 1) == 0) &&
        ((command[sizeof("i2c_write") - 1] == ' ') ||
         (command[sizeof("i2c_write") - 1] == '\t'))) {
        uint8_t data[ADAPTER_I2C_WRITE_MAX_LENGTH];
        uint16_t length = 0U;
        unsigned long flags;
        unsigned long value;
        I2C_Status_t status;

        argument = command + (sizeof("i2c_write") - 1);
        if (!Console_ReadValue(&argument, &address) ||
            !Console_ReadValue(&argument, &flags) ||
            (address > 0x7FU) || (flags != ADAPTER_I2C_NO_FLAGS)) {
            uart_puts("Usage: i2c_write <address> 0 [byte ...]\r\n");
            return;
        }
        while (*argument != '\0') {
            if (!Console_ReadValue(&argument, &value) || (value > 0xFFU) ||
                (length >= ADAPTER_I2C_WRITE_MAX_LENGTH)) {
                uart_puts("Invalid I2C write data.\r\n");
                return;
            }
            data[length++] = (uint8_t)value;
        }
        status = adapter_i2c_write((uint8_t)address, (uint16_t)flags,
                                   data, length);
        if (status != I2C_OK) {
            uart_puts("I2C write failed: ");
            uart_puts(I2C_StatusString(status));
            uart_puts("\r\n");
            return;
        }
        uart_puts("I2C write: ");
        Console_PrintUnsigned(length);
        uart_puts(" byte(s) transmitted\r\n");
        return;
    }

    if ((strncmp(command, "i2c_write_ext",
                 sizeof("i2c_write_ext") - 1) == 0) &&
        ((command[sizeof("i2c_write_ext") - 1] == ' ') ||
         (command[sizeof("i2c_write_ext") - 1] == '\t'))) {
        uint8_t data[ADAPTER_I2C_WRITE_MAX_LENGTH];
        uint16_t length = 0U;
        uint16_t num_written;
        unsigned long flags;
        unsigned long value;
        I2C_ExtendedStatus_t status;

        argument = command + (sizeof("i2c_write_ext") - 1);
        if (!Console_ReadValue(&argument, &address) ||
            !Console_ReadValue(&argument, &flags) ||
            (address > 0x7FU) || (flags != ADAPTER_I2C_NO_FLAGS)) {
            uart_puts("Usage: i2c_write_ext <address> 0 [byte ...]\r\n");
            return;
        }
        while (*argument != '\0') {
            if (!Console_ReadValue(&argument, &value) || (value > 0xFFU) ||
                (length >= ADAPTER_I2C_WRITE_MAX_LENGTH)) {
                uart_puts("Invalid I2C write data.\r\n");
                return;
            }
            data[length++] = (uint8_t)value;
        }
        status = adapter_i2c_write_ext(
            (uint8_t)address, (uint16_t)flags, data, length, &num_written);
        uart_puts("I2C write ext: status=");
        Console_PrintUnsigned((uint32_t)status);
        uart_puts(" (");
        uart_puts(I2C_ExtendedStatusString(status));
        uart_puts(") num_written=");
        Console_PrintUnsigned(num_written);
        uart_puts(" requested=");
        Console_PrintUnsigned(length);
        uart_puts("\r\n");
        return;
    }

    if ((strncmp(command, "i2c_read_ext",
                 sizeof("i2c_read_ext") - 1) == 0) &&
        ((command[sizeof("i2c_read_ext") - 1] == ' ') ||
         (command[sizeof("i2c_read_ext") - 1] == '\t'))) {
        uint8_t data[ADAPTER_I2C_READ_MAX_LENGTH];
        uint16_t num_read;
        unsigned long flags;
        unsigned long length;
        I2C_ExtendedStatus_t status;

        argument = command + (sizeof("i2c_read_ext") - 1);
        if (!Console_ReadValue(&argument, &address) ||
            !Console_ReadValue(&argument, &flags) ||
            !Console_ReadValue(&argument, &length) ||
            (*argument != '\0') || (address > 0x7FU) ||
            (flags != ADAPTER_I2C_NO_FLAGS) || (length == 0U) ||
            (length > ADAPTER_I2C_READ_MAX_LENGTH)) {
            uart_puts("Usage: i2c_read_ext <address> 0 <length>\r\n");
            return;
        }
        status = adapter_i2c_read_ext(
            (uint8_t)address, (uint16_t)flags, data, (uint16_t)length,
            &num_read);
        uart_puts("I2C read ext: status=");
        Console_PrintUnsigned((uint32_t)status);
        uart_puts(" (");
        uart_puts(I2C_ExtendedStatusString(status));
        uart_puts(") num_read=");
        Console_PrintUnsigned(num_read);
        uart_puts(" requested=");
        Console_PrintUnsigned((uint32_t)length);
        uart_puts("\r\n");
        if (num_read > 0U) {
            uart_puts("Data:");
            for (uint16_t index = 0U; index < num_read; index++) {
                uart_puts(" 0x");
                Console_PrintHexByte(data[index]);
            }
            uart_puts("\r\n");
        }
        return;
    }

    if ((strncmp(command, "i2c_slave_read_ext",
                 sizeof("i2c_slave_read_ext") - 1) == 0) &&
        ((command[sizeof("i2c_slave_read_ext") - 1] == ' ') ||
         (command[sizeof("i2c_slave_read_ext") - 1] == '\t'))) {
        uint8_t data[ADAPTER_I2C_SLAVE_DEFAULT_MAX_RX];
        uint8_t received_address;
        uint16_t num_read;
        unsigned long buffer_size;
        I2C_ExtendedStatus_t transaction_status;
        I2C_Status_t status;

        argument = command + (sizeof("i2c_slave_read_ext") - 1);
        if (!Console_ReadValue(&argument, &buffer_size) ||
            (*argument != '\0') || (buffer_size == 0U) ||
            (buffer_size > ADAPTER_I2C_SLAVE_DEFAULT_MAX_RX)) {
            uart_puts("Usage: i2c_slave_read_ext <buffer_size 1..258>\r\n");
            return;
        }
        status = adapter_i2c_slave_read_ext(
            (uint16_t)buffer_size, &received_address, data, &num_read,
            &transaction_status);
        if (status != I2C_OK) {
            uart_puts("I2C slave read ext failed: ");
            uart_puts(I2C_StatusString(status));
            uart_puts("\r\n");
            return;
        }
        uart_puts("I2C slave read ext: status=");
        Console_PrintUnsigned((uint32_t)transaction_status);
        uart_puts(" (");
        uart_puts(I2C_ExtendedStatusString(transaction_status));
        uart_puts(") address=0x");
        Console_PrintHexByte(received_address);
        uart_puts(" num_read=");
        Console_PrintUnsigned(num_read);
        uart_puts("\r\n");
        if (num_read > 0U) {
            uart_puts("Data:");
            for (uint16_t index = 0U; index < num_read; index++) {
                uart_puts(" 0x");
                Console_PrintHexByte(data[index]);
            }
            uart_puts("\r\n");
        }
        return;
    }

    /* i2c_detect and i2c_count both drive I2C_ScanCore() - see that
     * function's comment for what the scan actually does and why i2c_count
     * can't scan any faster than i2c_detect, just print less while doing
     * it. */
    if ((strncmp(command, "i2c_detect", sizeof("i2c_detect") - 1) == 0) &&
        ((command[sizeof("i2c_detect") - 1] == '\0') || (command[sizeof("i2c_detect") - 1] == ' ') || (command[sizeof("i2c_detect") - 1] == '\t'))) {
        I2C_Detect();
        return;
    }

    if ((strncmp(command, "i2c_count", sizeof("i2c_count") - 1) == 0) &&
        ((command[sizeof("i2c_count") - 1] == '\0') || (command[sizeof("i2c_count") - 1] == ' ') || (command[sizeof("i2c_count") - 1] == '\t'))) {
        I2C_Count();
        return;
    }

    /* Reached only when none of the strncmp() blocks above matched -
     * covers both genuinely unknown words and things like "smbus_read"
     * typed with no address (which intentionally does NOT match the
     * smbus_read block above, since that block requires a space/tab right
     * after the prefix). */
    uart_puts("Unknown command. Type help for available commands.\r\n");
}

/* One-time setup, called from application init before vTaskStartScheduler()
 * - i.e. before console_task or any other task is actually running. That
 * ordering is what makes it safe to create the mutex and touch I2C1 here
 * without taking s_i2c_mutex first: there is no concurrent task yet that
 * could be contending for the bus. Everything below runs once and in a
 * fixed order: mutex first (every I2C-touching call after this point
 * assumes it exists), then the I2C1 peripheral itself, then the MCTP/NVMe-MI
 * layers that are built on top of I2C1 and would be meaningless to
 * initialize first. */
void console_init(void)
{
    adapter_service_init();
}

/* Public accessor declared in console.h, backing e.g. the LED task's "on
 * after the latest successful SMBus transaction" behavior described in the
 * help text. Just exposes the volatile flag maintained by every
 * I2C-touching handler above (SMBus_PrintRead, SMBus_BlockWrite,
 * NvmeMi_RunCommand) - there is no separate connection/link concept beyond
 * "did the most recent transaction succeed". */
bool console_smbus_is_connected(void)
{
    return adapter_service_is_connected();
}

/* FreeRTOS task entry point: the interactive line editor and REPL loop.
 * Runs forever, one blocking uart_getc() at a time - there is no timeout
 * and no yielding beyond whatever uart_getc() itself does while waiting for
 * a byte, so this task is expected to block (not spin) between keystrokes.
 *
 * Local state, all scoped to a single "line being typed" plus the minimum
 * needed to support history recall:
 *   command[]         - the line currently on screen / being built.
 *   length             - how many valid characters are in command[] so far.
 *   saved_command[]    - snapshot of command[] taken at the moment Up is
 *                        first pressed, so pressing Down back past the
 *                        oldest recalled entry restores exactly what the
 *                        user had been typing (rather than an empty line).
 *   history_offset     - -1 means "not currently browsing history"; >= 0 is
 *                        an index into Console_GetHistory() (0 = most
 *                        recent). This is int8_t (signed) specifically so
 *                        -1 is representable as a distinct "not browsing"
 *                        sentinel, separate from any valid offset.
 *   escape_state       - tiny state machine for recognizing ANSI arrow-key
 *                        escape sequences spread across 2-3 separate bytes
 *                        (see below).
 *   previous_was_carriage_return - collapses a CRLF line ending (two bytes)
 *                        into a single logical Enter, see below. */
void console_task(void *argument)
{
    char command[CONSOLE_LINE_LENGTH];
    char saved_command[CONSOLE_LINE_LENGTH];
    size_t length = 0U;
    int8_t history_offset = -1;
    uint8_t escape_state = 0U;
    bool previous_was_carriage_return = false;

    (void)argument;

    uart_puts("\r\nSTM32F411 SMBus console ready. Type help\r\n> ");

    while (1) {
        char character = uart_getc();

        /* --- ANSI escape sequence decoding (Up/Down arrow keys) ---
         *
         * A terminal doesn't send one "Up arrow" byte; it sends a 3-byte
         * sequence: ESC (0x1B), then '[', then a letter identifying the
         * key ('A' = Up, 'B' = Down; Left/Right, Home/End etc. use other
         * letters this driver doesn't act on). Because bytes arrive one at
         * a time from uart_getc(), that sequence has to be recognized
         * across three separate passes through this loop - escape_state is
         * a tiny 3-state machine tracking "how far into a suspected escape
         * sequence are we":
         *   0 = not inside a sequence (normal character handling below)
         *   1 = just saw ESC, waiting for '['
         *   2 = just saw ESC '[', waiting for the final letter
         * Any unexpected byte while in state 1 aborts back to state 0
         * (it wasn't the sequence this code cares about); state 2 always
         * returns to 0 after handling the byte, matched sequence or not,
         * since a 3-byte sequence is exactly as long as this code expects
         * regardless of which letter arrives. */
        if (escape_state == 1U) {
            escape_state = (character == '[') ? 2U : 0U;
            continue;
        }
        if (escape_state == 2U) {
            escape_state = 0U;
            /* Up arrow: move one entry further back in history (toward
             * older commands). Guarded by s_console_history_count > 0 so
             * Up does nothing on an empty history rather than trying to
             * read an uninitialized slot. */
            if ((character == 'A') && (s_console_history_count > 0U)) {
                /* The very first Up press (history_offset still -1, i.e.
                 * "not browsing yet") snapshots whatever the user had
                 * already typed into saved_command, so it can be restored
                 * later if they press Down back past the newest history
                 * entry. Subsequent Up presses while already browsing
                 * don't re-snapshot - that would overwrite the original
                 * in-progress line with a history entry. */
                if (history_offset < 0) {
                    strncpy(saved_command, command, sizeof(saved_command));
                    saved_command[sizeof(saved_command) - 1U] = '\0';
                }
                /* Only advance if there's actually an older entry to show -
                 * this is what makes repeated Up presses stop at the
                 * oldest saved command instead of wrapping around or
                 * reading past what's been saved. */
                if ((history_offset + 1) < (int8_t)s_console_history_count) {
                    history_offset++;
                    Console_ReplaceLine(command, &length,
                                        Console_GetHistory((uint8_t)history_offset));
                }
            } else if ((character == 'B') && (history_offset >= 0)) {
                /* Down arrow only does something while already browsing
                 * history (history_offset >= 0) - pressing Down when not
                 * browsing is a no-op, matching typical shell behavior. */
                if (history_offset == 0) {
                    /* Stepping down from the newest history entry leaves
                     * history browsing entirely and restores the line the
                     * user had originally been typing before Up was ever
                     * pressed. */
                    history_offset = -1;
                    Console_ReplaceLine(command, &length, saved_command);
                } else {
                    history_offset--;
                    Console_ReplaceLine(command, &length,
                                        Console_GetHistory((uint8_t)history_offset));
                }
            }
            continue;
        }
        if ((uint8_t)character == 0x1BU) {
            /* Cast to uint8_t before comparing against 0x1B: `char` is not
             * guaranteed signed or unsigned by the C standard, and on a
             * platform where it's signed, comparing a possibly-negative
             * `char` value against a plain hex constant can behave
             * unexpectedly. Every raw byte-value comparison in this
             * function uses the same (uint8_t) cast for that reason. */
            escape_state = 1U;
            continue;
        }

        /* --- Ctrl+C (ASCII ETX, 0x03) ---
         * This is the "nothing is currently running, the user is just
         * typing" half of Ctrl+C support - it's reached here, in the
         * ordinary per-character loop, precisely because no command is
         * executing right now (a running command's own loop would already
         * have consumed this byte itself via Console_CheckCancel(), see
         * I2C_ScanCore()). Matches a normal shell: discard whatever's been
         * typed so far on the current line, echo "^C" so the keystroke is
         * visibly acknowledged, and start a fresh prompt - rather than,
         * say, treating it as a printable character or silently ignoring
         * it. History browsing state is reset the same way Enter resets it
         * below, so a stray Ctrl+C while browsing history doesn't leave
         * history_offset pointing somewhere stale for the next line. */
        if ((uint8_t)character == 0x03U) {
            uart_puts("^C\r\n> ");
            length = 0U;
            history_offset = -1;
            saved_command[0] = '\0';
            escape_state = 0U;
            continue;
        }

        /* --- Line-ending normalization ---
         * Different terminals/settings send "\r", "\n", or "\r\n" for
         * Enter. Treating a lone \r as "line ended" (below) means a
         * follow-on \n from a \r\n pair would otherwise be seen as a
         * second, empty Enter press. previous_was_carriage_return tracks
         * "did the immediately preceding byte end a line via \r" so that
         * exact-following \n can be swallowed as the second half of the
         * same CRLF pair rather than triggering its own empty command. */
        if (character == '\n' && previous_was_carriage_return) {
            previous_was_carriage_return = false;
            continue;
        }
        previous_was_carriage_return = (character == '\r');

        if ((character == '\r') || (character == '\n')) {
            /* Enter: terminate, echo the newline, then run the full
             * trim -> save-to-history -> execute pipeline before resetting
             * all per-line state for the next command. Note history is
             * saved *before* execution, so even a command that errors out
             * (bad address, unknown command, etc.) is still recallable with
             * Up - this console doesn't distinguish "valid" from "invalid"
             * for history purposes, only "non-empty". */
            command[length] = '\0';
            uart_puts("\r\n");
            Console_TrimCommand(command);
            Console_SaveHistory(command);
            Console_Execute(command);
            length = 0U;
            history_offset = -1;
            saved_command[0] = '\0';
            uart_puts("> ");
        } else if ((character == '\b') || ((uint8_t)character == 0x7FU)) {
            /* Backspace: terminals disagree about whether they send 0x08
             * (BS) or 0x7F (DEL) for the Backspace key, so both are treated
             * identically here. Guarded by length > 0 so backspacing on an
             * empty line does nothing rather than corrupting the terminal
             * display or underflowing `length`. */
            if (length > 0U) {
                length--;
                uart_puts("\b \b");
            }
        } else if (((uint8_t)character >= (uint8_t)' ') &&
                   ((uint8_t)character <= (uint8_t)'~')) {
            /* Ordinary printable character (space 0x20 through tilde
             * 0x7E - the full printable ASCII range). Anything outside
             * this range and not already handled above (other control
             * codes, high-bit-set bytes) is silently dropped rather than
             * stored, so garbage bytes can't corrupt the command buffer or
             * desync the displayed line from `command[]`. Bounded by
             * sizeof(command) - 1 to always leave room for the terminating
             * NUL written above at Enter. */
            if (length < (sizeof(command) - 1U)) {
                command[length++] = character;
                uart_putc(character);
            }
        }
    }
}
