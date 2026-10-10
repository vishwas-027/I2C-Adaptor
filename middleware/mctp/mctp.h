/*
 * MCTP packet API.
 *
 * The MCTP layer deliberately operates on complete MCTP packets, not on a
 * particular message type.  Consequently Control, PLDM, SPDM, vendor-defined
 * and future MCTP message types all use the same API.
 */
#ifndef MCTP_H
#define MCTP_H

#include <stdbool.h>
#include <stdint.h>

#define MCTP_HEADER_SIZE          4U
/* SMBus byte-count is 8 bits and includes one SMBus source-address byte. */
#define MCTP_PACKET_MAX_SIZE      256U
#define MCTP_TIMEOUT_DEFAULT_MS   2000U

typedef enum {
    MCTP_OK = 0,
    MCTP_ERROR_INVALID_ARGUMENT,
    MCTP_ERROR_PACKET_FORMAT,
    MCTP_ERROR_PACKET_TOO_LARGE,
    MCTP_ERROR_TIMEOUT,
    MCTP_ERROR_NACK,
    MCTP_ERROR_TRANSPORT,
    MCTP_ERROR_PEC,
    MCTP_ERROR_NOT_SUPPORTED
} MctpStatus;

/* data begins with the four-byte MCTP transport header:
 *   header-version, destination EID, source EID, flags/sequence/tag.
 * It contains no SMBus command, byte count, source slave address, or PEC. */
typedef struct {
    uint8_t data[MCTP_PACKET_MAX_SIZE];
    uint16_t length;
    uint8_t source_smbus_address;
} MctpPacket;

/* A transport supplies complete MCTP packets to the core. receive() shall
 * wait no longer than timeout_ms and return MCTP_ERROR_TIMEOUT on expiry. */
typedef MctpStatus (*MctpTransmitFn)(void *context,
                                     uint8_t destination_smbus_address,
                                     const MctpPacket *packet);
typedef MctpStatus (*MctpReceiveFn)(void *context,
                                    MctpPacket *packet,
                                    uint32_t timeout_ms);

typedef struct {
    MctpTransmitFn transmit;
    MctpReceiveFn receive;
    void *context;
    uint32_t timeout_ms;
} Mctp;

/* Bind an MCTP instance to any packet transport. This keeps the public MCTP
 * API reusable with future bindings while mctp_smbus.c implements SMBus/I2C. */
void mctp_init(Mctp *mctp,
               MctpTransmitFn transmit,
               MctpReceiveFn receive,
               void *context,
               uint32_t timeout_ms);

/* Send only. This is intentionally separate from mctp_receive(), allowing a
 * caller to wait for a deferred response such as an MPR completion. */
MctpStatus mctp_send(Mctp *mctp,
                     uint8_t destination_smbus_address,
                     const uint8_t *packet,
                     uint16_t packet_length);

/* Receive only. timeout_ms == 0 uses the timeout selected at mctp_init(). */
MctpStatus mctp_receive(Mctp *mctp, MctpPacket *packet, uint32_t timeout_ms);

/* Convenience operation for immediate request/response exchanges. */
MctpStatus mctp_exchange(Mctp *mctp,
                         uint8_t destination_smbus_address,
                         const uint8_t *request,
                         uint16_t request_length,
                         MctpPacket *response,
                         uint32_t timeout_ms);

#endif /* MCTP_H */
