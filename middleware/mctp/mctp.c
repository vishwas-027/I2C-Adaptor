#include "mctp.h"

#include <stddef.h>

static bool mctp_packet_is_valid(const uint8_t *packet, uint16_t length)
{
    /* All MCTP packets contain the four-byte transport header. The upper
     * nibble of byte zero is reserved and must be zero for header version 1. */
    return (packet != NULL) &&
           (length >= MCTP_HEADER_SIZE) &&
           (length <= MCTP_PACKET_MAX_SIZE) &&
           ((packet[0] & 0xF0U) == 0U) &&
           ((packet[0] & 0x0FU) == 1U);
}

void mctp_init(Mctp *mctp,
               MctpTransmitFn transmit,
               MctpReceiveFn receive,
               void *context,
               uint32_t timeout_ms)
{
    if (mctp == NULL) {
        return;
    }

    mctp->transmit = transmit;
    mctp->receive = receive;
    mctp->context = context;
    mctp->timeout_ms = (timeout_ms == 0U) ? MCTP_TIMEOUT_DEFAULT_MS : timeout_ms;
}

MctpStatus mctp_send(Mctp *mctp,
                     uint8_t destination_smbus_address,
                     const uint8_t *packet,
                     uint16_t packet_length)
{
    MctpPacket tx_packet;

    if ((mctp == NULL) || (mctp->transmit == NULL) ||
        (destination_smbus_address > 0x7FU)) {
        return MCTP_ERROR_INVALID_ARGUMENT;
    }
    if (packet == NULL) {
        return MCTP_ERROR_INVALID_ARGUMENT;
    }
    if (packet_length > MCTP_PACKET_MAX_SIZE) {
        return MCTP_ERROR_PACKET_TOO_LARGE;
    }
    if (!mctp_packet_is_valid(packet, packet_length)) {
        return MCTP_ERROR_PACKET_FORMAT;
    }

    for (uint16_t index = 0U; index < packet_length; index++) {
        tx_packet.data[index] = packet[index];
    }
    tx_packet.length = packet_length;
    tx_packet.source_smbus_address = 0U;

    return mctp->transmit(mctp->context, destination_smbus_address, &tx_packet);
}

MctpStatus mctp_receive(Mctp *mctp, MctpPacket *packet, uint32_t timeout_ms)
{
    MctpStatus status;

    if ((mctp == NULL) || (packet == NULL)) {
        return MCTP_ERROR_INVALID_ARGUMENT;
    }
    if (mctp->receive == NULL) {
        return MCTP_ERROR_NOT_SUPPORTED;
    }

    if (timeout_ms == 0U) {
        timeout_ms = mctp->timeout_ms;
    }
    status = mctp->receive(mctp->context, packet, timeout_ms);
    if (status != MCTP_OK) {
        return status;
    }
    if (!mctp_packet_is_valid(packet->data, packet->length)) {
        return MCTP_ERROR_PACKET_FORMAT;
    }
    return MCTP_OK;
}

MctpStatus mctp_exchange(Mctp *mctp,
                         uint8_t destination_smbus_address,
                         const uint8_t *request,
                         uint16_t request_length,
                         MctpPacket *response,
                         uint32_t timeout_ms)
{
    MctpStatus status = mctp_send(mctp, destination_smbus_address,
                                  request, request_length);

    if (status != MCTP_OK) {
        return status;
    }
    return mctp_receive(mctp, response, timeout_ms);
}
