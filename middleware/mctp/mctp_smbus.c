#include "mctp_smbus.h"

#include "i2c.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "logger.h"

#include <stddef.h>
#include <stdio.h>

/* command + byte-count + (source address + MCTP packet) + PEC */
#define MCTP_SMBUS_FRAME_MAX_SIZE (2U + 1U + MCTP_PACKET_MAX_SIZE + 1U)
#define MCTP_SMBUS_RX_FRAME_MAX_SIZE (1U + MCTP_PACKET_MAX_SIZE + 1U)
#define MCTP_SMBUS_RX_QUEUE_DEPTH 2U
#define MCTP_RX_SOM_MASK 0x80U
#define MCTP_RX_EOM_MASK 0x40U
#define MCTP_RX_SEQ_MASK 0x30U
#define MCTP_RX_SEQ_SHIFT 4U
#define MCTP_RX_TAG_OWNER_MASK 0x08U
#define MCTP_RX_TAG_MASK 0x07U

typedef struct {
    uint16_t length;
    uint8_t data[MCTP_SMBUS_RX_FRAME_MAX_SIZE];
} MctpSmbusI2c1RxFrame;

static QueueHandle_t s_i2c1_rx_queue;
static void mctp_smbus_log_reassembled_packet(const MctpPacket *packet)
{
    if (packet == NULL) {
        return;
    }

    LOG_INFO("MCTP RX: reassembled src_smbus=0x%02X len=%u hdr=%02X %02X %02X %02X",
             (unsigned int)packet->source_smbus_address,
             (unsigned int)packet->length,
             packet->data[0],
             packet->data[1],
             packet->data[2],
             packet->data[3]);

    char line[64];
    char ascii[17];

    for (uint16_t offset = 0U; offset < packet->length; offset += 16U) {
        uint16_t line_length = 0U;
        uint16_t byte_count = 0U;

        line_length += (uint16_t)snprintf(line + line_length,
                                          sizeof(line) - line_length,
                                          "MCTP PKT[%03u]:",
                                          (unsigned int)offset);
        for (uint16_t index = offset;
             (index < packet->length) &&
             (index < (uint16_t)(offset + 16U)); index++) {
            uint8_t value = packet->data[index];
            ascii[byte_count++] = ((value >= 0x20U) && (value <= 0x7EU)) ?
                               (char)value : '.';
            line_length += (uint16_t)snprintf(line + line_length,
                                              sizeof(line) - line_length,
                                              " %02X",
                                              value);
        }
        ascii[byte_count] = '\0';
        LOG_INFO("%s  |%s|", line, ascii);
    }
}

static void mctp_smbus_i2c1_on_receive(const uint8_t *received,
                                       uint16_t received_length)
{
    MctpSmbusI2c1RxFrame frame;
    BaseType_t task_woken = pdFALSE;
    uint8_t byte_count;

    /* The I2C callback receives only the SMBus Block Write payload:
     * command, byte-count, source-slave-address, MCTP packet bytes, PEC.
     * It does not include any extra outer transport/header bytes. Keep only
     * the bytes after block count so mctp_smbus_receive() sees exactly
     * source-slave-address, MCTP packet, PEC. */
    if ((received == NULL) || (received_length < 4U) ||
        (received[0] != MCTP_SMBUS_COMMAND_CODE)) {
        return;
    }
    byte_count = received[1];
    if ((byte_count == 0U) ||
        (received_length != (uint16_t)(byte_count + 3U))) {
        return;
    }

    frame.length = (uint16_t)(byte_count + 1U); /* block data plus PEC */
    if (frame.length > sizeof(frame.data)) {
        return;
    }
    for (uint16_t index = 0U; index < frame.length; index++) {
        frame.data[index] = received[2U + index];
    }
    (void)xQueueSendFromISR(s_i2c1_rx_queue, &frame, &task_woken);
    portYIELD_FROM_ISR(task_woken);
}

static MctpStatus mctp_smbus_transmit(void *context,
                                      uint8_t destination_smbus_address,
                                      const MctpPacket *packet)
{
    MctpSmbus *smbus = (MctpSmbus *)context;
    uint8_t frame[MCTP_SMBUS_FRAME_MAX_SIZE];
    uint16_t index;
    uint8_t byte_count;

    if ((smbus == NULL) || (smbus->write == NULL) || (packet == NULL) ||
        (packet->length > MCTP_PACKET_MAX_SIZE)) {
        return MCTP_ERROR_INVALID_ARGUMENT;
    }

    byte_count = (uint8_t)(1U + packet->length);
    frame[0] = MCTP_SMBUS_COMMAND_CODE;
    frame[1] = byte_count;
    frame[2] = (uint8_t)((smbus->local_smbus_address << 1U) |
                         MCTP_SMBUS_SOURCE_RW_BIT);
    for (index = 0U; index < packet->length; index++) {
        frame[3U + index] = packet->data[index];
    }

    /* Continue CRC over the SMBus command, count, and block data after the
     * destination address+W that preceded this driver call on the wire. */
    {
        uint8_t crc = (uint8_t)(destination_smbus_address << 1U);
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0x07U)
                                 : (uint8_t)(crc << 1U);
        }
        for (index = 0U; index < (uint16_t)(2U + byte_count); index++) {
            crc ^= frame[index];
            for (uint8_t bit = 0U; bit < 8U; bit++) {
                crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0x07U)
                                     : (uint8_t)(crc << 1U);
            }
        }
        frame[3U + packet->length] = crc;
    }

    LOG_INFO("MCTP SMBus TX: dst=0x%02X source=0x%02X packet=%u block=%u",
             destination_smbus_address, frame[2], (unsigned int)packet->length,
             (unsigned int)byte_count);
    return smbus->write(smbus->io_context, destination_smbus_address, frame,
                        (uint16_t)(3U + packet->length + 1U));
}

static MctpStatus mctp_smbus_receive(void *context,
                                     MctpPacket *packet,
                                     uint32_t timeout_ms)
{
    MctpSmbus *smbus = (MctpSmbus *)context;
    uint8_t frame[1U + MCTP_PACKET_MAX_SIZE + 1U];
    uint16_t frame_length = 0U;
    uint8_t crc_input[4U + MCTP_PACKET_MAX_SIZE];
    uint16_t assembled_length = 0U;
    uint8_t expected_sequence = 0U;
    uint8_t message_tag = 0U;
    uint8_t source_address = 0U;
    bool waiting_for_eom = false;

    if ((smbus == NULL) || (packet == NULL) || (smbus->read == NULL)) {
        return MCTP_ERROR_NOT_SUPPORTED;
    }
    for (;;) {
        uint16_t fragment_packet_length;
        uint8_t flags;
        bool som;
        bool eom;
        uint8_t sequence;
        MctpStatus status = smbus->read(smbus->io_context, frame, &frame_length,
                                        sizeof(frame), timeout_ms);

        if (status != MCTP_OK) {
            return status;
        }
        if ((frame_length < (1U + MCTP_HEADER_SIZE + 1U)) ||
            ((frame[0] & MCTP_SMBUS_SOURCE_RW_BIT) == 0U)) {
            return MCTP_ERROR_PACKET_FORMAT;
        }

        crc_input[0] = (uint8_t)(smbus->local_smbus_address << 1U);
        crc_input[1] = MCTP_SMBUS_COMMAND_CODE;
        crc_input[2] = (uint8_t)(frame_length - 1U); /* SMBus byte count */
        for (uint16_t index = 0U; index < (uint16_t)(frame_length - 1U); index++) {
            crc_input[3U + index] = frame[index];
        }
        if (mctp_smbus_pec(crc_input, (uint16_t)(frame_length + 2U)) !=
            frame[frame_length - 1U]) {
            return MCTP_ERROR_PEC;
        }

        fragment_packet_length = (uint16_t)(frame_length - 2U);
        if (fragment_packet_length > MCTP_PACKET_MAX_SIZE) {
            return MCTP_ERROR_PACKET_TOO_LARGE;
        }

        flags = frame[1U + 3U];
        som = ((flags & MCTP_RX_SOM_MASK) != 0U);
        eom = ((flags & MCTP_RX_EOM_MASK) != 0U);
        sequence = (uint8_t)((flags & MCTP_RX_SEQ_MASK) >> MCTP_RX_SEQ_SHIFT);

        if (som) {
            assembled_length = 0U;
            waiting_for_eom = true;
            expected_sequence = sequence;
            message_tag = (uint8_t)(flags & MCTP_RX_TAG_MASK);
            source_address = (uint8_t)(frame[0] >> 1U);

            if (fragment_packet_length < MCTP_HEADER_SIZE) {
                return MCTP_ERROR_PACKET_FORMAT;
            }
            for (uint16_t index = 0U; index < fragment_packet_length; index++) {
                packet->data[index] = frame[1U + index];
            }
            assembled_length = fragment_packet_length;
        } else {
            if ((!waiting_for_eom) ||
                (((uint8_t)(frame[0] >> 1U)) != source_address) ||
                ((flags & MCTP_RX_TAG_MASK) != message_tag) ||
                (sequence != expected_sequence)) {
                return MCTP_ERROR_PACKET_FORMAT;
            }

            if ((uint16_t)(assembled_length + fragment_packet_length - MCTP_HEADER_SIZE) >
                MCTP_PACKET_MAX_SIZE) {
                return MCTP_ERROR_PACKET_TOO_LARGE;
            }

            for (uint16_t index = MCTP_HEADER_SIZE; index < fragment_packet_length; index++) {
                packet->data[assembled_length++] = frame[1U + index];
            }
        }

        expected_sequence = (uint8_t)((sequence + 1U) & 0x03U);

        if (som && !eom) {
            continue;
        }
        if (!som && !eom) {
            continue;
        }

        packet->source_smbus_address = source_address;
        packet->length = assembled_length;
        mctp_smbus_log_reassembled_packet(packet);
        return MCTP_OK;
    }
}

uint8_t mctp_smbus_pec(const uint8_t *bytes, uint16_t length)
{
    uint8_t crc = 0U;

    if (bytes == NULL) {
        return 0U;
    }
    for (uint16_t index = 0U; index < length; index++) {
        crc ^= bytes[index];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0x07U)
                                 : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

MctpStatus mctp_smbus_init(Mctp *mctp,
                           MctpSmbus *smbus,
                           uint8_t local_smbus_address,
                           MctpSmbusWriteFn write,
                           MctpSmbusReadFn read,
                           void *io_context,
                           uint32_t timeout_ms)
{
    if ((mctp == NULL) || (smbus == NULL) || (write == NULL) ||
        (local_smbus_address > 0x7FU)) {
        return MCTP_ERROR_INVALID_ARGUMENT;
    }

    smbus->local_smbus_address = local_smbus_address;
    smbus->write = write;
    smbus->read = read;
    smbus->io_context = io_context;
    mctp_init(mctp, mctp_smbus_transmit, mctp_smbus_receive, smbus, timeout_ms);
    return MCTP_OK;
}

MctpStatus mctp_smbus_i2c1_write(void *context,
                                 uint8_t destination_smbus_address,
                                 const uint8_t *frame,
                                 uint16_t frame_length)
{
    I2C_Status_t status;
    (void)context;

    if ((frame == NULL) || (frame_length < 4U)) {
        return MCTP_ERROR_INVALID_ARGUMENT;
    }
    status = I2C1_Master_Transmit(destination_smbus_address, frame, frame_length);
    if (status == I2C_OK) {
        return MCTP_OK;
    }
    if (status == I2C_TIMEOUT) {
        return MCTP_ERROR_TIMEOUT;
    }
    if (status == I2C_NACK) {
        return MCTP_ERROR_NACK;
    }
    return MCTP_ERROR_TRANSPORT;
}

MctpStatus mctp_smbus_i2c1_enable_receive(uint8_t local_smbus_address)
{
    I2C_Status_t status = mctp_smbus_i2c1_configure_receive(
        local_smbus_address, I2C1_SLAVE_TX_MAX_LENGTH,
        I2C1_SLAVE_RX_MAX_LENGTH);

    if (status == I2C_OK) {
        return MCTP_OK;
    }
    return (status == I2C_ERROR) ? MCTP_ERROR_INVALID_ARGUMENT
                                 : MCTP_ERROR_TRANSPORT;
}

I2C_Status_t mctp_smbus_i2c1_configure_receive(uint8_t local_smbus_address,
                                                uint16_t max_tx_bytes,
                                                uint16_t max_rx_bytes)
{
    if (local_smbus_address > 0x7FU) {
        return I2C_ERROR;
    }
    if (s_i2c1_rx_queue == NULL) {
        s_i2c1_rx_queue = xQueueCreate(MCTP_SMBUS_RX_QUEUE_DEPTH,
                                       sizeof(MctpSmbusI2c1RxFrame));
        if (s_i2c1_rx_queue == NULL) {
            return I2C_ERROR;
        }
    }
    LOG_INFO("MCTP SMBus: listen address 0x%02X", local_smbus_address);
    {
        I2C_Status_t status = I2C1_ConfigureSlave(
            local_smbus_address, max_tx_bytes, max_rx_bytes,
            mctp_smbus_i2c1_on_receive);
        if (status == I2C_OK) {
            (void)xQueueReset(s_i2c1_rx_queue);
            return I2C_OK;
        }
        LOG_ERROR("MCTP SMBus: failed to enable receive for listen address 0x%02X", local_smbus_address);
        return status;
    }
}

MctpStatus mctp_smbus_i2c1_flush_receive(void)
{
    if (s_i2c1_rx_queue == NULL) {
        return MCTP_ERROR_NOT_SUPPORTED;
    }
    return (xQueueReset(s_i2c1_rx_queue) == pdPASS)
               ? MCTP_OK : MCTP_ERROR_TRANSPORT;
}

MctpStatus mctp_smbus_i2c1_read(void *context,
                                uint8_t *frame,
                                uint16_t *frame_length,
                                uint16_t frame_capacity,
                                uint32_t timeout_ms)
{
    MctpSmbusI2c1RxFrame received;
    TickType_t ticks;
    (void)context;

    if ((s_i2c1_rx_queue == NULL) || (frame == NULL) ||
        (frame_length == NULL)) {
        return MCTP_ERROR_NOT_SUPPORTED;
    }
    ticks = pdMS_TO_TICKS(timeout_ms);
    if ((timeout_ms > 0U) && (ticks == 0U)) {
        ticks = 1U;
    }
    if (xQueueReceive(s_i2c1_rx_queue, &received, ticks) != pdTRUE) {
        LOG_WARN("MCTP SMBus RX: timeout after %lu ms", (unsigned long)timeout_ms);
        return MCTP_ERROR_TIMEOUT;
    }
    if (received.length > frame_capacity) {
        return MCTP_ERROR_PACKET_TOO_LARGE;
    }
    for (uint16_t index = 0U; index < received.length; index++) {
        frame[index] = received.data[index];
    }
    *frame_length = received.length;
    LOG_INFO("MCTP SMBus RX: captured %u bytes after block count (src addr + MCTP packet + PEC)",
             (unsigned int)received.length);
    return MCTP_OK;
}
