#include "nvme_mi.h"

#include <stddef.h>

#define NVME_MI_MCTP_HEADER_SIZE       4U
#define NVME_MI_MESSAGE_HEADER_SIZE    4U
#define NVME_MI_ADMIN_REQUEST_SIZE     68U
#define NVME_MI_MIC_SIZE                4U
#define NVME_MI_PACKET_SIZE (NVME_MI_MCTP_HEADER_SIZE + NVME_MI_ADMIN_REQUEST_SIZE + NVME_MI_MIC_SIZE)
#define NVME_MI_MCTP_DEFAULT_MTU 64U

static void nvme_mi_store_le32(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
    destination[2] = (uint8_t)(value >> 16U);
    destination[3] = (uint8_t)(value >> 24U);
}

static uint32_t nvme_mi_crc32c(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;

    for (uint16_t index = 0U; index < length; index++) {
        crc ^= data[index];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc & 1U) ? ((crc >> 1U) ^ 0x82F63B78UL) : (crc >> 1U);
        }
    }
    return ~crc;
}

static MctpStatus nvme_mi_validate(const NvmeMi *nvme_mi,
                                   uint32_t data_length,
                                   uint32_t data_offset)
{
    if ((nvme_mi == NULL) || (nvme_mi->mctp == NULL) ||
        (nvme_mi->destination_smbus_address > 0x7FU)) {
        return MCTP_ERROR_INVALID_ARGUMENT;
    }
    /* NVMe-MI Admin Command DLEN/DOFST are byte fields, restricted to 4 KiB
     * and DWORD alignment by the NVMe-MI specification. */
    if ((data_length == 0U) || (data_length > 4096U) ||
        ((data_length & 0x3U) != 0U) || ((data_offset & 0x3U) != 0U)) {
        return MCTP_ERROR_INVALID_ARGUMENT;
    }
    return MCTP_OK;
}

static void nvme_mi_build_base(const NvmeMi *nvme_mi, uint8_t opcode,
                               uint32_t data_length, uint32_t data_offset,
                               uint8_t *packet)
{
    /* MCTP packet header: v1, EIDs, one complete packet, requester owns tag. */
    packet[0] = 0x01U;
    packet[1] = nvme_mi->destination_eid;
    packet[2] = nvme_mi->source_eid;
    packet[3] = (uint8_t)(0xC8U | (nvme_mi->message_tag & 0x07U));

    /* NVMe-MI header: MCTP type 4 with MIC, request, NVMe Admin command. */
    packet[4] = 0x84U;
    packet[5] = 0x10U;
    packet[6] = 0U;
    packet[7] = 0U;
    packet[8] = opcode;
    packet[9] = 0U;
    packet[10] = 0U;
    packet[11] = 0U;

    /* CDW1..5 are zero. DOFST and DLEN occupy bytes 28 and 32 of the
     * NVMe-MI Admin request (raw MCTP offsets 32 and 36). */
    for (uint16_t index = 12U; index < 72U; index++) {
        packet[index] = 0U;
    }
    nvme_mi_store_le32(&packet[32], data_offset);
    nvme_mi_store_le32(&packet[36], data_length);
}

static void nvme_mi_finish(uint8_t *packet, uint16_t *packet_length)
{
    uint32_t mic = nvme_mi_crc32c(&packet[NVME_MI_MCTP_HEADER_SIZE],
                                  NVME_MI_ADMIN_REQUEST_SIZE);
    nvme_mi_store_le32(&packet[72], mic);
    *packet_length = NVME_MI_PACKET_SIZE;
}

static MctpStatus nvme_mi_send_message(NvmeMi *nvme_mi,
                                       const uint8_t *packet,
                                       uint16_t packet_length)
{
    uint8_t fragment[NVME_MI_MCTP_DEFAULT_MTU];
    uint16_t message_offset = 0U;
    uint8_t sequence = 0U;

    if (packet_length <= NVME_MI_MCTP_DEFAULT_MTU) {
        return mctp_send(nvme_mi->mctp, nvme_mi->destination_smbus_address,
                         packet, packet_length);
    }

    /* NVMe-MI Admin requests are 76 bytes including the MCTP header. The
     * standard 64-byte MCTP MTU therefore requires two packet fragments. */
    while (message_offset < (uint16_t)(packet_length - NVME_MI_MCTP_HEADER_SIZE)) {
        uint16_t remaining = (uint16_t)(packet_length - NVME_MI_MCTP_HEADER_SIZE - message_offset);
        uint16_t payload_length = (remaining > (NVME_MI_MCTP_DEFAULT_MTU - NVME_MI_MCTP_HEADER_SIZE))
                                ? (NVME_MI_MCTP_DEFAULT_MTU - NVME_MI_MCTP_HEADER_SIZE) : remaining;
        bool som = (message_offset == 0U);
        bool eom = (payload_length == remaining);
        MctpStatus status;

        fragment[0] = packet[0];
        fragment[1] = packet[1];
        fragment[2] = packet[2];
        fragment[3] = (uint8_t)((som ? 0x80U : 0U) | (eom ? 0x40U : 0U) |
                                ((sequence & 0x03U) << 4U) | 0x08U |
                                (nvme_mi->message_tag & 0x07U));
        for (uint16_t index = 0U; index < payload_length; index++) {
            fragment[NVME_MI_MCTP_HEADER_SIZE + index] =
                packet[NVME_MI_MCTP_HEADER_SIZE + message_offset + index];
        }
        status = mctp_send(nvme_mi->mctp, nvme_mi->destination_smbus_address,
                           fragment,
                           (uint16_t)(NVME_MI_MCTP_HEADER_SIZE + payload_length));
        if (status != MCTP_OK) {
            return status;
        }
        message_offset = (uint16_t)(message_offset + payload_length);
        sequence++;
    }
    return MCTP_OK;
}

void nvme_mi_init(NvmeMi *nvme_mi,
                  Mctp *mctp,
                  uint8_t destination_smbus_address,
                  uint8_t destination_eid,
                  uint8_t source_eid,
                  uint8_t message_tag)
{
    if (nvme_mi == NULL) {
        return;
    }
    nvme_mi->mctp = mctp;
    nvme_mi->destination_smbus_address = destination_smbus_address;
    nvme_mi->destination_eid = destination_eid;
    nvme_mi->source_eid = source_eid;
    nvme_mi->message_tag = (uint8_t)(message_tag & 0x07U);
}

MctpStatus nvme_mi_build_identify(const NvmeMi *nvme_mi,
                                  const NvmeMiIdentifyRequest *request,
                                  uint8_t *packet,
                                  uint16_t *packet_length)
{
    MctpStatus status;

    if ((request == NULL) || (packet == NULL) || (packet_length == NULL)) {
        return MCTP_ERROR_INVALID_ARGUMENT;
    }
    status = nvme_mi_validate(nvme_mi, request->data_length, request->data_offset);
    if (status != MCTP_OK) {
        return status;
    }

    nvme_mi_build_base(nvme_mi, NVME_MI_ADMIN_IDENTIFY, request->data_length,
                       request->data_offset, packet);
    /* Identify CNS is in CDW10; the test-facing dws argument is raw CDW11. */
    nvme_mi_store_le32(&packet[48], request->cns);
    nvme_mi_store_le32(&packet[52], request->dword11);
    nvme_mi_finish(packet, packet_length);
    return MCTP_OK;
}

MctpStatus nvme_mi_build_get_log_page(const NvmeMi *nvme_mi,
                                      const NvmeMiGetLogPageRequest *request,
                                      uint8_t *packet,
                                      uint16_t *packet_length)
{
    MctpStatus status;
    uint32_t dword_count;
    uint32_t cdw10;

    if ((request == NULL) || (packet == NULL) || (packet_length == NULL)) {
        return MCTP_ERROR_INVALID_ARGUMENT;
    }
    status = nvme_mi_validate(nvme_mi, request->data_length, request->data_offset);
    if (status != MCTP_OK) {
        return status;
    }

    nvme_mi_build_base(nvme_mi, NVME_MI_ADMIN_GET_LOG_PAGE, request->data_length,
                       request->data_offset, packet);
    dword_count = request->data_length / 4U;
    /* CDW10: LID[7:0] and NUMDL[31:16]. CDW11 is caller controlled. */
    cdw10 = (uint32_t)request->lid | ((dword_count - 1U) << 16U);
    nvme_mi_store_le32(&packet[48], cdw10);
    nvme_mi_store_le32(&packet[52], request->dword11);
    /* CDW12/CDW13 (Log Page Offset) remain zero; data_offset above is the
     * NVMe-MI DOFST, i.e. response-data selection, not an NVMe LPO. */
    nvme_mi_finish(packet, packet_length);
    return MCTP_OK;
}

MctpStatus nvme_mi_send_identify(NvmeMi *nvme_mi,
                                 const NvmeMiIdentifyRequest *request)
{
    uint8_t packet[NVME_MI_PACKET_SIZE];
    uint16_t packet_length;
    MctpStatus status = nvme_mi_build_identify(nvme_mi, request, packet, &packet_length);

    if (status != MCTP_OK) {
        return status;
    }
    return nvme_mi_send_message(nvme_mi, packet, packet_length);
}

MctpStatus nvme_mi_send_get_log_page(NvmeMi *nvme_mi,
                                     const NvmeMiGetLogPageRequest *request)
{
    uint8_t packet[NVME_MI_PACKET_SIZE];
    uint16_t packet_length;
    MctpStatus status = nvme_mi_build_get_log_page(nvme_mi, request, packet,
                                                    &packet_length);

    if (status != MCTP_OK) {
        return status;
    }
    return nvme_mi_send_message(nvme_mi, packet, packet_length);
}

MctpStatus nvme_mi_receive(NvmeMi *nvme_mi,
                           MctpPacket *response,
                           uint32_t timeout_ms)
{
    if ((nvme_mi == NULL) || (nvme_mi->mctp == NULL)) {
        return MCTP_ERROR_INVALID_ARGUMENT;
    }
    return mctp_receive(nvme_mi->mctp, response, timeout_ms);
}

MctpStatus nvme_mi_response_parser_view(const MctpPacket *response,
                                        const uint8_t **body,
                                        uint16_t *body_length)
{
    if ((response == NULL) || (body == NULL) || (body_length == NULL)) {
        return MCTP_ERROR_INVALID_ARGUMENT;
    }

    if (response->length <= NVME_MI_RESPONSE_PARSER_PREFIX_SIZE) {
        return MCTP_ERROR_PACKET_FORMAT;
    }

    *body = &response->data[NVME_MI_RESPONSE_PARSER_PREFIX_SIZE];
    *body_length = (uint16_t)(response->length -
                              NVME_MI_RESPONSE_PARSER_PREFIX_SIZE);
    return MCTP_OK;
}
