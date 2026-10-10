/* NVMe-MI out-of-band NVMe Admin command packet generator. */
#ifndef NVME_MI_H
#define NVME_MI_H

#include "../mctp/mctp.h"

#define NVME_MI_DEFAULT_DATA_LENGTH       64U
#define NVME_MI_DEFAULT_DATA_OFFSET       0U
#define NVME_MI_DEFAULT_COMMAND_DWORD_11  0U

#define NVME_MI_ADMIN_GET_LOG_PAGE        0x02U
#define NVME_MI_ADMIN_IDENTIFY            0x06U
#define NVME_MI_RESPONSE_PARSER_PREFIX_SIZE 6U

typedef struct {
    Mctp *mctp;
    uint8_t destination_smbus_address;
    uint8_t destination_eid;
    uint8_t source_eid;
    uint8_t message_tag;
} NvmeMi;

/* dword11 is intentionally exposed as a raw NVMe CDW11 value for testing
 * command-specific options. data_offset is the NVMe-MI response DOFST. */
typedef struct {
    uint8_t cns;
    uint32_t data_length;
    uint32_t data_offset;
    uint32_t dword11;
} NvmeMiIdentifyRequest;

typedef struct {
    uint8_t lid;
    uint32_t data_length;
    uint32_t data_offset;
    uint32_t dword11;
} NvmeMiGetLogPageRequest;

void nvme_mi_init(NvmeMi *nvme_mi,
                  Mctp *mctp,
                  uint8_t destination_smbus_address,
                  uint8_t destination_eid,
                  uint8_t source_eid,
                  uint8_t message_tag);

MctpStatus nvme_mi_build_identify(const NvmeMi *nvme_mi,
                                  const NvmeMiIdentifyRequest *request,
                                  uint8_t *packet,
                                  uint16_t *packet_length);
MctpStatus nvme_mi_build_get_log_page(const NvmeMi *nvme_mi,
                                      const NvmeMiGetLogPageRequest *request,
                                      uint8_t *packet,
                                      uint16_t *packet_length);

MctpStatus nvme_mi_send_identify(NvmeMi *nvme_mi,
                                 const NvmeMiIdentifyRequest *request);
MctpStatus nvme_mi_send_get_log_page(NvmeMi *nvme_mi,
                                     const NvmeMiGetLogPageRequest *request);

/* Receive only, so an MPR response can be followed by the eventual command
 * completion without transmitting another request. */
MctpStatus nvme_mi_receive(NvmeMi *nvme_mi,
                           MctpPacket *response,
                           uint32_t timeout_ms);

/* The generic MCTP layer returns a complete packet beginning with the 4-byte
 * MCTP transport header. The NVMe-MI response parser, however, should start
 * at the reassembled response body after its fixed 6-byte prefix. */
MctpStatus nvme_mi_response_parser_view(const MctpPacket *response,
                                        const uint8_t **body,
                                        uint16_t *body_length);

#endif /* NVME_MI_H */
