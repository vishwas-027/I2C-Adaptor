#ifndef ADAPTER_DISCOVERY_H
#define ADAPTER_DISCOVERY_H

#include <libusb.h>
#include <stdint.h>

typedef uint16_t u16;
typedef uint32_t u32;
typedef uint8_t u08;
typedef int Aardvark;
typedef u16 AardvarkI2cFlags;
typedef int I2cQueue;

typedef struct {
    u16 software;
    u16 firmware;
    u16 hardware;
    u16 sw_req_by_fw;
    u16 fw_req_by_sw;
    u16 api_req_by_sw;
} AardvarkVersion;

typedef struct {
    AardvarkVersion version;
    int features;
} AardvarkExt;

#define ADAPTER_PORT_NOT_FREE 0x8000U
#define ADAPTER_FEATURE_SPI        (1U << 0)
#define ADAPTER_FEATURE_I2C        (1U << 1)
#define ADAPTER_FEATURE_GPIO       (1U << 3)

#define STATUS_INCOMPATIBLE_DEVICE (-5)
#define STATUS_COMMUNICATION_ERROR  (-6)
#define STATUS_UNABLE_TO_OPEN       (-7)
#define STATUS_INVALID_HANDLE       (-9)
#define STATUS_INVALID_QUEUE       (-20)
#define STATUS_QUEUE_FULL          (-21)
#define STATUS_QUEUE_BUSY          (-22)
#define STATUS_ABORTED             (-23)
#define STATUS_NO_MEMORY           (-24)
#define I2C_READ_ERROR              (-102)
#define I2C_WRITE_ERROR             (-103)
#define I2C_SLAVE_WRITE_ERROR       (-104)

#define I2C_NO_FLAGS                 0x0000U

/* Aardvark-style adapter enumeration. Returns the total number found even
 * when num_devices is smaller than that total or devices is NULL. */
int find_devices(int num_devices, u16 *devices);

/* Extended Aardvark-style enumeration. In addition to port identifiers,
 * returns the numeric USB serial number for each adapter. When both arrays are
 * supplied, each is limited by the smaller of num_devices and num_ids. */
int find_devices_ext(int num_devices, u16 *devices,
                     int num_ids, u32 *unique_ids);

/* Open and exclusively claim an adapter. The returned handle is positive on
 * success or a negative status code on failure. extended_info may be NULL. */
Aardvark open_ext(int port_number, AardvarkExt *extended_info);

/* Write num_bytes to a 7-bit I2C slave. Returns the number of bytes written
 * on success or a negative status code. */
int i2c_write(Aardvark aardvark, u16 slave_address,
              AardvarkI2cFlags flags, u16 num_bytes,
              const u08 *data);

/* Read up to num_bytes from a 7-bit I2C slave into data. Returns the number
 * of bytes read on success or a negative status code. */
int i2c_read(Aardvark aardvark, u16 slave_address,
             AardvarkI2cFlags flags, u16 num_bytes, u08 *data);

/* Preload the cyclic response transmitted when an external I2C controller
 * reads the adapter in slave mode. Returns the accepted byte count. */
int i2c_slave_write(Aardvark aardvark, u16 num_bytes, const u08 *data);

/* Create an ordered host-side I2C transaction queue bound to aardvark.
 * Returns a positive queue handle or a negative status code. */
I2cQueue queue_create(Aardvark aardvark);

/* Append an I2C write. The write data is copied into queue-owned storage. */
int queue_i2c_write(I2cQueue queue, u16 slave_address,
                    AardvarkI2cFlags flags, u16 num_bytes,
                    const u08 *data);

/* Append an I2C read. data must remain valid during every queue submission
 * that executes this command. */
int queue_i2c_read(I2cQueue queue, u16 slave_address,
                   AardvarkI2cFlags flags, u16 num_bytes, u08 *data);

/* Execute all queued transactions in order. Returns the number completed,
 * or a negative status code. The queue remains available for resubmission. */
int queue_submit(I2cQueue queue);

/* Request that an active queue_submit stop before its next transaction. */
int async_abort(I2cQueue queue);

/* Remove all queued transactions or release the queue handle. */
int queue_clear(I2cQueue queue);
int queue_destroy(I2cQueue queue);

/* Release one open handle, or every handle when aardvark is zero. */
int adapter_close(Aardvark aardvark);

/* Resolve an Aardvark handle for low-level libusb host operations. The
 * returned pointer is borrowed, is invalid after adapter_close, and must not
 * be used concurrently with the serialized high-level API. */
libusb_device_handle *adapter_libusb_handle(Aardvark aardvark);

/* Open a zero-based port returned by find_devices(). The NOT_FREE flag is
 * ignored when resolving the port number. */
libusb_device_handle *adapter_open_port(libusb_context *context, u16 port);

#endif /* ADAPTER_DISCOVERY_H */
