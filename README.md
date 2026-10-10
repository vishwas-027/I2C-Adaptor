# STM32F411 SMBus/I2C USB Adapter

Firmware and host tools for using an STM32F411 Blackpill as a USB-connected
I2C/SMBus adapter. The API follows Aardvark-style naming where practical and
also supports MCTP-over-SMBus request/response transactions.

The current USB implementation supports:

- Adapter discovery
- I2C bus scanning
- 100 kHz and 400 kHz I2C bitrates
- I2C controller/master writes and reads
- Ordered host-side I2C transaction queues with cooperative abort
- Detailed Aardvark-compatible I2C transaction statuses
- I2C/SMBus addressed-slave mode and asynchronous event polling
- Retrieval of writes received while operating as an I2C slave
- Raw SMBus reads and block writes
- MCTP-over-SMBus request/response exchanges
- USB connection diagnostics

## Hardware and USB interface

| Property | Value |
|---|---:|
| MCU | STM32F411CEU6 |
| I2C peripheral | I2C1 |
| SCL | PB8 |
| SDA | PB9 |
| Initial I2C bitrate | 100000 Hz |
| USB VID | `0xCAFE` |
| USB PID | `0x4110` |
| USB interface | Vendor interface 0 |
| Bulk OUT endpoint | `0x01` |
| Bulk IN endpoint | `0x81` |
| Maximum USB payload | 512 bytes |

The VID/PID are development identifiers and must be replaced with legitimately
assigned values before distributing a product.

I2C uses open-drain signals. Connect appropriate external pull-ups, typically
about 4.7 kOhm to 3.3 V, and connect a common ground between the adapter and
the target.

## Building

Build the firmware with the Arm GNU toolchain:

```bash
make
```

Generated images are placed in `build/`:

- `build/firmware.elf`
- `build/firmware.bin`
- `build/firmware.hex`

Flash through ST-Link:

```bash
make flash
```

Build the libusb host client:

```bash
make host
```

On macOS, install libusb first if needed:

```bash
brew install libusb
```

All examples below are run from the project root using:

```bash
./host/usb_client/smbus-usb <command> [arguments]
```

Except for `find_devices`, the host client automatically discovers and opens
the first free adapter.

## Common values and return conventions

### I2C addresses and flags

All I2C device addresses are 7-bit values in the range `0x00` to `0x7F`.
Do not shift the address or include the read/write bit.

Only normal transactions are currently supported:

```c
I2C_NO_FLAGS = 0
```

The CLI accepts either `I2C_NO_FLAGS` or `0`.

### Host process exit codes

| Exit code | Meaning |
|---:|---|
| 0 | Operation completed successfully |
| 1 | USB, adapter, bus, or recognized-command argument validation failed |
| 2 | No command was supplied or the command name/syntax was not recognized |

For `i2c_write_ext` and `i2c_read_ext`, a valid USB response can still contain
a failed I2C transaction status. The CLI exits with code 1 unless the extended
status is `I2C_STATUS_OK`.

### USB response statuses

Every USB response has a protocol-level status in its 16-byte header.

| Value | Name | Meaning |
|---:|---|---|
| 0 | `USB_STATUS_OK` | Request was accepted |
| 1 | `USB_STATUS_BAD_VERSION` | Unsupported protocol version |
| 2 | `USB_STATUS_BAD_MESSAGE_TYPE` | Message was not a request |
| 3 | `USB_STATUS_BAD_LENGTH` | Incorrect payload framing or size |
| 4 | `USB_STATUS_UNSUPPORTED_OPCODE` | Opcode is not implemented |
| 5 | `USB_STATUS_BUSY` | Adapter or I2C peripheral is busy |
| 6 | `USB_STATUS_INVALID_ARGUMENT` | Address, flags, length, or another argument is invalid |
| 7 | `USB_STATUS_I2C_TIMEOUT` | I2C operation timed out |
| 8 | `USB_STATUS_I2C_NACK` | Target did not acknowledge |
| 9 | `USB_STATUS_I2C_BUS_ERROR` | Bus error or arbitration loss |
| 10 | `USB_STATUS_MCTP_ERROR` | General MCTP framing or transport error |
| 11 | `USB_STATUS_MCTP_TIMEOUT` | MCTP response timed out |
| 12 | `USB_STATUS_MCTP_PEC_ERROR` | Received SMBus PEC was invalid |

### Extended I2C transaction statuses

Extended APIs return a second, I2C-specific status inside the response
payload. These numeric values match the Aardvark status definitions.
The firmware source names them `I2C_EXT_STATUS_*` and the shared USB header
names them `USB_I2C_STATUS_*`; the `I2C_STATUS_*` names below show their
Aardvark API equivalents.

| Value | Name | Meaning |
|---:|---|---|
| 0 | `I2C_STATUS_OK` | Transaction completed successfully |
| 1 | `I2C_STATUS_BUS_ERROR` | Illegal START/STOP or another bus error occurred |
| 2 | `I2C_STATUS_SLA_ACK` | Reserved Aardvark-compatible status: slave address ACK |
| 3 | `I2C_STATUS_SLA_NACK` | Slave address was not acknowledged |
| 4 | `I2C_STATUS_DATA_NACK` | A transmitted data byte was not acknowledged |
| 5 | `I2C_STATUS_ARB_LOST` | Arbitration was lost to another controller |
| 6 | `I2C_STATUS_BUS_LOCKED` | Bus or peripheral remained locked until timeout |
| 7 | `I2C_STATUS_LAST_DATA_ACK` | Reserved Aardvark-compatible status: final read byte was ACKed |

`SLA_ACK` and `LAST_DATA_ACK` are defined for compatibility but are not normal
success results from the currently supported `I2C_NO_FLAGS` operations.

### Threading and long-running use

- High-level handle APIs serialize USB traffic per adapter. Multiple threads
  cannot interleave request sequences or consume each other's responses.
- `queue_submit()` owns its adapter for the complete queue submission. Other
  high-level transactions on that adapter wait until it finishes or aborts.
- `async_abort()` is lock-free with respect to adapter I/O and may be called
  from another thread. It takes effect before the next queued transaction.
- A late response left behind by a timed-out request is identified by its
  opcode and sequence and discarded while waiting for the current response.
- Always call `queue_destroy()` for queues and `adapter_close()` for adapter
  handles. Closing an idle adapter also releases any queues still bound to it.
- `adapter_libusb_handle()` returns a borrowed low-level pointer. Do not use
  it concurrently with high-level calls or `adapter_close()`.

## Supported APIs

The recommended lifecycle is:

1. Discover with `find_devices()` or `find_devices_ext()`.
2. Open a selected port with `open_ext()`.
3. Configure the bitrate, then perform controller transactions directly or
   through a queue.
4. If slave mode is needed, enable it, preload its response, poll for events,
   and consume received writes.
5. Destroy explicit queues and finish with `adapter_close()`.

### `find_devices`

Discovers compatible adapters connected to the host.

```c
int find_devices(int num_devices, u16 *devices);
```

- Returns the total number of matching adapters, even when `devices` is NULL
  or `num_devices` is smaller than the number found.
- Writes at most `num_devices` port identifiers to `devices`.
- `ADAPTER_PORT_NOT_FREE` (`0x8000`) is ORed into a port identifier when its
  USB interface cannot be claimed.
- Returns 0 when no adapters are found or discovery cannot initialize.

CLI usage:

```bash
./host/usb_client/smbus-usb find_devices
```

Example result:

```text
2 device(s) found
port 0
port 1 (in use)
```

Discovery matches USB VID `0xCAFE` and PID `0x4110`. Port numbering is stable
for the current USB topology: devices are sorted by bus and physical port path.

### `find_devices_ext`

Discovers compatible adapters and returns their stable numeric unique IDs.

```c
int find_devices_ext(int num_devices, u16 *devices,
                     int num_ids, u32 *unique_ids);
```

- Returns the total number of matching adapters regardless of array sizes.
- When both arrays are supplied, both are limited to the smaller of
  `num_devices` and `num_ids`. If either array is NULL, the other array uses
  its own requested size.
- Unique IDs are non-zero 32-bit values exposed as 10-digit USB serial
  numbers and derived from the STM32 factory-programmed 96-bit UID.
- An ID of zero means that the host could not read or validate the USB serial
  descriptor.

### `open_ext`

Opens and exclusively claims an adapter while returning its version matrix and
supported features.

```c
Aardvark open_ext(int port_number, AardvarkExt *extended_info);
```

- Returns a positive `Aardvark` handle on success.
- Returns `STATUS_UNABLE_TO_OPEN` when the port is invalid, absent, or already in
  use, and `STATUS_INCOMPATIBLE_DEVICE` for an unsupported firmware version.
- Zeroes `extended_info` before attempting the open and fills all information that is
  available. Passing NULL behaves like the simple open operation.
- Version fields use `(major << 8) | minor`. The firmware version comes from
  the USB `bcdDevice` descriptor; this adapter currently reports I2C support.
- Call `adapter_close(handle)` to release the USB interface and its resources.

CLI smoke test:

```bash
./host/usb_client/smbus-usb open_ext 0
```

### `i2c_bitrate`

Sets the I2C controller bitrate. The initial value after firmware startup is
100000 Hz.

```c
uint32_t applied_bitrate = i2c_bitrate(uint32_t bitrate_hz);
```

Supported inputs:

- `100000` for standard mode
- `400000` for fast mode

The return value is the bitrate actually applied, in Hz.

CLI usage:

```bash
./host/usb_client/smbus-usb i2c_bitrate 100000
./host/usb_client/smbus-usb i2c_bitrate 400000
```

Example result:

```text
I2C bitrate: 400000 Hz
```

### `i2c_detect`

Scans standard usable 7-bit addresses `0x03` through `0x77` and reports every
address that acknowledges.

Logical return value:

```c
uint8_t address_bitmap[16];
```

Bit N of the 16-byte bitmap represents address N.

CLI usage:

```bash
./host/usb_client/smbus-usb i2c_detect
```

Example result:

```text
0x20
0x50
2 device(s) found
```

### `i2c_write`

Performs a normal I2C controller/master write.

```c
int num_written = i2c_write(handle, slave_address, flags, num_bytes, data);
```

- `handle` is a positive handle returned by `open_ext()`.
- `slave_address` is a 7-bit slave address.
- `flags` currently supports `I2C_NO_FLAGS` only.
- `num_bytes` may be 0 to 507 and `data` may be NULL only when it is zero.
- A zero-byte write performs an SMBus Quick write/address probe.
- On success, the return value equals `num_bytes`.
- Returns `STATUS_INVALID_HANDLE`, `STATUS_COMMUNICATION_ERROR`, or
  `I2C_WRITE_ERROR` on failure.

CLI usage:

```bash
./host/usb_client/smbus-usb i2c_write 0x50 I2C_NO_FLAGS 0x01 0x02
./host/usb_client/smbus-usb i2c_write 0x50 0 0x01 0x02
./host/usb_client/smbus-usb i2c_write 0x50 0
```

Example result:

```text
I2C write: 2 byte(s) transmitted to 0x50
```

### `i2c_read`

Reads bytes from an I2C controller/master transaction.

```c
uint8_t buffer[16];
int num_read = i2c_read(handle, 0x50, I2C_NO_FLAGS, 16, buffer);
```

- `handle` is a positive handle returned by `open_ext()`.
- The slave address must be a valid 7-bit address.
- `num_bytes` must be between 1 and 509 and `data` must not be NULL.
- Returns the number of bytes copied into `data`, including a partial read.
- Returns `STATUS_INVALID_HANDLE`, `STATUS_COMMUNICATION_ERROR`, or
  `I2C_READ_ERROR` on an API or transport failure.

CLI usage:

```bash
./host/usb_client/smbus-usb i2c_read 0x50 I2C_NO_FLAGS 16
./host/usb_client/smbus-usb i2c_read 0x50 0 16
```

### `i2c_write_ext`

Performs the same controller/master write while returning detailed transaction
status and the number of bytes acknowledged before completion or failure.

```c
status, num_written = i2c_write_ext(
    uint8_t device,
    I2C_NO_FLAGS,
    const uint8_t *data,
    uint16_t length
);
```

- `length` may be 0 to 507 bytes.
- `num_written` is initialized to zero before the transaction.
- Only bytes fully acknowledged by the target are counted.
- An address NACK returns `I2C_STATUS_SLA_NACK` and zero bytes.
- A data NACK returns `I2C_STATUS_DATA_NACK` and the number of earlier
  acknowledged bytes.

CLI usage:

```bash
./host/usb_client/smbus-usb i2c_write_ext 0x50 I2C_NO_FLAGS 0x01 0x02
```

Example result:

```text
I2C write ext: status=0 (OK) num_written=2 requested=2
```

### `i2c_read_ext`

Performs an I2C controller/master read and returns detailed transaction status,
the received bytes, and the number actually read.

```c
status, data, num_read = i2c_read_ext(
    uint8_t device,
    I2C_NO_FLAGS,
    uint16_t length
);
```

- `length` must be 1 to 509 bytes.
- `num_read` is initialized to zero before the transaction.
- On success, `num_read` equals `length`.
- If a bus failure occurs after receiving data, the partial data and actual
  byte count are returned with the failure status.

CLI usage:

```bash
./host/usb_client/smbus-usb i2c_read_ext 0x50 I2C_NO_FLAGS 16
./host/usb_client/smbus-usb i2c_read_ext 0x50 0 16
```

Example result:

```text
I2C read ext: status=0 (OK) num_read=4 requested=4
10 20 30 40
```

### I2C transaction queues

The host API can collect controller/master transactions and execute them in
order through one open adapter. Queue handles follow the same convention as
adapter handles: positive values are valid handles and negative values are
errors.

```c
uint8_t offset = 0x00;
uint8_t buffer[16];

I2cQueue queue = queue_create(handle);
if (queue < 0) {
    /* handle error */
}

if (queue_i2c_write(queue, 0x50, I2C_NO_FLAGS, 1, &offset) < 0 ||
    queue_i2c_read(queue, 0x50, I2C_NO_FLAGS,
                   sizeof(buffer), buffer) < 0) {
    /* handle error */
}

int completed = queue_submit(queue);
queue_destroy(queue);
```

Queue lifecycle and signatures, in required order:

```c
I2cQueue queue_create(Aardvark handle);
int queue_i2c_write(I2cQueue queue, u16 slave_address,
                    AardvarkI2cFlags flags, u16 num_bytes,
                    const u08 *data);
int queue_i2c_read(I2cQueue queue, u16 slave_address,
                   AardvarkI2cFlags flags, u16 num_bytes, u08 *data);
int queue_submit(I2cQueue queue);
int async_abort(I2cQueue queue);
int queue_clear(I2cQueue queue);
int queue_destroy(I2cQueue queue);
```

- `queue_create(handle)` binds a host-side queue to an open adapter and
  returns a positive `I2cQueue` handle.
- `queue_i2c_write()` copies the supplied bytes, so the source buffer may be
  released or changed after the call.
- `queue_i2c_read()` retains the destination pointer. Its buffer must be valid
  during every submission that executes the queued read.
- A queue holds up to 255 commands. `queue_submit()` executes them in order
  and returns the number completed, or the first negative transaction error.
- Submitted commands remain in the queue and may be submitted again. Use
  `queue_clear()` to remove them or `queue_destroy()` to release the handle.
- `async_abort(queue)` is safe to call from another thread while
  `queue_submit()` is running. It returns immediately and makes submission
  stop with `STATUS_ABORTED` before the next queued transaction.
- An I2C transaction already in progress cannot be interrupted safely; abort
  takes effect at the next transaction boundary.
- Closing an adapter automatically destroys its idle queues. Closing returns
  `STATUS_QUEUE_BUSY` while one of its queues is submitting.
- Queue mutation, clearing, destruction, and concurrent submission return
  `STATUS_QUEUE_BUSY` while submission is active. Invalid or exhausted queue
  handles return `STATUS_INVALID_QUEUE` or `STATUS_QUEUE_FULL`.

The queue is implemented in the host library and uses the existing I2C USB
operations, so no additional firmware opcode is required.

### `i2c_slave_enable`

Enables the adapter's addressed I2C/SMBus slave mode.

```c
applied_tx, applied_rx = i2c_slave_enable(
    uint8_t slave_addr,
    uint16_t max_tx_bytes,
    uint16_t max_rx_bytes
);
```

Arguments:

- `slave_addr`: 7-bit address to which the adapter responds.
- `max_tx_bytes`: 0 selects the default maximum of 256 bytes; otherwise the
  accepted range is 1 to 256.
- `max_rx_bytes`: 0 selects the default maximum of 258 bytes; otherwise the
  accepted range is 1 to 258.

The operation returns the address and effective transmit/receive limits.
Calling it also clears previously latched asynchronous event flags.

CLI usage:

```bash
./host/usb_client/smbus-usb i2c_slave_enable 0x20 0 0
./host/usb_client/smbus-usb i2c_slave_enable 0x20 128 128
```

Example result:

```text
I2C slave enabled: address=0x20 max_tx=256 max_rx=258
```

### `i2c_slave_write`

Preloads the response transmitted when an external I2C controller reads this
adapter in slave mode.

```c
const uint8_t response[] = { 0x10, 0x20, 0x30 };
int accepted = i2c_slave_write(handle, sizeof(response), response);
```

- The response must contain 1 to 256 bytes.
- Returns the number of response bytes accepted by the firmware.
- The response repeats cyclically if the external controller requests more
  bytes than the stored response contains.
- Call `i2c_slave_enable()` first to select the slave address and transmit
  limit. Until a response is loaded, slave reads return `0xFF`.
- Returns `STATUS_INVALID_HANDLE`, `STATUS_COMMUNICATION_ERROR`, or
  `I2C_SLAVE_WRITE_ERROR` on failure.

CLI usage:

```bash
./host/usb_client/smbus-usb i2c_slave_write 0x10 0x20 0x30
```

An external controller reading the configured slave address will receive
`10 20 30 10 20 30 ...` for as many bytes as it requests.

### `async_poll`

Waits for I2C slave-mode activity and returns a bitmask of pending events.

```c
uint8_t flags = async_poll(int32_t timeout_ms);
```

Timeout values:

- `0`: return immediately.
- `1` to `60000`: wait up to that many milliseconds.
- `-1`: wait indefinitely.

Event flags may be combined:

| Flag | Value | Meaning |
|---|---:|---|
| `ASYNC_NO_DATA` | `0x00` | No event was pending before timeout |
| `ASYNC_I2C_READ` | `0x01` | An external controller wrote data to this adapter |
| `ASYNC_I2C_WRITE` | `0x02` | An external controller read data from this adapter |

Polling is non-consuming. `ASYNC_I2C_READ` remains set while raw slave
receive frames are queued and is cleared after `i2c_slave_read_ext` consumes
the last one. Calling `i2c_slave_enable` again clears all event flags and
queued raw frames.

CLI usage:

```bash
./host/usb_client/smbus-usb async_poll 0
./host/usb_client/smbus-usb async_poll 5000
./host/usb_client/smbus-usb async_poll -1
```

Example result:

```text
async flags: 0x01 I2C_READ
```

### `i2c_slave_read_ext`

Retrieves one completed write that an external I2C controller/master sent to
the adapter while addressed-slave mode was enabled.

```c
status, addr, data, num_read = i2c_slave_read_ext(
    uint16_t buffer_size
);
```

- Call `i2c_slave_enable` first to select the adapter's slave address and
  receive limit.
- `buffer_size` must be 1 to 258 bytes.
- `addr` is the 7-bit slave address on which the transaction was received.
  General-call reception is not enabled, so this is currently the address
  most recently configured by `i2c_slave_enable`.
- `data` contains up to `buffer_size` bytes from one complete external-master
  write transaction.
- `num_read` is the number of bytes copied into `data`. A valid zero-byte
  external-master write returns `num_read=0`.
- `status` is normally `I2C_STATUS_OK`; a malformed START, STOP, or RESTART
  observed by the slave returns `I2C_STATUS_BUS_ERROR` and may include a
  partial frame.
- If the received transaction is larger than `buffer_size`, only
  `buffer_size` bytes are returned and the remaining bytes from that queued
  transaction are discarded.
- Up to four raw transactions are queued in FIFO order. If the queue is full,
  a newly arriving raw transaction is dropped. MCTP reception uses its own
  queue and is not consumed or corrupted by this API.
- If no transaction is queued, the call waits up to 500 ms and then returns
  the USB-level `USB_STATUS_I2C_TIMEOUT` with no result payload.

CLI usage:

```bash
./host/usb_client/smbus-usb i2c_slave_enable 0x20 0 0
./host/usb_client/smbus-usb async_poll 5000
./host/usb_client/smbus-usb i2c_slave_read_ext 258
```

Example result after an external master writes `0x10 0x20 0x30` to `0x20`:

```text
I2C slave read ext: status=0 (OK) address=0x20 num_read=3 buffer_size=258
10 20 30
```

### `smbus_write`

Performs an SMBus block-style write containing a command byte, byte count,
and data.

```c
smbus_write(
    uint8_t device,
    uint8_t command,
    const uint8_t *data,
    uint8_t length
);
```

- `device` is a 7-bit address.
- `command` is one byte from `0x00` to `0xFF`.
- `length` must be 1 to 32 bytes.
- The firmware transmits `command`, `length`, then the data bytes.
- Success returns an empty payload and `USB_STATUS_OK`.

CLI usage:

```bash
./host/usb_client/smbus-usb smbus_write 0x6A 0x01 0x12 0x34
```

Example result:

```text
SMBus write: OK
```

### `smbus_read`

Performs the project's legacy raw controller read operation.

```c
data = smbus_read(uint8_t device, uint8_t length);
```

- `device` is a 7-bit address.
- `length` must be 1 to 32 bytes.
- The returned payload contains exactly `length` raw bytes on success.
- This operation does not send a command/register byte first.
- It is retained for compatibility and internally uses the extended receive
  engine. Use `i2c_read_ext` when detailed status or larger reads are needed.

CLI usage:

```bash
./host/usb_client/smbus-usb smbus_read 0x6A 32
```

### `mctp_send_receive`

Sends one complete MCTP packet over SMBus and waits for its response.

```c
source_addr, response_packet = mctp_send_receive(
    uint8_t destination_smbus_address,
    uint32_t timeout_ms,
    const uint8_t *request_packet,
    uint16_t request_length
);
```

- The destination is a 7-bit SMBus address.
- `timeout_ms` is 0 to 60000 milliseconds. A value of 0 uses the MCTP
  transport's configured default timeout.
- Packet length must be 4 to 256 bytes.
- The packet begins with the four-byte MCTP transport header: header version,
  destination EID, source EID, and flags/sequence/tag.
- Do not include the SMBus command, byte count, source-address byte, or PEC;
  the firmware adds and validates those fields.
- The return contains the response source SMBus address and complete MCTP
  packet beginning with its transport header.

CLI usage:

```bash
./host/usb_client/smbus-usb mctp_send_receive \
  0x6A 5000 0x01 0x08 0x09 0xC8
```

Example result:

```text
source SMBus address: 0x6A
MCTP packet: 01 09 08 C8 ...
```

### `adapter_close`

Releases an adapter and its host resources.

```c
int closed = adapter_close(handle);
```

- Pass a positive handle to close one adapter; success returns 1.
- Pass 0 to close every open adapter; success returns the number closed.
- Idle queues bound to a closed adapter are destroyed automatically.
- `STATUS_QUEUE_BUSY` is returned without closing anything if a relevant
  queue submission is active. Call `async_abort()`, wait for
  `queue_submit()` to return, and then close again.
- A closed handle and its borrowed libusb pointer must not be reused.

## USB diagnostic APIs

### `ping`

Checks that the adapter is responding to framed USB requests.

```bash
./host/usb_client/smbus-usb ping
```

Returns the four ASCII bytes `PONG`.

### `info`

Returns the firmware's adapter capability string.

```bash
./host/usb_client/smbus-usb info
```

Current result:

```text
STM32F411 SMBus Adapter;protocol=1;max_payload=512;usb=full-speed;adapter=i2c,smbus,mctp
```

### `echo`

Returns the supplied payload unchanged. The CLI form accepts a single text
argument; the raw protocol accepts any binary payload from 0 to 512 bytes.

```bash
./host/usb_client/smbus-usb echo "hello"
```

### `selftest`

Runs `ping`, `info`, and echo tests across USB payload boundaries at lengths
0, 1, 2, 63, 64, 65, 127, 128, 255, 256, 511, and 512 bytes.

```bash
./host/usb_client/smbus-usb selftest
```

This tests USB framing and transport only; it does not test the external I2C
bus or connected I2C devices.

## Raw USB protocol

Applications may use the CLI as a reference implementation or communicate
directly over the vendor bulk endpoints. The shared definitions are in
`protocol/usb_protocol.h`.

USB bulk packet boundaries are not protocol message boundaries. A receiver
must accumulate the 16-byte header and then exactly `payload_length` bytes.
All multi-byte integers are little-endian.

### Message header

| Offset | Size | Field | Request value |
|---:|---:|---|---|
| 0 | 4 | Magic | `0x41424D53` (`SMBA` bytes on the wire) |
| 4 | 1 | Protocol version | `1` |
| 5 | 1 | Message type | `1` for request, `2` for response |
| 6 | 2 | Opcode | Operation identifier |
| 8 | 4 | Sequence | Caller-selected request identifier |
| 12 | 2 | Payload length | 0 to 512 |
| 14 | 2 | Status | Set to 0 in requests; response contains `UsbStatus` |

The response repeats the request opcode and sequence number.

### Opcode and payload reference

Notation: `u8`, `u16`, `u32`, and `i32` are little-endian integer fields;
`bytes[N]` is an N-byte array.

| Opcode | API | Request payload | Successful response payload |
|---:|---|---|---|
| `0x0001` | `ping` | empty | ASCII `PONG` |
| `0x0002` | `info` | empty | UTF-8 capability string |
| `0x0003` | `echo` | `bytes[0..512]` | Identical bytes |
| `0x0100` | `i2c_detect` | empty | `address_bitmap[16]` |
| `0x0101` | `smbus_read` | `address:u8, length:u8` | `data[length]` |
| `0x0102` | `smbus_write` | `address:u8, command:u8, length:u8, data[length]` | empty |
| `0x0103` | `i2c_bitrate` | `bitrate_hz:u32` | `applied_bitrate_hz:u32` |
| `0x0104` | `i2c_slave_enable` | `address:u8, max_tx:u16, max_rx:u16` | `address:u8, applied_tx:u16, applied_rx:u16` |
| `0x010A` | `i2c_slave_write` | `length:u16, data[length]` | `accepted_length:u16` |
| `0x0105` | `async_poll` | `timeout_ms:i32` | `pending_flags:u8` |
| `0x0106` | `i2c_write` | `address:u8, flags:u16, length:u16, data[length]` | `num_written:u16` |
| `0x0107` | `i2c_write_ext` | `address:u8, flags:u16, length:u16, data[length]` | `i2c_status:u8, num_written:u16` |
| `0x0108` | `i2c_read_ext` | `address:u8, flags:u16, length:u16` | `i2c_status:u8, num_read:u16, data[num_read]` |
| `0x0109` | `i2c_slave_read_ext` | `buffer_size:u16` | `i2c_status:u8, address:u8, num_read:u16, data[num_read]` |
| `0x0200` | `mctp_send_receive` | `destination:u8, timeout_ms:u32, packet[4..256]` | `source:u8, packet[4..256]` |

Extended I2C operations intentionally return `USB_STATUS_OK` for a
well-formed request even when the bus transaction fails. Inspect the first
response-payload byte for the detailed I2C result and use `num_written` or
`num_read` to determine partial progress.

## Source reference

- USB protocol constants: `protocol/usb_protocol.h`
- Firmware USB dispatcher: `middleware/usb/usb_protocol.c`
- Adapter service API: `middleware/adapter/adapter_service.h`
- I2C driver API: `drivers/i2c.h`
- Host CLI: `host/usb_client/main.c`
- Adapter discovery API: `host/usb_client/adapter_discovery.h`
