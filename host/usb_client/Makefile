CC ?= cc
LIBUSB_PREFIX ?= $(shell brew --prefix libusb 2>/dev/null)

CFLAGS := -std=c11 -Wall -Wextra -Werror -O2 \
          -pthread \
          -I../../protocol \
          -I$(LIBUSB_PREFIX)/include/libusb-1.0
LDFLAGS := -L$(LIBUSB_PREFIX)/lib
LDLIBS := -lusb-1.0 -pthread

TARGET := smbus-usb
TEST_TARGET := adapter-queue-test

.PHONY: all clean test

all: $(TARGET)

$(TARGET): main.c adapter_discovery.c adapter_discovery.h ../../protocol/usb_protocol.h
	$(CC) $(CFLAGS) main.c adapter_discovery.c $(LDFLAGS) $(LDLIBS) -o $@

test: $(TEST_TARGET)
	./$(TEST_TARGET)

$(TEST_TARGET): tests/adapter_queue_test.c adapter_discovery.c adapter_discovery.h ../../protocol/usb_protocol.h
	$(CC) $(CFLAGS) tests/adapter_queue_test.c adapter_discovery.c -pthread -o $@

clean:
	rm -f $(TARGET) $(TEST_TARGET)
	rm -rf $(TARGET).dSYM $(TEST_TARGET).dSYM
