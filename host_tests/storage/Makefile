# Host tests for the vendored HomeKit storage layer (lib/HomeKit-ESP8266).
#
# env/ is a shadow include directory: storage.c and its portable headers are
# symlinks to the shipping sources, while the four ESP-only headers (port,
# crypto, homekit_debug, lwipopts) are stubs. Compiling env/storage.c makes the
# compiler resolve its quoted includes against env/ first, which is the only way
# to substitute headers that also exist next to the source.
CC       ?= gcc
CXX      ?= g++
SAN       = -fsanitize=address,undefined
WARN      = -Wall -Wextra -g
INCLUDES  = -Ienv
# storage.c casts a linker symbol's address straight to uint32_t; harmless on the
# host and not ours to change.
CFLAGS    = $(WARN) $(SAN) $(INCLUDES) -Wno-pointer-to-int-cast
CXXFLAGS  = -std=c++17 $(WARN) $(SAN) $(INCLUDES)

OBJS = storage.o fake_flash.o test_storage.o

test_storage: $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $@

storage.o: env/storage.c
	$(CC) $(CFLAGS) -c $< -o $@

fake_flash.o: fake_flash.c
	$(CC) $(CFLAGS) -c $< -o $@

test_storage.o: test_storage.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: run clean
run: test_storage
	./test_storage

clean:
	rm -f $(OBJS) test_storage
