CC      ?= cc
CFLAGS  ?= -O2 -g -Wall -Wextra
USB_CFLAGS := $(shell pkg-config --cflags libusb-1.0)
USB_LIBS   := $(shell pkg-config --libs libusb-1.0)
PREFIX  ?= /usr/local

VERSION  = 0.1.0
SONAME   = libzk4500.so.0

LIB_OBJ  = build/zk4500.o
BINS     = build/zk4500_capture build/zkprobe build/zkctl

all: build/libzk4500.so build/libzk4500.a $(BINS)

build:
	mkdir -p build

build/zk4500.o: src/zk4500.c include/zk4500.h | build
	$(CC) $(CFLAGS) -fPIC -Iinclude $(USB_CFLAGS) -c $< -o $@

build/libzk4500.so: $(LIB_OBJ)
	$(CC) -shared -Wl,-soname,$(SONAME) -o $@ $^ $(USB_LIBS) -lm

build/libzk4500.a: $(LIB_OBJ)
	ar rcs $@ $^

build/zk4500_capture: examples/zk4500_capture.c build/libzk4500.a
	$(CC) $(CFLAGS) -Iinclude $< build/libzk4500.a $(USB_LIBS) -lm -o $@

build/zkprobe: tools/zkprobe.c | build
	$(CC) $(CFLAGS) $(USB_CFLAGS) $< $(USB_LIBS) -o $@

build/zkctl: tools/zkctl.c | build
	$(CC) $(CFLAGS) $(USB_CFLAGS) $< $(USB_LIBS) -o $@

install: all
	install -Dm644 include/zk4500.h $(DESTDIR)$(PREFIX)/include/zk4500.h
	install -Dm755 build/libzk4500.so $(DESTDIR)$(PREFIX)/lib/libzk4500.so.$(VERSION)
	ln -sf libzk4500.so.$(VERSION) $(DESTDIR)$(PREFIX)/lib/$(SONAME)
	ln -sf $(SONAME) $(DESTDIR)$(PREFIX)/lib/libzk4500.so
	install -Dm644 build/libzk4500.a $(DESTDIR)$(PREFIX)/lib/libzk4500.a
	install -Dm755 build/zk4500_capture $(DESTDIR)$(PREFIX)/bin/zk4500_capture
	install -Dm644 udev/60-zk4500.rules $(DESTDIR)/etc/udev/rules.d/60-zk4500.rules

clean:
	rm -rf build tools/zkprobe tools/zkctl

.PHONY: all install clean
