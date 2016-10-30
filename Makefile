#!/usr/bin/make -f

include Makefile.mk

# --------------------------------------------------------------

PREFIX  ?= /usr/local
DESTDIR ?=
BUILDDIR ?= build/bolliedelay.lv2

# --------------------------------------------------------------
# Default target is to build all plugins

all: build
build: bolliedelay

# --------------------------------------------------------------
# bolliedelay build rules

bolliedelay: $(BUILDDIR) $(BUILDDIR)/bolliedelay$(LIB_EXT) $(BUILDDIR)/manifest.ttl $(BUILDDIR)/modgui.ttl $(BUILDDIR)/bolliedelay.ttl $(BUILDDIR)/modgui

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(BUILDDIR)/bolliedelay$(LIB_EXT): src/bollie-delay.c
	$(CC) $^ $(BUILD_C_FLAGS) $(LINK_FLAGS) -lm $(SHARED) -o $@

$(BUILDDIR)/manifest.ttl: lv2ttl/manifest.ttl.in
	sed -e "s|@LIB_EXT@|$(LIB_EXT)|" $< > $@

$(BUILDDIR)/modgui.ttl: lv2ttl/modgui.ttl.in
	sed -e "s|@LIB_EXT@|$(LIB_EXT)|" $< > $@

$(BUILDDIR)/bolliedelay.ttl: lv2ttl/bolliedelay.ttl
	cp $< $@

$(BUILDDIR)/modgui: modgui
	mkdir -p $@ 
	cp -rv $^/* $@/

# --------------------------------------------------------------

clean:
	rm -f $(BUILDDIR)/bolliedelay$(LIB_EXT) $(BUILDDIR)/*.ttl
	rm -fr $(BUILDDIR)/modgui

# --------------------------------------------------------------

install: build
	echo "Install"
	install -d $(DESTDIR)$(PREFIX)/lib/lv2/bolliedelay.lv2
	install -d $(DESTDIR)$(PREFIX)/lib/lv2/bolliedelay.lv2/modgui

	install -m 644 $(BUILDDIR)/*.so  $(DESTDIR)$(PREFIX)/lib/lv2/bolliedelay.lv2/
	install -m 644 $(BUILDDIR)/*.ttl $(DESTDIR)$(PREFIX)/lib/lv2/bolliedelay.lv2/
	cp -rv $(BUILDDIR)/modgui/* $(DESTDIR)$(PREFIX)/lib/lv2/bolliedelay.lv2/modgui/

# --------------------------------------------------------------
