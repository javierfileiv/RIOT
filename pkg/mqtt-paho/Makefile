PKG_NAME = mqtt-paho
PKG_URL = https://github.com/eclipse/paho.mqtt.embedded-c.git
PKG_VERSION = 29ab2aa29c5e47794284376d7f8386cfd54c3eed
PKG_LICENSE = EDL

include $(RIOTBASE)/pkg/pkg.mk

.PHONY: mqtt-paho

all: mqtt-paho

mqtt-paho:
	"$(MAKE)" -C $(PKG_BUILDDIR)/MQTTPacket/src/ -f $(CURDIR)/Makefile.$(PKG_NAME)-packet
	"$(MAKE)" -C $(PKG_BUILDDIR)/MQTTClient-C/src/ -f $(CURDIR)/Makefile.$(PKG_NAME)
