PKG_NAME = wolf-mqtt 
PKG_URL = git@github.com:wolfSSL/wolfMQTT.git
PKG_VERSION = 238a70ec96a7501853b81241352c5091a86e9251 # v1.7
PKG_LICENSE = GPLv2

include $(RIOTBASE)/pkg/pkg.mk

all: wolf-mqtt

wolf-mqtt:
	echo "*********$(PKG_SOURCE_DIR)"
	"$(MAKE)" -C $(PKG_SOURCE_DIR)/wolf-MQTT/src -f $(CURDIR)/Makefile.wolfmqtt-src
