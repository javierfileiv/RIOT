APPLICATION = paho-mqtt-example

# If no BOARD is found in the environment, use this default:
BOARD ?= native

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

#####################
USEPKG += paho-mqtt
USEMODULE += ipv6_addr
USEMODULE += lwip_ipv6
USEMODULE += lwip_ipv6_autoconfig
USEMODULE += lwip lwip_sock_ip lwip_netdev
USEMODULE += lwip_udp lwip_sock_udp
USEMODULE += lwip_tcp lwip_sock_tcp
USEMODULE += lwip_sock_async
USEMODULE += sock_async_event
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += netdev_default

ifeq ($(BOARD),native)
  USEMODULE += lwip_ethernet
endif

LOG_LEVEL ?= LOG_INFO
CFLAGS += -DSO_REUSE
CFLAGS += -DLWIP_SO_RCVTIMEO
CFLAGS += -DLWIP_SOCK_TCP_ACCEPT_TIMEOUT=500
CFLAGS += -DLWIP_NETIF_LOOPBACK=1
CFLAGS += -DLWIP_HAVE_LOOPIF=0

CFLAGS += -DDEFAULT_MQTT_BROKER_IP=\"fe80::d09d:72ff:fef7:88ba\"

include $(RIOTBASE)/Makefile.include
