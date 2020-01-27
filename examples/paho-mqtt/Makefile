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
USEMODULE += lwip_ipv6
USEMODULE += lwip_ipv6_autoconfig
# including lwip_ipv6_mld would currently break this test on at86rf2xx radios
USEMODULE += lwip lwip_sock_ip lwip_netdev
USEMODULE += lwip_tcp lwip_sock_tcp
USEMODULE += inet_csum
USEMODULE += shell
USEMODULE += od
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += netdev_default

CFLAGS += -DSO_REUSE
CFLAGS += -DLWIP_SO_RCVTIMEO
CFLAGS += -DLWIP_SOCK_TCP_ACCEPT_TIMEOUT=500
CFLAGS += -DLWIP_NETIF_LOOPBACK=0
CFLAGS += -DLWIP_HAVE_LOOPIF=0
ifeq ($(BOARD),native)
  USEMODULE += lwip_ethernet
endif

include $(RIOTBASE)/Makefile.include
