/*
 * Copyright (C) 2019  Javier FILEIV <javier.fileiv@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @brief       MQTT common RIOT interface functions
 *
 * @author      Javier FILEIV <javier.fileiv@gmail.com>
 */

#include <assert.h>
#include <string.h>
#include <errno.h>

#ifdef MODULE_IPV6_ADDR
#include "net/ipv6/addr.h"
#endif
#ifdef MODULE_IPV4_ADDR
#include "net/ipv4/addr.h"
#endif
#include "net/sock/tcp.h"
#include "wolfmqtt.h"
#include "wolfmqtt/mqtt_client.h"
#include "xtimer.h"
#include "tsrb.h"
#include "log.h"

#define ENABLE_DEBUG        0
#include "debug.h"

#define IP_MAX_LEN_ADDRESS  (39)    /*IPv6 max length */

#ifndef TSRB_MAX_SIZE
#define TSRB_MAX_SIZE       (1024)
#endif

#ifdef MODULE_LWIP
static uint8_t buffer[TSRB_MAX_SIZE];
static uint8_t _temp_buf[TSRB_MAX_SIZE];
static tsrb_t tsrb_lwip_tcp;
#endif

#ifndef PAHO_MQTT_YIELD_MS
#define PAHO_MQTT_YIELD_MS  (10)
#endif

static int mqtt_read(struct Network *n, unsigned char *buf, int len,
                     int timeout_ms)
{
    int _timeout;
    int _len;
    void *_buf;
    int rc = -1;

    if (IS_USED(MODULE_LWIP)) {
        /* As LWIP doesn't support packet reading byte per byte and
         * PAHO MQTT reads like that to decode it on the fly,
         * we read TSRB_MAX_SIZE at once and keep them in a ring buffer.
         */
        _buf = _temp_buf;
        _len = TSRB_MAX_SIZE;
        _timeout = 0;
    }
    else {
        _buf = buf;
        _len = len;
        _timeout = timeout_ms;
    }

    uint64_t send_tick = xtimer_now64().ticks64 +
            xtimer_ticks_from_usec64(timeout_ms * US_PER_MS).ticks64;
    do {
        rc = sock_tcp_read(&n->sock, _buf, _len, _timeout);
        if (rc == -EAGAIN) {
            rc = 0;
        }

        if (IS_USED(MODULE_LWIP)) {
            if (rc > 0) {
                tsrb_add(&tsrb_lwip_tcp, _temp_buf, rc);
            }

            rc = tsrb_get(&tsrb_lwip_tcp, buf, len);
        }
    } while (rc < len && xtimer_now64().ticks64 < send_tick && rc >= 0);

    if (IS_ACTIVE(ENABLE_DEBUG) && IS_USED(MODULE_LWIP) && rc > 0) {
        DEBUG("MQTT buf asked for %d, available to read %d\n",
                rc, tsrb_avail(&tsrb_lwip_tcp));
        for (int i = 0; i < rc; i++) {
            DEBUG("0x%02X ", buf[i]);
        }
        DEBUG("\n");
    }

    return rc;
}

static int mqtt_write(struct Network *n, unsigned char *buf, int len,
                      int timeout_ms)
{
    /* timeout is controlled by upper layer in PAHO */
    (void) timeout_ms;

    return sock_tcp_write(&n->sock, buf, len);
}

void NetworkInit(Network *n)
{
    if (IS_USED(MODULE_LWIP)) {
        tsrb_init(&tsrb_lwip_tcp, buffer, TSRB_MAX_SIZE);
    }
    n->mqttread = mqtt_read;
    n->mqttwrite = mqtt_write;
}

int NetworkConnect(Network *n, char *addr_ip, int port)
{
    int ret =-1;
    sock_tcp_ep_t remote = SOCK_IPV4_EP_ANY;
    char _local_ip[IP_MAX_LEN_ADDRESS];

    strncpy(_local_ip, addr_ip, sizeof(_local_ip));
    if (IS_USED(MODULE_IPV4_ADDR) &&
        ipv4_addr_from_str((ipv4_addr_t *)&remote.addr, _local_ip)) {
            remote.port = port;
    }

    /* ipvN_addr_from_str modifies input buffer */
    strncpy(_local_ip, addr_ip, sizeof(_local_ip));
    if (IS_USED(MODULE_IPV6_ADDR) && (remote.port == 0)  &&
        ipv6_addr_from_str((ipv6_addr_t *)&remote.addr, _local_ip)) {
            remote.port = port;
            remote.family = AF_INET6;
    }

    if (remote.port == 0) {
        LOG_ERROR("Error: unable to parse destination address\n");
        return ret;
    }

    ret = sock_tcp_connect(&n->sock, &remote, 0, 0);
    return ret;
}

void NetworkDisconnect(Network *n)
{
    sock_tcp_disconnect(&n->sock);
}

void TimerInit(Timer *timer)
{
    timer->set_ticks.ticks64 = 0;
    timer->ticks_timeout.ticks64 = 0;
}

char TimerIsExpired(Timer *timer)
{
    return (TimerLeftMS(timer) == 0);
}

void TimerCountdownMS(Timer *timer, unsigned int timeout_ms)
{
    timer->set_ticks = xtimer_now64();
    timer->ticks_timeout = xtimer_ticks_from_usec64(timeout_ms * US_PER_MS);
}

void TimerCountdown(Timer *timer, unsigned int timeout_s)
{
    TimerCountdownMS(timer, timeout_s * MS_PER_SEC);
}

int TimerLeftMS(Timer *timer)
{
    xtimer_ticks64_t diff_ticks = xtimer_diff64(xtimer_now64(),
            timer->set_ticks);  /* should be always greater than 0 */
    if (xtimer_less64(diff_ticks, timer->ticks_timeout)) {
        diff_ticks = xtimer_diff64(timer->ticks_timeout, diff_ticks);
        return (xtimer_usec_from_ticks64(diff_ticks) / US_PER_MS);
    }
    return 0;
}

void MutexInit(Mutex *mutex)
{
    mutex_init(&mutex->lock);
}

int MutexLock(Mutex *mutex)
{
    mutex_lock(&mutex->lock);
    return 0;
}

int MutexUnlock(Mutex *mutex)
{
    mutex_unlock(&mutex->lock);
    return 0;
}

void *mqtt_riot_run(void *arg)
{
    MQTTClient *client = (MQTTClient *)arg;
    assert(client);

    while (1) {
        int rc;
        MutexLock(&client->mutex);
        if ((rc = MQTTYield(client, PAHO_MQTT_YIELD_MS)) != 0) {
            LOG_DEBUG("riot_iface: error while MQTTYield()(%d)\n", rc);
        }
        MutexUnlock(&client->mutex);
        /* let other threads do their work */
        xtimer_msleep(MQTT_YIELD_POLLING_MS);
    }
    return NULL;
}

int ThreadStart(Thread *thread, void (*fn)(void *), void *arg)
{
    (void) fn;
    thread->pid = thread_create(thread->stack, sizeof(thread->stack),
                                MQTT_THREAD_PRIORITY,
                                THREAD_CREATE_STACKTEST, mqtt_riot_run, arg,
                                "paho_mqtt_riot");
    return thread->pid;
}



#include "wolfmqtt/mqtt_client.h"
#include "examples/mqttnet.h"
#include "examples/mqttexample.h"

// #elif defined(FREERTOS) && defined(WOLFSSL_LWIP)
    /* Scheduler includes. */
    #include "FreeRTOS.h"
    #include "task.h"
    #include "semphr.h"

    /* lwIP includes. */
    #include "lwip/api.h"
    #include "lwip/tcpip.h"
    #include "lwip/memp.h"
    #include "lwip/stats.h"
    #include "lwip/sockets.h"
    #include "lwip/netdb.h"

/* Local context for Net callbacks */
typedef enum {
    SOCK_BEGIN = 0,
    SOCK_CONN,
} NB_Stat;



typedef struct _SocketContext {
    SOCKET_T fd;
    NB_Stat stat;
    SOCK_ADDR_IN addr;
    MQTTCtx* mqttCtx;
} SocketContext;

/* Private functions */
/* -------------------------------------------------------------------------- */
/* FREERTOS TCP NETWORK CALLBACK EXAMPLE */
/* -------------------------------------------------------------------------- */
#ifdef FREERTOS_TCP

#ifndef WOLFMQTT_NO_TIMEOUT
    static SocketSet_t gxFDSet = NULL;
#endif
static int NetConnect(void *context, const char* host, word16 port,
    int timeout_ms)
{
    SocketContext *sock = (SocketContext*)context;
    uint32_t hostIp = 0;
    int rc = -1;
    MQTTCtx* mqttCtx = sock->mqttCtx;

    switch (sock->stat) {
    case SOCK_BEGIN:
        PRINTF("NetConnect: Host %s, Port %u, Timeout %d ms, Use TLS %d",
            host, port, timeout_ms, mqttCtx->use_tls);

        hostIp = FreeRTOS_gethostbyname_a(host, NULL, 0, 0);
        if (hostIp == 0)
            break;

        sock->addr.sin_family = FREERTOS_AF_INET;
        sock->addr.sin_port = FreeRTOS_htons(port);
        sock->addr.sin_addr = hostIp;

        /* Create socket */
        sock->fd = FreeRTOS_socket(sock->addr.sin_family, FREERTOS_SOCK_STREAM,
                                   FREERTOS_IPPROTO_TCP);

        if (sock->fd == FREERTOS_INVALID_SOCKET)
            break;

#ifndef WOLFMQTT_NO_TIMEOUT
        /* Set timeouts for socket */
        timeout_ms = pdMS_TO_TICKS(timeout_ms);
        FreeRTOS_setsockopt(sock->fd, 0, FREERTOS_SO_SNDTIMEO,
            (void*)&timeout_ms, sizeof(timeout_ms));
        FreeRTOS_setsockopt(sock->fd, 0, FREERTOS_SO_RCVTIMEO,
            (void*)&timeout_ms, sizeof(timeout_ms));
#else
        (void)timeout_ms;
#endif
        sock->stat = SOCK_CONN;

        FALL_THROUGH;
    case SOCK_CONN:
        /* Start connect */
        rc = FreeRTOS_connect(sock->fd, (SOCK_ADDR_IN*)&sock->addr,
                              sizeof(sock->addr));
        break;
    }

    return rc;
}

static int NetRead(void *context, byte* buf, int buf_len,
    int timeout_ms)
{
    SocketContext *sock = (SocketContext*)context;
    int rc = -1, timeout = 0;
    word32 bytes = 0;

    if (context == NULL || buf == NULL || buf_len <= 0) {
        return MQTT_CODE_ERROR_BAD_ARG;
    }

#ifndef WOLFMQTT_NO_TIMEOUT
    /* Create the set of sockets that will be passed into FreeRTOS_select(). */
    if (gxFDSet == NULL)
        gxFDSet = FreeRTOS_CreateSocketSet();
    if (gxFDSet == NULL)
        return MQTT_CODE_ERROR_OUT_OF_BUFFER;
    timeout_ms = pdMS_TO_TICKS(timeout_ms); /* convert ms to ticks */
#else
    (void)timeout_ms;
#endif

    /* Loop until buf_len has been read, error or timeout */
    while ((bytes < buf_len) && (timeout == 0)) {

#ifndef WOLFMQTT_NO_TIMEOUT
        /* set the socket to do used */
        FreeRTOS_FD_SET(sock->fd, gxFDSet, eSELECT_READ | eSELECT_EXCEPT);

        /* Wait for any event within the socket set. */
        rc = FreeRTOS_select(gxFDSet, timeout_ms);
        if (rc != 0) {
            if (FreeRTOS_FD_ISSET(sock->fd, gxFDSet))
#endif
            {
                /* Try and read number of buf_len provided,
                    minus what's already been read */
                rc = (int)FreeRTOS_recv(sock->fd, &buf[bytes],
                    buf_len - bytes, 0);

                if (rc <= 0) {
                    break; /* Error */
                }
                else {
                    bytes += rc; /* Data */
                }
            }
#ifndef WOLFMQTT_NO_TIMEOUT
        }
        else {
            timeout = 1;
        }
#endif
    }

    if (rc == 0 || timeout) {
        rc = MQTT_CODE_ERROR_TIMEOUT;
    }
    else if (rc < 0) {
    #ifdef WOLFMQTT_NONBLOCK
        if (rc == -pdFREERTOS_ERRNO_EWOULDBLOCK) {
            return MQTT_CODE_CONTINUE;
        }
    #endif
        PRINTF("NetRead: Error %d", rc);
        rc = MQTT_CODE_ERROR_NETWORK;
    }
    else {
        rc = bytes;
    }

    return rc;
}

static int NetWrite(void *context, const byte* buf, int buf_len, int timeout_ms)
{
    SocketContext *sock = (SocketContext*)context;
    int rc = -1;

    (void)timeout_ms;

    if (context == NULL || buf == NULL || buf_len <= 0) {
        return MQTT_CODE_ERROR_BAD_ARG;
    }

    rc = (int)FreeRTOS_send(sock->fd, buf, buf_len, 0);

    if (rc < 0) {
    #ifdef WOLFMQTT_NONBLOCK
        if (rc == -pdFREERTOS_ERRNO_EWOULDBLOCK) {
            return MQTT_CODE_CONTINUE;
        }
    #endif
        PRINTF("NetWrite: Error %d", rc);
        rc = MQTT_CODE_ERROR_NETWORK;
    }

    return rc;
}

static int NetDisconnect(void *context)
{
    SocketContext *sock = (SocketContext*)context;
    if (sock) {
        FreeRTOS_closesocket(sock->fd);
        sock->stat = SOCK_BEGIN;
    }

#ifndef WOLFMQTT_NO_TIMEOUT
    if (gxFDSet != NULL) {
        FreeRTOS_DeleteSocketSet(gxFDSet);
        gxFDSet = NULL;
    }
#endif

    return 0;
}