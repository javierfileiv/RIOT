/*
 * Copyright (C) 2021  Javier FILEIV <javier.fileiv@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @brief       WolfMQTT common RIOT interface functions
 *
 * @author      Javier FILEIV <javier.fileiv@gmail.com>
 */

#include <assert.h>
#include <string.h>
#include <errno.h>

#include "wolfmqtt_riot.h"
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

static int NetRead(void *context, byte* buf, int buf_len, int timeout_ms)
{
    SocketContext *sockCtx = (SocketContext*)context;
    int _timeout;
    int _len;
    void *_buf;
    int rc = -1;

    // TODO: timeout control   and   if (sockCtx) {

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
        _len = buf_len;
        _timeout = timeout_ms;
    }

    uint64_t send_tick = xtimer_now64().ticks64 +
            xtimer_ticks_from_usec64(timeout_ms * US_PER_MS).ticks64;
    do {
        rc = sock_tcp_read(&sockCtx->sock, _buf, _len, _timeout);
        if (rc == -EAGAIN) {
            rc = 0;
        }

        if (IS_USED(MODULE_LWIP)) {
            if (rc > 0) {
                tsrb_add(&tsrb_lwip_tcp, _temp_buf, rc);
            }

            rc = tsrb_get(&tsrb_lwip_tcp, buf, buf_len);
        }
    } while (rc < buf_len && xtimer_now64().ticks64 < send_tick && rc >= 0);

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

static int NetWrite(void *context, const byte* buf, int buf_len, int timeout_ms)
{
    // TODO: timeout control   and   if (sockCtx) {
    (void) timeout_ms;
    SocketContext *sockCtx = (SocketContext*)context;
    return sock_tcp_write(&sockCtx->sock, buf, buf_len);
}

static int NetConnect(void *context, const char* host, word16 port, int timeout_ms) {
    int ret =-1;
    sock_tcp_ep_t remote = SOCK_IPV4_EP_ANY;
    char _local_ip[IP_MAX_LEN_ADDRESS];
    SocketContext *sockCtx = (SocketContext*)context;
    
    // TODO: timeout control and   if (sockCtx) {
    (void) timeout_ms;

    strncpy(_local_ip, host, sizeof(_local_ip));
    if (IS_USED(MODULE_IPV4_ADDR) &&
        ipv4_addr_from_str((ipv4_addr_t *)&remote.addr, _local_ip)) {
            remote.port = port;
    }

    /* ipvN_addr_from_str modifies input buffer */
    strncpy(_local_ip, host, sizeof(_local_ip));
    if (IS_USED(MODULE_IPV6_ADDR) && (remote.port == 0)  &&
        ipv6_addr_from_str((ipv6_addr_t *)&remote.addr, _local_ip)) {
            remote.port = port;
            remote.family = AF_INET6;
    }

    if (remote.port == 0) {
        LOG_ERROR("Error: unable to parse destination address\n");
        return ret;
    }

    ret = sock_tcp_connect(&sockCtx->sock, &remote, 0, 0);
    return ret;
}

static int NetDisconnect(void *context)
{
    SocketContext *sockCtx = (SocketContext*)context;

    if (sockCtx) {
        sock_tcp_disconnect(&sockCtx->sock);
    }
    return 0;
}

/* Public Functions */
int MqttClientNet_Init(MqttNet* net, MQTTCtx* mqttCtx)
{
    int ret = MQTT_CODE_SUCCESS;

    if (IS_USED(MODULE_LWIP)) {
        tsrb_init(&tsrb_lwip_tcp, buffer, TSRB_MAX_SIZE);
    }

    if (net && mqttCtx) {
        XMEMSET(net, 0, sizeof(MqttNet));
        net->connect = NetConnect;
        net->read = NetRead;
        net->write = NetWrite;
        net->disconnect = NetDisconnect;
        net->context = &mqttCtx->sock;
    }
    else {
        ret = MQTT_CODE_ERROR_BAD_ARG;
    }

    return ret;
}

int wm_SemInit(wm_Sem *s){
    int ret = MQTT_CODE_SUCCESS;

    if (s) {
        mutex_t *lock = (mutex_t *)s; 
        mutex_init(lock);
    }
    else {
        ret = MQTT_CODE_ERROR_BAD_ARG;
    } 

    return ret;
}

int wm_SemFree(wm_Sem *s){
    /* no free */
    (void)s;
    return 0;
}

// int wm_SemLock(wm_Sem *s){
//     dispatch_semaphore_wait(*s, DISPATCH_TIME_FOREVER);
//     return 0;
// }
// int wm_SemUnlock(wm_Sem *s){
//     dispatch_semaphore_signal(*s);
//     return 0;
// }

// void TimerInit(Timer *timer)
// {
//     timer->set_ticks.ticks64 = 0;
//     timer->ticks_timeout.ticks64 = 0;
// }

// char TimerIsExpired(Timer *timer)
// {
//     return (TimerLeftMS(timer) == 0);
// }

// void TimerCountdownMS(Timer *timer, unsigned int timeout_ms)
// {
//     timer->set_ticks = xtimer_now64();
//     timer->ticks_timeout = xtimer_ticks_from_usec64(timeout_ms * US_PER_MS);
// }

// void TimerCountdown(Timer *timer, unsigned int timeout_s)
// {
//     TimerCountdownMS(timer, timeout_s * MS_PER_SEC);
// }

// int TimerLeftMS(Timer *timer)
// {
//     xtimer_ticks64_t diff_ticks = xtimer_diff64(xtimer_now64(),
//             timer->set_ticks);  /* should be always greater than 0 */
//     if (xtimer_less64(diff_ticks, timer->ticks_timeout)) {
//         diff_ticks = xtimer_diff64(timer->ticks_timeout, diff_ticks);
//         return (xtimer_usec_from_ticks64(diff_ticks) / US_PER_MS);
//     }
//     return 0;
// }

// void MutexInit(Mutex *mutex)
// {
//     mutex_init(&mutex->lock);
// }

// int MutexLock(Mutex *mutex)
// {
//     mutex_lock(&mutex->lock);
//     return 0;
// }

// int MutexUnlock(Mutex *mutex)
// {
//     mutex_unlock(&mutex->lock);
//     return 0;
// }

// void *mqtt_riot_run(void *arg)
// {
//     MQTTClient *client = (MQTTClient *)arg;
//     assert(client);

//     while (1) {
//         int rc;
//         MutexLock(&client->mutex);
//         if ((rc = MQTTYield(client, PAHO_MQTT_YIELD_MS)) != 0) {
//             LOG_DEBUG("riot_iface: error while MQTTYield()(%d)\n", rc);
//         }
//         MutexUnlock(&client->mutex);
//         /* let other threads do their work */
//         xtimer_msleep(MQTT_YIELD_POLLING_MS);
//     }
//     return NULL;
// }

// int ThreadStart(Thread *thread, void (*fn)(void *), void *arg)
// {
//     (void) fn;
//     thread->pid = thread_create(thread->stack, sizeof(thread->stack),
//                                 MQTT_THREAD_PRIORITY,
//                                 THREAD_CREATE_STACKTEST, mqtt_riot_run, arg,
//                                 "paho_mqtt_riot");
//     return thread->pid;
// }
