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
#include <string.h>
#include <net/ipv6/addr.h>
#include <net/af.h>
#include <net/sock.h>
#include <net/sock/tcp.h>
#include <mqtt.h>
#include <log.h>

#define ENABLE_DEBUG        (0)
#include "debug.h"

static int mqtt_read(struct Network *n, unsigned char *buf, int len,
                     int timeout_ms)
{
    return sock_tcp_read(&n->sock, buf, len, timeout_ms);
}

static int mqtt_write(struct Network *n, unsigned char *buf, int len,
                      int timeout_ms)
{
    (void)timeout_ms;
    return sock_tcp_write(&n->sock, buf, len);
}

static int mqtt_close(struct Network *n)
{
    sock_tcp_disconnect(&n->sock);
    return 0;
}

void NetworkInit(Network *n)
{
    n->mqttread = mqtt_read;
    n->mqttwrite = mqtt_write;
    n->disconnect = mqtt_close;
}

int NetworkConnect(Network *n, char *addr_ip, int port)
{
    sock_tcp_ep_t remote = SOCK_IPV6_EP_ANY;
    char _local_ip[25];

    strncpy(_local_ip, addr_ip, sizeof(_local_ip));
    remote.family = AF_INET6;
    if (ipv6_addr_from_str((ipv6_addr_t *)&remote.addr, _local_ip) == NULL) {
        puts("Error IPv6: unable to parse destination address, trying IPv4");
        /* ipv6_addr_from_str modifies the buffer */
        strncpy(_local_ip, addr_ip, sizeof(_local_ip));
        if (ipv4_addr_from_str((ipv4_addr_t *)&remote.addr,
                               _local_ip) == NULL) {
            puts("Error IPv4: unable to parse destination address");
            return -1;
        }
        remote.family = AF_INET;
    }
    remote.port = port;

    int ret = sock_tcp_connect(&n->sock, &remote, 0, 0);
    if (ret < 0) {
        LOG_ERROR("paho-mqtt: unable to connect (%d)\n", ret);
    }
    return ret;
}

void NetworkDisconnect(Network *n)
{
    sock_tcp_disconnect(&n->sock);
}

void TimerInit(Timer *timer)
{
    timer->timeout.ticks64 = 0;
}

char TimerIsExpired(Timer *timer)
{
    uint64_t ref = xtimer_now64().ticks64;
    int64_t diff = timer->timeout.ticks64 - ref;

    return (diff > 0) ? 0 : 1;
}

void TimerCountdownMS(Timer *timer, unsigned int timeout_ms)
{
    uint64_t ref = xtimer_now64().ticks64;
    timer->timeout.ticks64 = ref + (timeout_ms * US_PER_MS);
}

void TimerCountdown(Timer *timer, unsigned int timeout_s)
{
    TimerCountdownMS(timer, timeout_s * MS_PER_SEC);
}

int TimerLeftMS(Timer *timer)
{
    uint64_t ref = xtimer_now64().ticks64;
    int64_t diff = timer->timeout.ticks64 - ref;

    return ((diff < 0) ? 0 : diff / US_PER_MS);
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

int ThreadStart(Thread *thread, void *(*fn)(void *), void *arg)
{
    thread->pid = thread_create(thread->stack, sizeof(thread->stack),
                                MQTT_THREAD_PRIORITY,
                                THREAD_CREATE_STACKTEST, fn, arg,
                                "paho_mqtt_riot");
    return thread->pid;
}
