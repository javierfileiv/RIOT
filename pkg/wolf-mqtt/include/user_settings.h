/*
 * Copyright (C) 2021 Javier FILEIV <javier.fileiv@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup      pkg_wolf_mqtt
 * @ingroup      config
 * @defgroup     
 *
 * @brief        User setting for WolfMQTT package
 *               
 * @{
 *
 * @file        user_settings.h
 * @brief       WolfMQTT user settins
 *
 * @author      Javier FILEIV <javier.fileiv@gmail.com>
 */
#include <stdint.h>
#include "mutex.h"
// #include "xtimer.h"
// #include "thread.h"
// #include "net/sock/tcp.h"

#ifndef WOLF_MQTT_USER_SETTINGS_H
#define WOLF_MQTT_USER_SETTINGS_H


#undef WOLFMQTT_NONBLOCK
#undef WOLFMQTT_SN
#undef __FreeBSD__
#undef __linux__

#define WOLFMQTT_MULTITHREAD
#define WOLFMQTT_USER_THREADING

#define WOLFMQTT_CUSTOM_STRING
#define WOLFMQTT_CUSTOM_MALLOC

// #define WOLFMQTT_V5
// #define WOLFMQTT_PACK  

#ifdef __cplusplus
extern "C" {
#endif


/* System */
#define WOLFMQTT_CUSTOM_TYPES
/* Basic Types */
#ifndef byte
    typedef uint8_t byte;
#endif
#ifndef word16
    typedef uint16_t word16;
#endif
#ifndef word32
    typedef uint32_t word32;
#endif

#ifdef WOLFMQTT_MULTITHREAD

typedef mutex_t wm_Sem;
#endif

#define XMEMSET     memset
#define XMEMCPY     memcpy
#define XSTRLEN     strlen

#ifdef __cplusplus
}
#endif
/** @} */
#endif /* WOLF_MQTT_USER_SETTINGS_H */