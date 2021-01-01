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
 * @brief        WolfMQTT config header. Replaces the one created by GNU autoconf tool.
 *               
 * @{
 *
 * @file
 * @brief       WolfMQTT user settings
 *
 * @author      Javier FILEIV <javier.fileiv@gmail.com>
 */

#ifdef MODULE_IPV6_ADDR
#include "net/ipv6/addr.h"
#endif
#ifdef MODULE_IPV4_ADDR
#include "net/ipv4/addr.h"
#endif
#include "net/sock/tcp.h"

#include "user_settings.h"
#include "wolfmqtt/mqtt_client.h" 

#ifdef __cplusplus
extern "C" {
#endif

/* MQTT Client state */
typedef enum _MQTTCtxState {
    WMQ_BEGIN = 0,
    WMQ_NET_INIT,
    WMQ_INIT,
    WMQ_TCP_CONN,
    WMQ_MQTT_CONN,
    WMQ_SUB,
    WMQ_PUB,
    WMQ_WAIT_MSG,
    WMQ_UNSUB,
    WMQ_DISCONNECT,
    WMQ_NET_DISCONNECT,
    WMQ_DONE
} MQTTCtxState;

/* MQTT Socket context */
typedef struct _SocketContext {
    sock_tcp_t sock;                /**< socket number */
} SocketContext;

/* MQTT Client context */
/* This is used for the examples as reference */
/* Use of this structure allow non-blocking context */
typedef struct _MQTTCtx {

    MQTTCtxState stat;
    
    /* client and net containers */
    MqttClient client;
    MqttNet net;
    SocketContext sock;

    /* temp mqtt containers */
    MqttConnect connect;
    MqttMessage lwt_msg;
    MqttSubscribe subscribe;
    MqttUnsubscribe unsubscribe;
    MqttTopic topics[1];
    MqttPublish publish;
    MqttDisconnect disconnect;
    MqttPing ping;
#ifdef WOLFMQTT_SN
    SN_Publish publishSN;
#endif

    /* configuration */
    MqttQoS qos;
    const char* app_name;
    const char* host;
    const char* username;
    const char* password;
    const char* topic_name;
    const char* message;
    const char* pub_file;
    const char* client_id;
    uint8_t *tx_buf, *rx_buf;
    int return_code;
    int use_tls;
    int retain;
    int enable_lwt;
#ifdef WOLFMQTT_V5
    int      max_packet_size;
#endif
    word32 cmd_timeout_ms;
#if defined(WOLFMQTT_NONBLOCK)
    word32  start_sec; /* used for keep-alive */
#endif
    word16 keep_alive_sec;
    word16 port;
#ifdef WOLFMQTT_V5
    word16  topic_alias;
    word16  topic_alias_max; /* Server property */
#endif
    uint8_t    clean_session;
    uint8_t    test_mode;
#ifdef WOLFMQTT_V5
    uint8_t    subId_not_avail; /* Server property */
    uint8_t    enable_eauth; /* Enhanced authentication */
#endif
    unsigned int dynamicTopic:1;
    unsigned int dynamicClientId:1;
#ifdef WOLFMQTT_NONBLOCK
    unsigned int useNonBlockMode:1; /* set to use non-blocking mode.
        network callbacks can return MQTT_CODE_CONTINUE to indicate "would block" */
#endif
} MQTTCtx;

/* Functions used to handle the MqttNet structure creation / destruction */
int MqttClientNet_Init(MqttNet* net, MQTTCtx* mqttCtx);
int MqttClientNet_DeInit(MqttNet* net);

#ifdef WOLFMQTT_SN
int SN_ClientNet_Init(MqttNet* net, MQTTCtx* mqttCtx);
#endif

int MqttClientNet_Wake(MqttNet* net);

#ifdef __cplusplus
}
#endif
/** @} */ 