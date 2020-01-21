/*
 * Copyright (C) 2019
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       MQTT-paho example
 *
 * @author      Javier FILEIV <javier.fileiv@gmail.com>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "thread.h"
#include "log.h"
#include "mqtt.h"
#include "msg.h"
#include "lwip.h"
#include "lwip/netif.h"
#include "shell.h"

#include "MQTTClient.h"

#define BUF_SIZE                250
#define KEEP_ALIVE_INTERVAL     3600
#define MQTT_VERSION            3
#define MQTT_DEFAULT_QOS        0
#define COMMAND_TIMEOUT_MS      1000
#define IS_CLEAN_SESSION        1
#define NOT_RETAINED_MSG        0

static MQTTClient client;
static Network network;
static unsigned char buf[BUF_SIZE];
static unsigned char readbuf[BUF_SIZE];


static unsigned get_qos(const char *str)
{
    int qos = atoi(str);

    switch (qos) {
        case 1:     return QOS1;
        case 2:     return QOS2;
        default:    return QOS0;
    }
}

static int mqtt_launch_thread(void)
{
    printf("Launching MQTT Task\n");
    return MQTTStartTask(&client);
}

static void _on_msg_received(MessageData *data)
{
    printf("mqtt_paho_example: message received on topic"
           " %.*s: %.*s\n\n",
           (int)data->topicName->lenstring.len,
           data->topicName->lenstring.data, (int)data->message->payloadlen,
           (char *)data->message->payload);
}

static int cmd_con(int argc, char **argv)
{
    if (argc < 7) {
        printf(
            "usage: %s <ipv6 addr> <port> <clientid> <user> <password> <KeepAliveInterval in sec>\n",
            argv[0]);
        return 1;
    }

    char *remote_ip = argv[1];
    int port = atoi(argv[2]);

    MQTTNetworkInit(&network);
    printf("Trying to connect to %s , port: %d\n",
           remote_ip, port);
    if (MQTTNetworkConnect(&network, remote_ip, port)) {
        printf("error: Unable to connect to %s:%d\n", remote_ip,
               port);
        return 1;
    }

    MQTTClientInit(&client, &network, COMMAND_TIMEOUT_MS, buf, BUF_SIZE, readbuf,
                   BUF_SIZE);

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = MQTT_VERSION;
    data.clientID.cstring = argv[3];
    data.username.cstring = argv[4];
    data.password.cstring = argv[5];
    data.keepAliveInterval = atoi(argv[6]);
    data.cleansession = IS_CLEAN_SESSION;
    printf("Connecting to %s %d\n", remote_ip, port);

    int ret = MQTTConnect(&client, &data);
    if (ret) {
        printf("error: Unable to connect client %d\n", ret);
    } else {
        printf("Connection successfully\n");
    }
    return ret;
}

static int cmd_discon(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    int res = MQTTDisconnect(&client);
    if (res) {
        puts("error: Unable to disconnect");
    }
    else {
        puts("Disconnect successful");
    }

    MQTTNetworkDisconnect(&network);
    return res;
}

static int cmd_pub(int argc, char **argv)
{
    enum QoS qos = QOS0;

    if (argc < 3) {
        printf("usage: %s <topic name> <string msg> [QoS level]\n",
               argv[0]);
        return 1;
    }
    if (argc == 4) {
        qos = get_qos(argv[3]);
    }
    MQTTMessage message;
    message.qos = qos;
    message.retained = NOT_RETAINED_MSG;
    message.payload = argv[2];
    message.payloadlen = strlen(message.payload);

    int rc;
    if ((rc = MQTTPublish(&client, argv[1], &message)) != 0) {
        printf("error: Unable to publish (%d)\n", rc);
    }
    else {
        printf("Message (%s) has been published to topic %s\n",
               (char *)message.payload, argv[1]);
    }

    return rc;
}

static int cmd_sub(int argc, char **argv)
{
    enum QoS qos = QOS0;

    if (argc < 2) {
        printf("usage: %s <topic name> [QoS level]\n", argv[0]);
        return 1;
    }

    if (argc >= 3) {
        qos = get_qos(argv[2]);
    }

    printf("Subscribing to %s\n", argv[1]);
    int ret = MQTTSubscribe(&client, argv[1], qos, _on_msg_received);
    printf("Subscribed %d\n", ret);

    if (!ret) {
        printf("error: Unable to subscribe to %s\n", argv[1]);
    }
    else {

        printf("Now subscribed to %s\n", argv[1]);
    }
    return ret;
}

static int cmd_unsub(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage %s <topic name>\n", argv[0]);
        return 1;
    }

    int ret = MQTTUnsubscribe(&client, argv[1]);

    if (!ret) {
        printf("Unsubscribed from topic:%s\n", argv[1]);
    }
    else {
        printf("error: Unable to unsubscribe from topic: %s\n", argv[1]);

    }
    return ret;
}

static const shell_command_t shell_commands[] =
{
    { "con", "connect to MQTT broker", cmd_con },
    { "discon", "disconnect from the current broker", cmd_discon },
    { "pub", "publish something", cmd_pub },
    { "sub", "subscribe topic", cmd_sub },
    { "unsub", "unsubscribe from topic", cmd_unsub },
    { NULL, NULL, NULL }
};

int main(void)
{
    /* Welcome message */
    printf("Running mqtt paho example. Type help for commands info\n");

    mqtt_launch_thread();
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    return 0;
}
