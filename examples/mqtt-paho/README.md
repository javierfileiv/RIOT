### About
This application demonstrates the usage of the MQTT paho package in RIOT.

### Setup
For using this example, two prerequisites have to be fulfilled:

1. You need a running MQTT broker like Mosquitto broker for example. Take a look at 
[Mosquitto Broker](https://mosquitto.org/). Check online any guide that will help you setting up the broker into some port (a).
For example this one for debian base linux users [How to setup a Mosquitto MQTT Server and receive data](https://www.digitalocean.com/community/questions/how-to-setup-a-mosquitto-mqtt-server-and-receive-data-from-owntracks).

2. Your RIOT node needs to be able to speak to that broker at the same port you set in 1.

### Setting up RIOT `native`
When running the example

- To connect to a broker, use the `con` command:
```
con fec0:affe::1 1885 <clientID> <username passwd> <keepalivetime>
```
clientID: is the client id you set up on the broker.
username and passwd: those set in the broker, check online tutorial to do it regarding chosen broker.
keepalivetime: keep alive in second for your client.

- To subscribe to a topic, run `sub` with the topic name as parameter and a QoS level between 1 to 3 (optional), e.g.
```
sub hello/world [2]
```
- For publishing, use the `pub` command (with a QoS level between 1 to 3 (optional)):
```
pub hello/world "One more beer, please." [2]
```