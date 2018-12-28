import time
from random import randint

def wait_for(client, msgType, period=0.25):
    if msgType == "SUBACK":
        if client.on_subscribe:
            while not client.suback_flag:
                print("waiting suback")
                client.loop()  # check for messages
                time.sleep(period)

def start_connection_client(client, server, port, connect_topic):
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.connect(server, port, keepalive=6)
    client.connect_topic =connect_topic
    client.subscribe(connect_topic, qos=2)

def on_connect(client, user_data, flags, rc):
    client.loop_start()
    if rc == 0:
        client.connected_flag = True  # set flag
        print("connected OK")
    else:
        print("Bad connection Returned code=", rc)

def on_message(client, user_data, message):
    if message.topic == client.connect_topic:
        print("Platform %s is connected." % str(message.payload.decode("utf-8")))
    print("message: ", str(message.payload.decode("utf-8")))
    print("message topic: ", message.topic)
    print("message qos: ", message.qos)
    print("message retain flag: ", message.retain)

def on_disconnect(client, user_rdata, rc=0):
    print("DisConnected result code " + str(rc))
    client.loop_stop()
    client.disconnect()

def send_rand_msg(client, platform_lst, msg_topic, qos):
    for platform in platform_lst:
        msg = "Test msg " + str(randint(50, 100))
        topic = msg_topic + str(platform)
        client.publish(topic, msg, qos=qos)

def send_angle(client, platform_id, msg_topic, qos, angle):
    msg = str(angle)
    topic = msg_topic + str(platform_id)
    client.publish(topic, msg, qos=qos)