import paho.mqtt.client as mqtt
import configparser
import argparse
import logging
import time
import json
import sys


def main():
	logging.basicConfig(format='[%(asctime)s] %(levelname)s: %(message)s', level=logging.INFO)

	parser = argparse.ArgumentParser(description='Save LoRa uplink messages received through ' \
							'The Things Network to a file.')
	parser.add_argument('--app_id', '--username', type=str, help='TTN application ID')
	parser.add_argument('--access_key', '--password', type=str, help='TTN access key')
	parser.add_argument('--mqtt_server', '--server', type=str, help='MQTT server address',
				default='eu1.cloud.thethings.network:8883')
	parser.add_argument('--out', metavar='FILENAME', type=str,
				help='output filename (accepts strftime format string)',
				default='/dev/stdout')
	args = parser.parse_args()

	lastf = None
	def callback(_client, _userdata, msg):
		nonlocal lastf
		if not msg.topic.endswith("/up"):
			logging.info("message in topic %s: %s" % (msg.topic, msg.payload))
			return
		else:
			try:
				payload = json.loads(msg.payload)
			except Exception as e:
				logging.error(e)
				return
			try:
				logging.info("picked up uplink message from %s", payload["end_device_ids"]["device_id"])
			except:
				logging.info("picked up weird uplink message")
			try:
				fn = time.strftime(args.out)
				if lastf is None or lastf.name != fn:
					if lastf is not None:
						lastf.close()
					lastf = open(fn, "ab")
				lastf.write(msg.payload+b"\n")
				lastf.flush()
			except Exception as e:
				logging.error(e)

	def on_connect(client, userdata, flags, rc):
		if rc == 0:
			logging.info("connected")
			client.subscribe("#")
		else:
			logging.error("connection attempt failed (code: %d)" % rc)

	client = mqtt.Client()
	client.on_message = callback
	client.on_connect = on_connect
	client.tls_set()
	addr_pieces = args.mqtt_server.split(":", 2)
	host = addr_pieces[0]
	port = int(addr_pieces[1] if len(addr_pieces) == 2 else "8883")
	client.username_pw_set(args.app_id, args.access_key)
	client.connect(host, port, 60)
	client.loop_forever()


if __name__ == "__main__":
	main()
