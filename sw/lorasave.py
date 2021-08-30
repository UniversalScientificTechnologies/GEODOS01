import ttn
import configparser
import argparse
import time
import json

def main():
	parser = argparse.ArgumentParser(description='Save LoRa uplink messages received through ' \
							'The Things Network to a file.')
	parser.add_argument('--app_id', type=str, help='TTN application ID')
	parser.add_argument('--access_key', type=str, help='TTN access key')
	parser.add_argument('--mqtt_server', type=str, help='MQTT server address',
				default='eu1.cloud.thethings.network:8883')
	parser.add_argument('--out', metavar='FILENAME', type=str,
				help='output filename (accepts strftime format string)',
				default='/dev/stdout')
	args = parser.parse_args()

	lastf = None
	def callback(msg, client):
		nonlocal lastf
		try:
			fn = time.strftime(args.out)
			if lastf is None or lastf.name != fn:
				if lastf is not None:
					lastf.close()
				lastf = open(fn, "a")
			lastf.write(json.dumps(msg)+"\n")
			lastf.flush()
		except Exception as e:
			print(e)
			sys.stdout.flush()

	mqtt_client = ttn.MQTTClient(args.app_id, args.access_key, mqtt_address=args.mqtt_server)
	mqtt_client.set_uplink_callback(callback)
	mqtt_client.connect()

	while True:
		time.sleep(1)

if __name__ == "__main__":
	main()
