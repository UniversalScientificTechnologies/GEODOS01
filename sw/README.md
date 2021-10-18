# Supporting software

`lorasave.py` is a script for general logging of LoRa uplink messages to a file. It works with the The Things Network infrastructure.

It creates log files with newline-separated JSON messages. To process the logged messages and extract specific fields, you may for example use the `jq` tool, like this:

	$ cat __path_to_logs__ | jq -r '[.received_at, .end_device_ids.device_id, .device_id, .uplink_message.decoded_payload.V] | @tsv'
