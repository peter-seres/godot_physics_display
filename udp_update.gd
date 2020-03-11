extends Spatial

# This script receives NED attitude and quaternion rotation updates through a 
# UDP port and updates the transform of the Spatial node accordingly.

var port = 12345
var socketUDP = PacketPeerUDP.new()
var attitude
var position

func _ready():
	# Set up the UDP port to listen to:
	if (socketUDP.listen(port, "127.0.0.1", 4096) != OK):
		printt("Error listening on port: " + str(port))
	else:
		printt("Listening on port: " + str(port))

func _process(_delta):
	
	# Check for incoming packets:
	if socketUDP.get_available_packet_count() > 0:
		var json = JSON.parse(socketUDP.get_packet().get_string_from_utf8())
		
		if json.error == OK:
			position = json.get_result().position
			attitude = json.get_result().attitude
		
			# Set the position of the transform:			
			translation = Vector3(position[0], position[1], position[2])
			
			# Set the rotation of the transform:
			var q = Quat(attitude[0], attitude[1], attitude[2], attitude[3])
			transform.basis = Basis(q)
			
			printt(attitude)
			printt(position)
			
		else:
			printt('JSON parse error.')
			
