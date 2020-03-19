extends Spatial

# This script receives NED attitude and quaternion rotation updates through a 
# UDP port and updates the transform of the Spatial node accordingly.

var port = 12345
var socketUDP = PacketPeerUDP.new()
var attitude
var position

var obj_cam
var cam_dst = 4

func _ready():
	# Set up the UDP port to listen to:
	if (socketUDP.listen(port, "127.0.0.1", 4096) != OK):
		printt("Error listening on port: " + str(port))
	else:
		printt("Listening on port: " + str(port))
	obj_cam = get_parent().get_node("ObjCamera")


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
	
	# Update ObjCamera position
	obj_cam.translation = translation + Vector3(-1, 0, 0) * cam_dst
	
	# Zoom camera with input
	if Input.is_key_pressed(KEY_UP):
		cam_dst += 0.1
	if Input.is_key_pressed(KEY_DOWN):
		cam_dst -= 0.1
