extends Spatial

var uav;

func _ready():
	uav = get_node("UAV")

func _physics_process(delta):
	translation = uav.translation
	
