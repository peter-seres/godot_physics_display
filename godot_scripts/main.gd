extends Spatial

# Called every frame. 'delta' is the elapsed time since the previous frame.
#func _process(delta):
#	pass

var mov_camera;
var static_camera;

func _ready():
	mov_camera = get_node("UAV/MovingCamera")
	static_camera = get_node("StaticCamera")

func _process(_delta):
	
	if Input.is_action_just_pressed("ui_accept"):
		if mov_camera.is_current():
			static_camera.make_current()
		else:
			mov_camera.make_current()
