PID_Component(
	APPLICATION
	NAME OpenHSML_apps
	DIRECTORY OpenHSML_apps
	CXX_STANDARD 11
	DEPEND
		OpenHSML/OpenHSML-shared
		opencv/opencv-highgui
		opencv/opencv-calib3d
		eigen
		libyaml
)
