PID_Component(
	SHARED_LIB
	NAME OpenHSML-shared
	DIRECTORY OpenHSML
	CXX_STANDARD 11
	DEPEND
		opencv/opencv-highgui
		opencv/opencv-calib3d
		eigen
		libyaml
)
