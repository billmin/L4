import sys

ros_cv2_path = '/opt/ros/indigo/lib/python2.7/dist-packages'

if ros_cv2_path in sys.path:
	sys.path.remove(ros_cv2_path)
