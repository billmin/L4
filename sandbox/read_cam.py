import cv2
import time

cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

while True:
	ret, frame = cap.read()
	cv2.imshow("test", frame)
	if cv2.waitKey(33) & 0xFF == ord('q'):
		break
		
cap.release()
cv2.destroyAllWindows()
