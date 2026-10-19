import cv2
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ok, frame = cap.read() if cap.isOpened() else (False, None)
print("opened:", cap.isOpened(), "| frame:", None if not ok else frame.shape)
if ok:
    cv2.imwrite("/ws/cam_test.jpg", frame)
    print("saved cam_test.jpg in the repo folder")
cap.release()
