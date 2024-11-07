import cv2

def read_Cam(file):

    cam = cv2.VideoCapture(0) # could be 0 or -1, depends where it is plugged in

    if not cam.isOpened():
        print("Error: could not open camera.")
        return
    
    ret, frame = cam.read()

    if ret:

        cv2.imwrite(file, frame)
        print("Image Captured and Saved")
    else:
        print('Image capture failed')

    cam.release()