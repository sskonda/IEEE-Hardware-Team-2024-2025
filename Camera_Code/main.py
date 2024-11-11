import cv2
import os
from pi_camera.read_april_tag import read_april_tag
from pi_camera.read_camera import capture_image_from_picam2

# RUN IF USING THE PI CAMERA

# def main(): 

#     imgDir = "pi_camera/read_camera/images/testIMG.png"

#     capture_image_from_picam2.read_Cam(imgDir)

#     tags = read_april_tag.readTag(imgDir)

#     if tags:
#         for tag in tags:
#             print("Tag ID: ", tag.tag_id)
#             print("Tag Corners: ", tag.corners)

#     else:
#         print("No AprilTag detected.")

#     # Create Bounding Box

#     corners_as_ints =[[None] * len(tags[0].corners) for _ in range(len(tags))]

#     if tags:
#         for i in range(len(tags)):
#             for j in range(len(tags[0].corners)):
                
#                 corners_as_ints[i][j] = tuple(int(value) for value in tags[i].corners[j])
                    

#     image = cv2.imread(imgDir)
#     green = (0, 255, 0)
#     black = (0, 0, 0)
#     line_size = 5


#     for i in range(len(corners_as_ints)):
#         cv2.line(image, corners_as_ints[i][0], corners_as_ints[i][1], green, line_size)
#         cv2.line(image, corners_as_ints[i][1], corners_as_ints[i][2], green, line_size)
#         cv2.line(image, corners_as_ints[i][2], corners_as_ints[i][3], green, line_size)
#         cv2.line(image, corners_as_ints[i][3], corners_as_ints[i][0], green, line_size)


#     cv2.imwrite('new.png', image)


# RUN IF TESTING IMAGES

def main():
    
    imgDir = "Camera_Code/pi_camera/read_camera/images/example_01.png"

    tags = read_april_tag.readTag(imgDir)

    if tags:
        for tag in tags:
            print("Tag ID: ", tag.tag_id)
            print("Tag Corners: ", tag.corners)
    else:
        print("No AprilTag detected.")

    # Create Bounding Box

    corners_as_ints =[[None] * len(tags[0].corners) for _ in range(len(tags))]

    if tags:
        for i in range(len(tags)):
            for j in range(len(tags[0].corners)):
                
                corners_as_ints[i][j] = tuple(int(value) for value in tags[i].corners[j])
                    

    image = cv2.imread(imgDir)
    green = (0, 255, 0)
    black = (0, 0, 0)
    line_size = 5


    for i in range(len(corners_as_ints)):
        cv2.line(image, corners_as_ints[i][0], corners_as_ints[i][1], green, line_size)
        cv2.line(image, corners_as_ints[i][1], corners_as_ints[i][2], green, line_size)
        cv2.line(image, corners_as_ints[i][2], corners_as_ints[i][3], green, line_size)
        cv2.line(image, corners_as_ints[i][3], corners_as_ints[i][0], green, line_size)


    cv2.imwrite('new.png', image)


if __name__ == "__main__":
    main()



