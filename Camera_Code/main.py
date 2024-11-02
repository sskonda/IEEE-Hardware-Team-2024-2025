from pi_camera.read_april_tag import read_april_tag
from pi_camera.read_camera import capture_image_from_picam2

def main(): 

    imgDir = "pi_camera/read_camera/images/testIMG.png"

    capture_image_from_picam2.read_Cam(imgDir)

    tags = read_april_tag.readTag(imgDir)

    if tags:
        for tag in tags:
            print("Tag ID: ", tag.tag_id)
            print("Tag Corners: ", tag.corners)

    else:
        print("No AprilTag detected.")



if __name__ == "main":
    main()



