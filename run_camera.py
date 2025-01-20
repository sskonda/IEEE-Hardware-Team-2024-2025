from robot.components import Camera

from apriltag import apriltag

def main():
    camera = Camera(0)
    camera.init()

    try:
        while True:
            print(camera.detect_apriltag())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
    