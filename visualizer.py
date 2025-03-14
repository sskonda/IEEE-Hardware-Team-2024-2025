#!/usr/bin/python3

# Mostly copied from https://picamera.readthedocs.io/en/release-1.13/recipes2.html
# Run this script, then point a web browser at http:<this-ip-address>:8000
# Note: needs simplejpeg to be installed (pip3 install simplejpeg).

import io
import cv2
import logging
import socketserver
import numpy as np
from http import server
from threading import Condition, Thread

PAGE = """\
<html>
<head>
<title>picamera2 MJPEG streaming demo</title>
</head>
<body>
<h1>Picamera2 MJPEG Streaming Demo</h1>
<img src="stream.mjpg" width="640" height="480" />
</body>
</html>
"""

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        buf = cv2.imencode(".jpeg", buf)[1].tobytes()
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

OUTPUT = StreamingOutput()

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with OUTPUT.condition:
                        OUTPUT.condition.wait()
                        frame = OUTPUT.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

def run_server():
    address = ('', 8000)
    server = StreamingServer(address, StreamingHandler)
    server.serve_forever()

# Draw indicator function
def draw_indicator(image, position, heading, length=50, color=(0, 0, 255), thickness=2):
    img_height, img_width = image.shape[:2]
    
    # Normalize position from (0, 96) to (0, 48) onto image dimensions
    x = int(position[0] / 96 * img_width)
    y = int((48 - position[1]) / 48 * img_height)
    
    # Convert heading from degrees to radians
    angle_rad = np.deg2rad(heading)
    
    # Calculate end point for the arrow
    end_x = int(x + length * np.cos(angle_rad))
    end_y = int(y - length * np.sin(angle_rad))
    
    # Draw the arrow line
    cv2.arrowedLine(image, (x, y), (end_x, end_y), color, thickness)
    
    return image

SERVER = Thread(target=run_server)

if __name__ == "__main__":
    SERVER.start()
    
    # Create a blank white image
    bg = np.ones((500, 500, 3), dtype=np.uint8) * 255
    
    i = 0
    while True:
        img = draw_indicator(np.copy(bg), (0, 0), i)
        i += 1
        i %= 360
        OUTPUT.write(img)