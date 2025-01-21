#!/bin/python3

from robot.components import HallEncoder
from collections import deque
import time

def main():
    encoder = HallEncoder(22, 23, 1)
    encoder.init()
    q = deque(maxlen=25)

    try:
        while True:
            q.append(encoder.get_speed())
            print("Speed:", sum(q) / len(q) / 60)
            time.sleep(0.02)

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
    