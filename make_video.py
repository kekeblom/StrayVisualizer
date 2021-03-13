import argparse
import os
import numpy as np
from PIL import Image
import skvideo
from matplotlib import cm
from skvideo import io

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('data')
    return parser.parse_args()

def main():
    flags = parse_args()

    images = sorted(os.listdir(os.path.join(flags.data, 'depth')))
    frames = []
    for i, image in enumerate(images):
        print(f"Reading image {image}", end='\r')
        path = os.path.join(flags.data, 'depth', image)
        depth = np.load(path)
        max_depth = 7.5
        depth_m = depth / 1000.0
        depth_map = np.clip(1.0 - depth_m / max_depth, 0.0, 1.0)
        depth_map = cm.inferno(depth_map)

        frames.append((depth_map * 255).astype(np.uint8))

    writer = io.FFmpegWriter(os.path.join(flags.data, 'depth_video.mp4'))
    try:
        for i, frame in enumerate(frames):
            print(f"Writing frame {i:06}" + " " * 10, end='\r')
            writer.writeFrame(frame)
    finally:
        writer.close()

if __name__ == '__main__':
    main()

