import argparse
import os
import numpy as np
import json
import cv2
from skvideo import io
from stray_visualize import DEPTH_WIDTH, DEPTH_HEIGHT, _resize_camera_matrix

def read_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', type=str)
    parser.add_argument('--out', type=str)
    return parser.parse_args()

def write_frames(flags, rgb_out_dir):
    rgb_video = os.path.join(flags.dataset, 'rgb.mp4')
    video = io.vreader(rgb_video)
    for i, frame in enumerate(video):
        print(f"Writing rgb frame {i:06}" + " " * 10, end='\r')
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        image = cv2.resize(frame, (DEPTH_WIDTH, DEPTH_HEIGHT))
        frame_path = os.path.join(rgb_out_dir, f"{i:06}.jpg")
        cv2.imwrite(frame_path, image)

def write_depth(flags, depth_out_dir):
    depth_dir_in = os.path.join(flags.dataset, 'depth')
    files = sorted(os.listdir(depth_dir_in))
    for filename in files:
        if '.npy' not in filename:
            continue
        print(f"Writing depth frame {filename}", end='\r')
        number, _ = filename.split('.')
        depth = np.load(os.path.join(depth_dir_in, filename))
        cv2.imwrite(os.path.join(depth_out_dir, number + '.png'), depth)

def write_intrinsics(flags):
    intrinsics = np.loadtxt(os.path.join(flags.dataset, 'camera_matrix.csv'), delimiter=',')
    intrinsics_scaled = _resize_camera_matrix(intrinsics, DEPTH_WIDTH / 1920, DEPTH_HEIGHT / 1440)
    data = {}
    data['intrinsic_matrix'] = [intrinsics_scaled[0, 0], 0.0, 0.0,
            0.0, intrinsics_scaled[1, 1], 0.0,
            intrinsics_scaled[0, 2], intrinsics_scaled[1, 2], 1.0]
    data['width'] = DEPTH_WIDTH
    data['height'] = DEPTH_HEIGHT
    data['depth_scale'] = 1000.0
    data['fps'] = 60.0
    data['depth_format'] = 'Z16'
    with open(os.path.join(flags.out, 'camera_intrinsics.json'), 'wt') as f:
        f.write(json.dumps(data))

def write_config(flags):
    dataset_path = os.path.abspath(os.path.expanduser(flags.out))
    intrinsics_path = os.path.join(dataset_path, 'camera_intrinsics.json')
    config = {
        "name": "Stray Scanner dataset",
        "path_dataset": dataset_path,
        "path_intrinsic": intrinsics_path,
        "depth_scale": 1000.0,
        "max_depth": 15.0,
        "python_multi_threading": False
    }
    with open(os.path.join(dataset_path, 'config.json'), 'w') as f:
        f.write(json.dumps(config))

def main():
    flags = read_args()
    rgb_out = os.path.join(flags.out, 'color/')
    depth_out = os.path.join(flags.out, 'depth/')
    os.makedirs(rgb_out, exist_ok=True)
    os.makedirs(depth_out, exist_ok=True)

    write_config(flags)
    write_intrinsics(flags)
    write_depth(flags, depth_out)
    write_frames(flags, rgb_out)
    print("\nDone.")



if __name__ == "__main__":
    main()

