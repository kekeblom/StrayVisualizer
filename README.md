# StrayVisualizer

A set of scripts to work with data collected using [Stray Scanner](https://apps.apple.com/us/app/stray-scanner/id1557051662).

![Staircase pointcloud](images/pointcloud.webp)

## Usage

To access the data from your device, see [this wiki entry](https://github.com/kekeblom/StrayVisualizer/wiki/Accessing-Data).

### Installing Dependencies

Install dependencies with `pip -r requirements.txt`.

### Example Datasets

If you don't have your own dataset, download one of these example datasets:
- `wget https://stray-data.nyc3.digitaloceanspaces.com/datasets/ZB1.tar.gz`
- `wget https://stray-data.nyc3.digitaloceanspaces.com/datasets/ZB2.tar.gz`
- `wget https://stray-data.nyc3.digitaloceanspaces.com/datasets/ZB3.tar.gz`

Assuming you selected `ZB1.tar.gz`, you can extract the dataset using the command `tar -xvf ZB1.tar.gz`.

## Visualizing the data

Run `python stray_visualize.py <path-to-dataset>`.

Available command line options are:
- `--point-clouds` shows pointclouds.
- `--confidence=<value>` there are three levels of confidence for the depth outputs: 0, 1 and 2. Higher is more confident. Only points having confidence higher or equal to the given value are shown.
- `--frames` shows the camera pose coordinate frames.
- `--every=<n>` determines how often the coordinate frame is drawn. Default is to draw every 60th frame.
- `--trajectory` shows a black line for the trajectory of the camera.
- `--integrate` will run the data through the Open3D RGB-D integration pipeline and visualize the resulting mesh.
- `--voxel-size=<size>` sets the voxel size in meters for RGB-D integration.
- `--mesh-filename` save the mesh from RGB-D integration into the given file. Defaults to no mesh saved.

## Creating a Video From the Depth Maps

`python make_video.py <dataset-path>` will combine depth maps into a video. It will create a file `depth_video.mp4` inside the dataset folder.

## Running 3D Reconstruction on Collected Data

For convenience, the `convert_to_open3d.py` script is provided to convert the Stray Scanner format to the Open3D [reconstruction system format](http://www.open3d.org/docs/release/tutorial/reconstruction_system/capture_your_own_dataset.html).

Usage: `python convert_to_open3d.py --dataset <path-to-dataset> --out <where-to-save-converted-dataset>`.

You can run their reconstruction pipeline using `python <path-to-open3d-repo>/examples/python/reconstruction_system/run_system.py <config.json> --make --register --refine --integrate` as described [here](http://www.open3d.org/docs/release/tutorial/reconstruction_system/system_overview.html). `<config.json>` is a configuration file created for convenience by `convert_to_open3d.py` into the newly created dataset folder. It contains absolute paths, so if you move your dataset, be sure to update the configuration.

Beware that the Open3D reconstruction system takes up quite a lot of memory and compute. On Mac, you might need to add `"python_multi_threading": false` into the config file to avoid crashing.

## Reporting Issues

If you find any issues with this project or bugs in the Stray Scanner app, you can open an issue on this repository.




