# kinect_capture
A simple tool for creating point cloud data using a Kinect 2. Requires [libfreenect2](https://github.com/OpenKinect/libfreenect2).

# Usage
capt *filename* *number_of_captures*

Creates *number_of_captures* point cloud files using *filename* as base name. Assumes the target is being rotated 360 degrees and numbers filenames accordingly.
