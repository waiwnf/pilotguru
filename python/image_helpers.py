import re
import subprocess

import av

import numpy as np
import scipy.misc

def CropHWC(img, top, bottom, left, right):
  """Crops given number of pixels from the edges of the image in HWC format."""

  assert left >= 0
  assert right >= 0
  assert top >= 0
  assert bottom >= 0
  assert (top + bottom) < img.shape[0]
  assert (left + right) < img.shape[1]
  return img[top:(img.shape[0] - bottom), left:(img.shape[1] - right), ...]

def MaybeResizeHWC(img, height, width):
  if height <= 0 and width <= 0:
    return img
  else:
    effective_height = height if height > 0 else img.shape[0]
    effective_width = width if width > 0 else img.shape[1]
    return scipy.misc.imresize(img, (effective_height, effective_width))

def RawVideoFrameGenerator(filename):
  container = av.open(filename)
  for packet in container.demux():
    for frame in packet.decode():
      if isinstance(frame, av.video.frame.VideoFrame):
        yield frame

def VideoOrientationDegrees(filename):
  '''Infers video rotation by parsing  avprobe output.'''
  avprobe_outcome = subprocess.run(
      ['avprobe', filename], stderr=subprocess.PIPE)
  for line in avprobe_outcome.stderr.decode('utf-8').split('\n'):
    match_result = re.match('^\s*rotate\s*:\s*([0-9]+)\s*$', line)
    if match_result is not None:
      return int(match_result.group(1))
  return 0  # No rotation information is in the file, hence zero rotation.

def Video90RotationTimes(rotation_degrees):
  '''Converts rotation in degrees to number of 90-degree rotations.'''
  assert rotation_degrees % 90 == 0
  return (rotation_degrees % 360) / 90

def VideoFrameGenerator(filename):
  rotation_times = Video90RotationTimes(VideoOrientationDegrees(filename))
  raw_frames_generator = RawVideoFrameGenerator(filename)
  for raw_frame in raw_frames_generator:
    frame_image_raw = np.asarray(raw_frame.to_image())
    yield (np.rot90(frame_image_raw, k=rotation_times), raw_frame.index)
