import random

import re		
import subprocess
import time
		
import av
import numpy as np
import scipy.misc
import scipy.ndimage
import sklearn.decomposition

def CropHWC(img, top, bottom, left, right):
  """Crops given number of pixels from the edges of the image in HWC format."""

  assert left >= 0
  assert right >= 0
  assert top >= 0
  assert bottom >= 0
  assert (top + bottom) < img.shape[0]
  assert (left + right) < img.shape[1]
  return img[top:(img.shape[0] - bottom), left:(img.shape[1] - right), ...]

def RgbToYuv(rgb_image):
  assert rgb_image.dtype == np.uint8
  assert rgb_image.shape[2] == 3

  # Constants from https://en.wikipedia.org/wiki/YUV#Conversion_to.2Ffrom_RGB
  # (with input RGB values in [0, 1]).
  u_max = 0.437
  v_max = 0.615
  # Shift to have minimum values at 0.
  yuv_bias = np.array([0, u_max, v_max], dtype=np.float64).reshape([1,1,3])
  # Scale to have max YUV values at 1 (for RGB inputs in [0, 1]).
  yuv_scale = np.array(
      [1.0, 1.0 / (2.0 * u_max), 1.0 / (2.0 * v_max)],
      dtype=np.float64).reshape([1,1,3])

  # Coefficients from scikit-image.
  yuv_from_rgb = np.array(
      [[ 0.299     ,  0.587     ,  0.114      ],
       [-0.14714119, -0.28886916,  0.43601035 ],
       [ 0.61497538, -0.51496512, -0.10001026 ]],
      dtype=np.float64)
  rgb_image_float = rgb_image.astype(np.float64) / 255.0
  # Scale all the channels to 0...255 range to be able to store values as uint8.
  yuv_image_float = (
      (np.dot(rgb_image_float, yuv_from_rgb.T) + yuv_bias) *
      yuv_scale * 255.0)
  return np.clip(yuv_image_float, 0, 255).astype(np.uint8)

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

def VideoCaptureFrameGenerator(video_capture):
  '''Wraps OpenCV VideoCapture.'''
  video_frame_id = 0
  while True:
    yield (video_capture.read(), video_frame_id)
    video_frame_id += 1

def VideoFrameGeneratorLimitedFpsDelay(base_generator, max_fps):
  '''Inserts time delays between frames from base generator to not exceed
      max_fps.'''
  frame_interval_sec = 1.0 / max_fps
  prev_frame_time = time.time()
  for frame_data in base_generator:
    time_since_last_frame = time.time() - prev_frame_time
    remaining_sleep = max(0.0, frame_interval_sec - time_since_last_frame)
    time.sleep(remaining_sleep)
    prev_frame_time = time.time()
    yield frame_data

def VideoFrameGeneratorLimitedFpsSkip(base_generator, max_fps):
  '''Skips frames from base generator to not exceed max_fps.'''
  frame_interval_sec = 1.0 / max_fps
  prev_frame_time = time.time()
  for frame_data in base_generator:
    time_since_last_frame = time.time() - prev_frame_time
    if time_since_last_frame >= frame_interval_sec:
      prev_frame_time = time.time()
      yield frame_data

def GetPcaRgbDirections(images_chw):
  # Train data is of size [examples x (frames per example) x C x H x W].
  # For PCA we need to reshape it to [pixels x C]
  channels_last = np.transpose(images_chw, (0, 1, 3, 4, 2))
  pca_input = np.reshape(channels_last, (-1, images_chw.shape[2]))
  pca = sklearn.decomposition.PCA()
  pca.fit(pca_input)
  return pca.explained_variance_[:,np.newaxis] * pca.components_

def GrayscaleInterpolateInPlace(item, image_element_idx, grayscale_share):
  images_chw = item[image_element_idx]
  assert len(images_chw.shape) == 4  # Must be (examples x C x H x W) array.
  assert images_chw.shape[1] == 3  # Must be RGB.
  
  rgb_to_gray_weights = np.array(
      [0.2989, 0.5870, 0.1140], dtype=np.float32).reshape([1,3,1,1])
  img_gray = np.sum(images_chw * rgb_to_gray_weights, axis=1, keepdims=True)
  images_chw *= (1.0 - grayscale_share)
  images_chw += grayscale_share * img_gray
  return item

def RandomGrayscaleInterpolateInPlace(image_element_idx):
  return lambda x : GrayscaleInterpolateInPlace(x, image_element_idx, 1.0)

def GrayscaleInterpolateInPlaceTransform(image_element_idx, grayscale_share):
  return lambda x : GrayscaleInterpolateInPlace(
      x, image_element_idx, grayscale_share)

def BlurInPlace(item, image_element_idx, sigma):
  images_chw = item[image_element_idx]
  assert len(images_chw.shape) == 4  # Must be (examples x C x H x W) array.
  for example_idx in range(images_chw.shape[0]):
    for channel_idx in range(images_chw.shape[1]):
      image_slice = images_chw[example_idx, channel_idx, ...]
      scipy.ndimage.gaussian_filter(image_slice, sigma, output=image_slice)
  return item

def BlurInPlaceTransform(image_element_idx, sigma):
  return lambda x : BlurInPlace(x, image_element_idx, sigma)

def RandomShiftInPlace(item, image_element_idx, scaled_directions):
  images_chw = item[image_element_idx]
  assert len(images_chw.shape) == 4  # Must be (examples x C x H x W) array.
  assert len(scaled_directions.shape) == 2  # Must be (directions x C) matrix.
  assert scaled_directions.shape[1] == images_chw.shape[1]

  shifts_magnitude = np.random.normal(size=(scaled_directions.shape[0], 1))
  shift_vector_1d = np.sum(scaled_directions * shifts_magnitude, axis=0)
  shift_vector = shift_vector_1d.reshape(1,images_chw.shape[1],1,1)
  images_chw += shift_vector
  return item

def RandomShiftInPlaceTransform(image_element_idx, scaled_directions):
  return lambda x : RandomShiftInPlace(x, image_element_idx, scaled_directions)

def MaybeApplyInPlaceTransformLogic(item, transform, apply_probability):
  if random.uniform(0.0, 1.0) < apply_probability:
    return transform(item)
  else:
    return item

def MaybeApplyInPlaceTransform(transform, apply_probability):
  return lambda x : MaybeApplyInPlaceTransformLogic(
      x, transform, apply_probability)
