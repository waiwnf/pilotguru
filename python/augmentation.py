import random
from collections import namedtuple

import image_helpers

import numpy as np

def SteeringTrainingRandomShift(
    item,
    image_element_idx,
    steering_element_idx,
    target_width,
    max_horizontal_shift,
    horizontal_label_shift_rate):
  """Shifted-crop steering angle augmentation.
  
  Takes crops randomly off-centered horizontally, and linearly andjusts the
  label according to the crop shift magnitude.

  Args:
    item: a list with all of the data pieces of a given training sample.
    image_element_idx: index of the data piece within item representing the 
        image
    steering_element_idx: index of the data piece within item representing the
        steering magnitudes
    target_width: output width to crop the input image to.
    max_horizontal_shift: maximum off-center crop shift in pixels. 
        The actual shift for each retrieved item is sampled uniformly from
        [-max_horizontal_shift,max_horizontal_shift interval].
    horizontal_label_shift_rate: linear coefficient for tweaking the label 
        according to the off-center crop shift. The resulting label is
        source_label + shift_rate * shift_pixels / max_horizontal_shift.
  """
  
  images = item[image_element_idx]  # batch x C x H x W
  labels = item[steering_element_idx]

  # Margin to the edge of the source image for a centered crop.
  crop_margin = int((images.shape[3] - target_width) / 2)
  assert crop_margin >= 0

  # Randomly choose the fraction of max_horizontal_shift to shift by.
  horizontal_shift_fraction = random.uniform(-1.0, 1.0)
  horizontal_shift_pixels = round(
      horizontal_shift_fraction * max_horizontal_shift)
  left_boundary = crop_margin + horizontal_shift_pixels
  right_boundary = left_boundary + target_width
  images = images[:,:,:,left_boundary:right_boundary]

  # Adjust the labels to account for the off-center crop shift.
  labels += horizontal_shift_fraction * horizontal_label_shift_rate

  return item

def SteeringTrainingRandomShiftTransform(
    image_element_idx,
    steering_element_idx,
    target_width,
    max_horizontal_shift,
    horizontal_label_shift_rate):
  return lambda item : SteeringTrainingRandomShift(
      item,
      image_element_idx,
      steering_element_idx,
      target_width,
      max_horizontal_shift,
      horizontal_label_shift_rate)


AugmentSettings = namedtuple(
    'AugmentSettings',
    ['target_width', 'max_horizontal_shift_pixels',
      'horizontal_label_shift_rate', 'blur_sigma', 'blur_prob',
      'grayscale_interpolate_prob', 'random_shift_directions'])
AugmentSettings.__new__.__defaults__ = (
    -1,     # target_width
    0,      # max_horizontal_shift_pixels
    np.array([0.0], dtype=np.float32),    # horizontal_label_shift_rate
    2.0,    # blur_sigma
    0.0,    # blur_prob
    0.0,    # grayscale_interpolate_prob
    None    # random_shift_directions
  )

def MakeAugmenters(
    augment_settings, image_element_idx, steering_element_idx, in_data = None):
  augmenters = []
  if augment_settings.max_horizontal_shift_pixels > 0:
    assert augment_settings.target_width > 0
    augmenters.append(
        SteeringTrainingRandomShiftTransform(
            image_element_idx,
            steering_element_idx,
            augment_settings.target_width,
            augment_settings.max_horizontal_shift_pixels,
            augment_settings.horizontal_label_shift_rate))

  if augment_settings.random_shift_directions is not None:
    augmenters.append(
        image_helpers.RandomShiftInPlaceTransform(
            image_element_idx, augment_settings.random_shift_directions))

  if augment_settings.blur_prob > 0:
    assert augment_settings.blur_sigma > 0
    augmenters.append(
        image_helpers.MaybeApplyInPlaceTransform(
            image_helpers.BlurInPlaceTransform(
                image_element_idx, augment_settings.blur_sigma),
            augment_settings.blur_prob))

  if augment_settings.grayscale_interpolate_prob > 0:
    augmenters.append(
        image_helpers.MaybeApplyInPlaceTransform(
            image_helpers.RandomGrayscaleInterpolateInPlace(image_element_idx),
            augment_settings.grayscale_interpolate_prob))
  
  return augmenters
