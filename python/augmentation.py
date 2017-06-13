from collections import namedtuple

import image_helpers

import numpy as np

AugmentSettings = namedtuple(
    'AugmentSettings',
    'max_horizontal_shift_pixels horizontal_label_shift_rate ' + 
    'blur_sigma blur_prob '+ 
    'grayscale_interpolate_prob ' +
    'do_pca_random_shifts')
AugmentSettings.__new__.__defaults__ = (
    0,      # max_horizontal_shift_pixels
    0.0,    # horizontal_label_shift_rate
    2.0,    # blur_sigma
    0.0,    # blur_prob
    0.0,    # grayscale_interpolate_prob
    False   # do_pca_random_shifts
  )

def MakeAugmenters(augment_settings, in_data = None):
  data_augmenters = []
  if augment_settings.do_pca_random_shifts:
    pca_directions = image_helpers.GetPcaRgbDirections(
        plain_train_data.data.astype(np.float32) / 255.0)
    data_augmenters.append(
        image_helpers.RandomShiftInPlaceTransform(pca_directions))

  if augment_settings.blur_prob > 0:
    assert augment_settings.blur_sigma > 0
    data_augmenters.append(
        image_helpers.MaybeApplyInPlaceTransform(
            image_helpers.BlurInPlaceTransform(augment_settings.blur_sigma),
            augment_settings.blur_prob))

  if augment_settings.grayscale_interpolate_prob > 0:
    data_augmenters.append(
        image_helpers.MaybeApplyInPlaceTransform(
            image_helpers.RandomGrayscaleInterpolateInPlace,
            augment_settings.grayscale_interpolate_prob))
  
  return data_augmenters
