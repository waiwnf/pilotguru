import glob
import os
import random

import numpy as np
import torch.utils.data

class NumpyFileDataset(torch.utils.data.Dataset):
  """Pytorch dataset to read input examples from numpy files.
  
  Each example consists of two files: data file and label file, each with a 
  numpy array.
  """

  def __init__(self, data_dirs, data_suffix, label_suffix):
    super(NumpyFileDataset, self).__init__()
    # Attach .npy extensions to the file name masks.
    full_data_suffix = data_suffix + '.npy'
    full_label_suffix = label_suffix + '.npy'
    # Find all the matching frame image files in the data directories.
    self.data_files = []
    for data_dir in data_dirs:
      self.data_files.extend(
          glob.glob(os.path.join(data_dir, '*' + full_data_suffix)))
    self.data_files.sort()
    # Label files have the same name as the corresponding frame image file,
    # except for the suffix.
    self.label_files = [
        x.replace(full_data_suffix, full_label_suffix)
        for x in self.data_files]
  
  def __len__(self):
    return len(self.data_files)
  
  def __getitem__(self, idx):
    return np.load(self.data_files[idx]), np.load(self.label_files[idx])

class ImageFrameDataset(NumpyFileDataset):
  """Dataset for data files with images (byte per channel).

  Optionally takes a centered crop to target width.

  Makes sure the raw data is in uint8 format (i.e byte per channel).
  Converts to float32 to feed to tensors and normalizes to [0, 1].
  """

  def __init__(
      self,
      data_dirs,
      target_crop_width=None,
      img_suffix='img',
      label_suffix='angular'):
    super(ImageFrameDataset, self).__init__(
        data_dirs, data_suffix=img_suffix, label_suffix=label_suffix)
    self.target_crop_width = target_crop_width
  
  def __getitem__(self, idx):
    img_raw, label = super(ImageFrameDataset, self).__getitem__(idx)
    # Make sure input images are encoded as byte-per-channel.
    assert img_raw.dtype == np.uint8

    # Optionally crop to target_crop_width.
    if self.target_crop_width is not None:
      assert img_raw.shape[2] >= self.target_crop_width
      left_boundary = int((img_raw.shape[2] - self.target_crop_width) / 2)
      img_raw = img_raw[:,:,left_boundary:(left_boundary + self.target_crop_width)]

    # Convert to float to feed into tensors, and normalize to [0, 1].
    img = img_raw.astype(np.float32) / 255.0
    return img, label

class SteeringShiftAugmenterDataset(torch.utils.data.Dataset):
  """Dataset wrapper for shifted-crop steering angle augmentation.
  
  Take crops randomly off-centered horizontally, and linearly andjusts the
  label according to the crop shift magnitude.
  """

  def __init__(
      self,
      source_dataset,
      target_width,
      max_horizontal_shift,
      horizontal_label_shift_rate):
    """
    Ars:
      source_dataset: torch.utils.data.Dataset descendant to wrap.
          We assume that source_dataset provides camera images and 
          corresponding steering angles as data and labels.
      target_width: output width to crop the input image to.
      max_horizontal_shift: maximum off-center crop shift in pixels. 
          The actual shift for each retrieved item is sampled uniformly from
          [-max_horizontal_shift,max_horizontal_shift interval].
      horizontal_label_shift_rate: linear coefficient for tweaking the label 
          according to the off-center crop shift. The resulting label is
          source_label + shift_rate * shift_pixels / max_horizontal_shift.
    """
    super(SteeringShiftAugmenterDataset, self).__init__()
    self.source_dataset = source_dataset
    self.target_width = target_width
    self.max_horizontal_shift = max_horizontal_shift
    self.horizontal_label_shift_rate = horizontal_label_shift_rate
  
  def __len__(self):
    return self.source_dataset.__len__()
  
  def __getitem__(self, idx):
    source_img, source_label = self.source_dataset.__getitem__(idx)
    # Margin to the edge of the source image for a centered crop.
    crop_margin = int((source_img.shape[2] - self.target_width) / 2)
    assert crop_margin >= 0

    # Randomly choose the fraction of max_horizontal_shift to shift by.
    horizontal_shift_fraction = random.uniform(-1.0, 1.0)
    horizontal_shift_pixels = int(
        horizontal_shift_fraction * self.max_horizontal_shift)
    left_boundary = crop_margin + horizontal_shift_pixels
    right_boundary = left_boundary + self.target_width
    img_cropped = source_img[:,:,left_boundary:right_boundary]

    # Adjust the label to account for the off-center crop shift.
    shifted_label = (source_label + 
        horizontal_shift_fraction * self.horizontal_label_shift_rate)

    return img_cropped, shifted_label