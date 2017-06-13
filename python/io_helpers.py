import glob
import os
import random

import numpy as np
import torch.utils.data

ANGULAR = 'angular'
IMG = 'img'
NUMPY_EXTN = '.npy'

def SortedExampleFiles(data_dirs, full_data_suffix, full_label_suffix):
  """Returns two aligned lists of files for training examples: data and labels.

  Globs the files with given suffixes in data_dirs and sorts alphabetically.  
  """
  data_files = []
  for data_dir in data_dirs:
    data_files.extend(
        glob.glob(os.path.join(data_dir, '*' + full_data_suffix)))
  data_files.sort()
  # Label files have the same name as the corresponding frame image file,
  # except for the suffix.
  label_files = [
      x.replace(full_data_suffix, full_label_suffix) for x in data_files]
  return data_files, label_files

def LoadDatasetNumpyFiles(
    data_dirs,
    data_suffix=IMG,
    label_suffix=ANGULAR,
    file_extension=NUMPY_EXTN):
  """Reads data and labels from name-aligned files and stacks into two arrays.
  """

  # Attach file extensions to the file name masks.
  full_data_suffix = data_suffix + file_extension
  full_label_suffix = label_suffix + file_extension
  # Find all the matching frame image files in the data directories.
  data_files, label_files = SortedExampleFiles(
      data_dirs, full_data_suffix, full_label_suffix)

  single_image = np.load(data_files[0])
  data = np.zeros((len(data_files),) + single_image.shape, dtype=np.uint8)
  for elem_idx, data_file in enumerate(data_files):
    data[elem_idx, ...] = np.load(data_file)
  labels = np.array([np.load(x) for x in label_files], dtype=np.float32)

  return data, labels

class NumpyFileDataset(torch.utils.data.Dataset):
  """Pytorch dataset to read input examples from numpy files.
  
  Each example consists of two files: data file and label file, each with a 
  numpy array.
  """

  def __init__(self, data_dirs, data_suffix=IMG, label_suffix=ANGULAR):
    super(NumpyFileDataset, self).__init__()
    # Attach .npy extensions to the file name masks.
    full_data_suffix = data_suffix + NUMPY_EXTN
    full_label_suffix = label_suffix + NUMPY_EXTN
    self.data_files, self.label_files = SortedExampleFiles(
        data_dirs, full_data_suffix, full_label_suffix)
  
  def __len__(self):
    return len(self.data_files)
  
  def __getitem__(self, idx):
    return np.load(self.data_files[idx]), np.load(self.label_files[idx]), np.array([1.0], dtype=np.float32)

class InMemoryNumpyDataset(torch.utils.data.Dataset):
  def __init__(self, data, labels):
    assert data.shape[0] == labels.shape[0]
    self.data = data
    self.labels = labels
  
  def __len__(self):
    return self.data.shape[0]
  
  def __getitem__(self, idx):
    return self.data[idx, ...], self.labels[idx], np.array([1.0], dtype=np.float32)

class L1LabelWeightingDataset(torch.utils.data.Dataset):
  """Upweights the examples from the source dataset by (1 + |label|).

  This is useful for steering angle prediction task to give higher weights to
  the naturally more rare examples recorded in turns (vs. driving straight).
  """

  def __init__(self, source_dataset, label_scale = 0.0):
    super(L1LabelWeightingDataset, self).__init__()
    self.source_dataset = source_dataset
    assert label_scale >= 0
    self.label_scale = label_scale
  
  def __len__(self):
    return self.source_dataset.__len__()
  
  def __getitem__(self, idx):
    data, label, source_weight = self.source_dataset.__getitem__(idx)
    assert len(label) == 1
    effective_weight_scale = self.label_scale * np.abs(label) + 1.0
    return data, label, source_weight * effective_weight_scale

class ImageFrameDataset(torch.utils.data.Dataset):
  """Dataset for data files with images (byte per channel).

  Optionally takes a centered crop to target width.

  Makes sure the raw data is in uint8 format (i.e byte per channel).
  Converts to float32 to feed to tensors and normalizes to [0, 1].
  """

  def __init__(
      self,
      source_dataset,
      joint_transforms=[],
      data_transforms=[],
      target_crop_width=None):
    super(ImageFrameDataset, self).__init__()
    self.source_dataset = source_dataset
    self.joint_transforms = joint_transforms
    self.data_transforms = data_transforms
    self.target_crop_width = target_crop_width
  
  def __len__(self):
    return self.source_dataset.__len__()

  def __getitem__(self, idx):
    img_raw, label, example_weight = self.source_dataset.__getitem__(idx)
    # Make sure input images are encoded as byte-per-channel.
    assert img_raw.dtype == np.uint8

    # Optionally crop to target_crop_width.
    if self.target_crop_width is not None:
      assert img_raw.shape[3] >= self.target_crop_width
      left_boundary = int((img_raw.shape[3] - self.target_crop_width) / 2)
      right_boundary = left_boundary + self.target_crop_width
      img_raw = img_raw[:,:,:,left_boundary:right_boundary]

    # Convert to float to feed into tensors, and normalize to [0, 1].
    img = img_raw.astype(np.float32) / 255.0

    for t in self.joint_transforms:
      img, label = t(img, label)

    for t in self.data_transforms:
      img = t(img)

    return img, label, example_weight
