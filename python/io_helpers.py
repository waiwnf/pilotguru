import glob
import json
import os
import random

import numpy as np
import torch.utils.data

DATA_SUFFIX = 'data.npz'

# Constants for naming saved model checkpoints.
MODEL = 'model'
LAST = 'last'
BEST = 'best'

def LoadForwardAxis(forward_axis_json_filename):
  with open(forward_axis_json_filename) as forward_axis_file:
      forward_axis_json = json.load(forward_axis_file)
      forward_axis_dict = forward_axis_json['forward_axis']
      return np.array([
          forward_axis_dict['x'],
          forward_axis_dict['y'],
          forward_axis_dict['z']],
        dtype=np.float32)

def ModelFileName(out_dir, model_int_id, model_tag):
  return os.path.join(
      out_dir, '%s-%d-%s.pth' % (MODEL, model_int_id, model_tag))

def PreloadModelNames(models_dir, num_models):
  if models_dir is None:
    return None
  else:
    return [ModelFileName(models_dir, i, LAST) for i in range(num_models)]

def SortedDataFiles(data_dirs, data_suffix):
  data_files = []
  for data_dir in data_dirs:
    data_files.extend(
        glob.glob(os.path.join(data_dir, '*' + data_suffix)))
  data_files.sort()
  return data_files

def LoadDatasetNumpyFiles(
    data_dirs,
    element_names,
    data_suffix=DATA_SUFFIX):
  data_files = SortedDataFiles(data_dirs, data_suffix)
  single_data_file = np.load(data_files[0])

  data = [
    np.zeros(
      (len(data_files),) + single_data_file[x].shape,
      dtype=single_data_file[x].dtype)
    for x in element_names]
  for elem_idx, data_file in enumerate(data_files):
    data_file_loaded = np.load(data_file)
    for i, element_name in enumerate(element_names):
      data[i][elem_idx, ...] = data_file_loaded[element_name]

  return data

class NumpyFileDataset(torch.utils.data.Dataset):
  def __init__(self, data_dirs, element_names, data_suffix=DATA_SUFFIX):
    super(NumpyFileDataset, self).__init__()
    self.data_files = SortedDataFiles(data_dirs, data_suffix)
    self.element_names = element_names
  
  def __len__(self):
    return len(self.data_files)
  
  def __getitem__(self, idx):
    data_file_loaded = np.load(self.data_files[idx])
    return tuple(data_file_loaded[x] for x in self.element_names)

class InMemoryNumpyDataset(torch.utils.data.Dataset):
  def __init__(self, data):
    for item in data:
      assert data[0].shape[0] == item.shape[0]
    self.data = data
  
  def __len__(self):
    return self.data[0].shape[0]
  
  def __getitem__(self, idx):
    return tuple(np.copy(element[idx, ...]) for element in self.data)

class ImageFrameDataset(torch.utils.data.Dataset):
  """Dataset for data files with images (byte per channel).

  Optionally takes a centered crop to target width.

  Makes sure the raw data is in uint8 format (i.e byte per channel).
  Converts to float32 to feed to tensors and normalizes to [0, 1].
  """

  def __init__(
      self,
      source_dataset,
      image_element_idx,
      transforms=[],
      target_crop_width=None):
    super(ImageFrameDataset, self).__init__()
    self.source_dataset = source_dataset
    self.image_element_idx = image_element_idx
    self.transforms = transforms
    self.target_crop_width = target_crop_width
  
  def __len__(self):
    return self.source_dataset.__len__()

  def __getitem__(self, idx):
    item_base = self.source_dataset.__getitem__(idx)
    img_raw = item_base[self.image_element_idx]
    # Make sure input images are encoded as byte-per-channel.
    assert img_raw.dtype == np.uint8
    # Convert to float to feed into tensors, and normalize to [0, 1].
    item = item_base[:self.image_element_idx] + (
        img_raw.astype(np.float32) / 255.0,) + (
        item_base[(self.image_element_idx + 1):])
    
    for t in self.transforms:
      item = t(item)

    # Optionally crop to target_crop_width.
    img = item[self.image_element_idx]
    if self.target_crop_width is not None:
      assert img.shape[-1] >= self.target_crop_width
      left_boundary = int((img.shape[-1] - self.target_crop_width) / 2)
      right_boundary = left_boundary + self.target_crop_width
      img = img[...,left_boundary:right_boundary]

    return item[:self.image_element_idx] + (img,) + (
        item[(self.image_element_idx+1):])
