import glob
import os

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

  Makes sure the raw data is in uint8 format (i.e byte per channel).
  Converts to float32 to feed to tensors and normalizes to [0, 1].
  """

  def __init__(self, data_dirs, img_suffix='img', label_suffix='angular'):
    super(ImageFrameDataset, self).__init__(
        data_dirs, data_suffix=img_suffix, label_suffix=label_suffix)
  
  def __getitem__(self, idx):
    img_raw, label = super(ImageFrameDataset, self).__getitem__(idx)
    # Make sure input images are encoded as byte-per-channel.
    assert img_raw.dtype == np.uint8
    # Convert to float to feed into tensors, and normalize to [0, 1].
    img = img_raw.astype(np.float32) / 255.0
    # Load the corresponding label.
    return img, label
