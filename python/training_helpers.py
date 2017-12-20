import augmentation
import io_helpers

import torch.utils.data

def MakeDataLoader(
    data,
    image_element_idx,
    weighting_label_idx,
    target_width,
    example_label_extra_weight_scale,
    augmenters,
    batch_size,
    shuffle):
  plain_dataset = io_helpers.InMemoryNumpyDataset(data)
  weighted_dataset = io_helpers.L1LabelWeightingDataset(
      plain_dataset, weighting_label_idx, example_label_extra_weight_scale)
  image_dataset = io_helpers.ImageFrameDataset(
      weighted_dataset,
      image_element_idx,
      augmenters,
      target_width)
  return torch.utils.data.DataLoader(
      image_dataset, batch_size=batch_size, shuffle=shuffle)
  

def MakeDataLoaders(
    train_data,
    val_data,
    image_element_idx,
    weighting_label_idx,
    target_width,
    augment_settings,
    batch_size,
    example_label_extra_weight_scale=0.0):
  augmenters = augmentation.MakeAugmenters(
      augment_settings, image_element_idx, weighting_label_idx, train_data)
  
  train_loader = MakeDataLoader(
      train_data,
      image_element_idx,
      weighting_label_idx,
      target_width,
      example_label_extra_weight_scale,
      augmenters,
      batch_size,
      True)
  
  val_loader = MakeDataLoader(
      val_data,
      image_element_idx,
      weighting_label_idx,
      target_width,
      example_label_extra_weight_scale,
      [],  # augmenters
      batch_size,
      False)
  
  return train_loader, val_loader
  