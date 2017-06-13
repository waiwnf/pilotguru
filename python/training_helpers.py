import augmentation
import io_helpers

import torch.utils.data

def MakeDataLoader(
    data,
    labels,
    target_width,
    example_label_extra_weight_scale,
    joint_augmenters,
    data_augmenters,
    batch_size,
    shuffle):
  plain_dataset = io_helpers.InMemoryNumpyDataset(data, labels)
  weighted_dataset = io_helpers.L1LabelWeightingDataset(
      plain_dataset, example_label_extra_weight_scale)
  image_dataset = io_helpers.ImageFrameDataset(
      weighted_dataset, joint_augmenters, data_augmenters, target_width)
  return torch.utils.data.DataLoader(
      image_dataset, batch_size=batch_size, shuffle=shuffle)
  

def MakeDataLoaders(
    train_data,
    train_labels,
    val_data,
    val_labels,
    target_width,
    augment_settings,
    batch_size,
    example_label_extra_weight_scale=0.0):
  joint_augmenters, data_augmenters = augmentation.MakeAugmenters(
      augment_settings, train_data)
  
  train_loader = MakeDataLoader(
      train_data,
      train_labels,
      target_width,
      example_label_extra_weight_scale,
      joint_augmenters,
      data_augmenters,
      batch_size,
      True)
  
  val_loader = MakeDataLoader(
      val_data,
      val_labels,
      target_width,
      example_label_extra_weight_scale,
      [],
      [],
      batch_size,
      False)
  
  return train_loader, val_loader
  