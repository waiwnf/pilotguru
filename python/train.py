import argparse

import augmentation
import io_helpers
import models
import optimize

import numpy as np

import torch.optim
import torch.utils.data

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--data_dirs', required=True)
  parser.add_argument('--validation_data_dirs', required=True)
  parser.add_argument('--labels_file_suffix', default='inverse-radius')
  parser.add_argument('--batch_size', type=int, required=True)
  parser.add_argument('--epochs', type=int, required=True)
  parser.add_argument('--target_height', type=int, required=True)
  parser.add_argument('--target_width', type=int, required=True)
  parser.add_argument('--out_prefix', required=True)
  parser.add_argument('--dropout_prob', type=float, default=0.0)
  parser.add_argument('--max_horizontal_shift_pixels', type=int, default=0)
  parser.add_argument('--horizontal_label_shift_rate', type=float, default=0.0)
  parser.add_argument('--train_blur_sigma', type=float, default=2.0)
  parser.add_argument('--train_blur_prob', type=float, default=0.0)
  parser.add_argument('--do_pca_random_shifts', type=bool, default=False)
  parser.add_argument('--grayscale_interpolate_prob', type=float, default=0.0)
  parser.add_argument(
      '--example_label_extra_weight_scale', type=float, default=0.0)
  args = parser.parse_args()

  plain_train_in, plain_train_labels = io_helpers.LoadDatasetNumpyFiles(
      args.data_dirs.split(','), label_suffix=args.labels_file_suffix)
  plain_train_data = io_helpers.InMemoryNumpyDataset(
      plain_train_in, plain_train_labels)
  
  augment_settings = augmentation.AugmentSettings(
      target_width=args.target_width,
      max_horizontal_shift_pixels=args.max_horizontal_shift_pixels,
      horizontal_label_shift_rate=args.horizontal_label_shift_rate,
      blur_sigma=args.train_blur_sigma,
      blur_prob=args.train_blur_prob,
      grayscale_interpolate_prob=args.grayscale_interpolate_prob,
      do_pca_random_shifts=args.do_pca_random_shifts)
  joint_augmenters, data_augmenters = augmentation.MakeAugmenters(
      augment_settings, plain_train_in)

  trainset = io_helpers.ImageFrameDataset(
    io_helpers.L1LabelWeightingDataset(
        plain_train_data, args.example_label_extra_weight_scale),
    joint_transforms=joint_augmenters,
    data_transforms=data_augmenters)
  trainloader = torch.utils.data.DataLoader(
      trainset, batch_size=args.batch_size, shuffle=True)
  
  plain_val_in, plain_val_labels = io_helpers.LoadDatasetNumpyFiles(
      args.validation_data_dirs.split(','),
      label_suffix=args.labels_file_suffix)
  weighted_val_data = io_helpers.L1LabelWeightingDataset(
        io_helpers.InMemoryNumpyDataset(plain_val_in, plain_val_labels),
        args.example_label_extra_weight_scale)
  valset = io_helpers.ImageFrameDataset(
      weighted_val_data, target_crop_width=args.target_width)
  validation_loader = torch.utils.data.DataLoader(
      valset, batch_size=args.batch_size, shuffle=False)

  net = models.NvidiaSingleFrameNet(
      [3, args.target_height, args.target_width], args.dropout_prob)
  net.cuda()
  
  loss = optimize.WeightedMSELoss()
  optimizer = torch.optim.Adam(net.parameters())

  optimize.TrainModel(
      net,
      trainloader,
      validation_loader,
      loss,
      optimizer,
      args.epochs,
      args.out_prefix,
      optimize.FlattenInnerChunk,
      optimize.FlattenInnerChunk)
