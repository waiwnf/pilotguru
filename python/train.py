import argparse

import augmentation
import image_helpers
import io_helpers
import models
import optimize
import training_helpers

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
  parser.add_argument('--net_name', default=models.NVIDIA_NET_NAME)
  parser.add_argument('--label_dimensions', type=int, default=1)
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

  train_data, train_labels = io_helpers.LoadDatasetNumpyFiles(
      args.data_dirs.split(','), label_suffix=args.labels_file_suffix)
  val_data, val_labels = io_helpers.LoadDatasetNumpyFiles(
      args.validation_data_dirs.split(','),
      label_suffix=args.labels_file_suffix)
  random_shift_directions = None if not args.do_pca_random_shifts else (
      image_helpers.GetPcaRgbDirections(train_data.astype(np.float32) / 255.0))
  augment_settings = augmentation.AugmentSettings(
      target_width=args.target_width,
      max_horizontal_shift_pixels=args.max_horizontal_shift_pixels,
      horizontal_label_shift_rate=args.horizontal_label_shift_rate,
      blur_sigma=args.train_blur_sigma,
      blur_prob=args.train_blur_prob,
      grayscale_interpolate_prob=args.grayscale_interpolate_prob,
      random_shift_directions=random_shift_directions)
  
  train_loader, val_loader = training_helpers.MakeDataLoaders(
      train_data,
      train_labels,
      val_data,
      val_labels,
      args.target_width,
      augment_settings,
      args.batch_size,
      args.example_label_extra_weight_scale)

  net = models.MakeNetwork(
      args.net_name,
      in_shape=[3, args.target_height, args.target_width],
      out_dims=args.label_dimensions,
      dropout_prob=args.dropout_prob)
  net.cuda()
  
  train_settings = optimize.TrainSettings(
      optimize.LossSettings(optimize.WeightedMSELoss()),
      torch.optim.Adam(net.parameters()),
      args.epochs)

  optimize.TrainModel(
      net,
      train_loader,
      val_loader,
      train_settings,
      args.out_prefix)
