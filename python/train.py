import argparse

import augmentation
import image_helpers
import io_helpers
import json
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
  parser.add_argument('--data_file_suffix', default='data.npz')
  parser.add_argument('--batch_size', type=int, required=True)
  parser.add_argument('--batch_use_prob', type=float, default=1.0)
  parser.add_argument('--epochs', type=int, required=True)
  parser.add_argument('--learning_rate', type=float, default=1e-3)
  parser.add_argument('--in_channels', type=int, default=3)
  parser.add_argument('--target_height', type=int, required=True)
  parser.add_argument('--target_width', type=int, required=True)
  parser.add_argument('--net_name', default=models.NVIDIA_NET_NAME)
  parser.add_argument('--num_nets_to_train', type=int, default=1,
      help='How many identically structured models to train simultaneously.')
  parser.add_argument(
    '--net_options',
    default=json.dumps({
        models.CONV: {
            models.BATCHNORM: True,
            models.ACTIVATION: models.RELU,
            models.DROPOUT: models.DROPOUT_2D},
        models.FC : {
            models.BATCHNORM: True,
            models.ACTIVATION: models.RELU,
            models.DROPOUT: models.DROPOUT_2D}}))
  parser.add_argument('--label_dimensions', type=int, default=1)
  parser.add_argument('--out_prefix', required=True)
  parser.add_argument('--log_dir', default='')
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

  # TODO get this from the networks
  data_element_names = ['frame_img', 'steering']
  image_element_idx = data_element_names.index('frame_img')
  weight_label_idx = data_element_names.index('steering')

  train_data = io_helpers.LoadDatasetNumpyFiles(
      args.data_dirs.split(','),
      data_element_names,
      data_suffix=args.data_file_suffix)
  val_data = io_helpers.LoadDatasetNumpyFiles(
      args.validation_data_dirs.split(','),
      data_element_names,
      data_suffix=args.data_file_suffix)
  random_shift_directions = None if not args.do_pca_random_shifts else (
      image_helpers.GetPcaRgbDirections(
          train_data[image_element_idx].astype(np.float32) / 255.0))
  augment_settings = augmentation.AugmentSettings(
      target_width=args.target_width,
      max_horizontal_shift_pixels=args.max_horizontal_shift_pixels,
      horizontal_label_shift_rate=args.horizontal_label_shift_rate,
      blur_sigma=args.train_blur_sigma,
      blur_prob=args.train_blur_prob,
      grayscale_interpolate_prob=args.grayscale_interpolate_prob,
      random_shift_directions=random_shift_directions)
  print(args.net_options)
  net_options = json.loads(args.net_options)
  
  train_loader, val_loader = training_helpers.MakeDataLoaders(
      train_data,
      val_data,
      image_element_idx,
      weight_label_idx,
      args.target_width,
      augment_settings,
      args.batch_size,
      args.example_label_extra_weight_scale)

  nets = []
  train_settings = []
  for _ in range(args.num_nets_to_train):
    net = models.MakeNetwork(
        args.net_name,
        in_shape=[args.in_channels, args.target_height, args.target_width],
        out_dims=args.label_dimensions,
        dropout_prob=args.dropout_prob,
        options=net_options)
    nets.append(net)
    net.cuda()

    net_train_settings = optimize.TrainSettings(
        optimize.LossSettings(optimize.WeightedMSELoss()),
        torch.optim.Adam(net.parameters(), lr=args.learning_rate),
        args.epochs)
    train_settings.append(net_train_settings)

  optimize.TrainModels(
      nets,
      train_loader,
      val_loader,
      train_settings,
      args.out_prefix,
      batch_use_prob=args.batch_use_prob,
      log_dir=args.log_dir)
