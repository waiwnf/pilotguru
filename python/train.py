import argparse
import os
import sys

import io_helpers
import json
import models
import optimize
import sample_weighting
import training_helpers

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--data_dirs', required=True)
  parser.add_argument('--validation_data_dirs', required=True)
  parser.add_argument('--data_file_suffix', default='data.npz')
  parser.add_argument('--batch_size', type=int, required=True)
  parser.add_argument('--batch_use_prob', type=float, default=1.0)
  parser.add_argument('--epochs', type=int, required=True)
  parser.add_argument('--optimizer', default=training_helpers.SGD)
  parser.add_argument('--learning_rate', type=float, default=1e-3)
  parser.add_argument('--loss_norm_pow', type=float, default=2.0)
  parser.add_argument('--plateau_patience_epochs', type=int, default=0)
  parser.add_argument('--in_channels', type=int, default=3)
  parser.add_argument('--target_height', type=int, required=True)
  parser.add_argument('--target_width', type=int, required=True)
  parser.add_argument('--net_name', default=models.NVIDIA_NET_NAME)
  parser.add_argument('--net_input_names',
      default=','.join([models.FRAME_IMG, models.FORWARD_AXIS]))
  parser.add_argument('--net_label_names', default=models.STEERING)
  parser.add_argument('--net_head_dims', type=int, default=10,
      help='Dimensionality of the penultimate layer (just before the actual '
          'predictions)')
  parser.add_argument(
      '--linear_bias_options',
      default=json.dumps([
          {
              training_helpers.INPUT_NAME: models.FORWARD_AXIS,
              training_helpers.INPUT_DIMS: 3
          }]))
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
            models.DROPOUT: models.DROPOUT_VANILLA}}))
  parser.add_argument('--label_dimensions', type=int, default=1)
  parser.add_argument('--out_dir', required=True)
  parser.add_argument('--log_dir', default='')
  parser.add_argument('--base_preload_dir', default=None)
  parser.add_argument('--dropout_prob', type=float, default=0.0)
  parser.add_argument('--max_horizontal_shift_pixels', type=int, default=0)
  parser.add_argument('--horizontal_label_shift_rate', default="0.0",
      help='Must be either a scalar, or same dimensionality as the labels.')
  parser.add_argument('--train_blur_sigma', type=float, default=2.0)
  parser.add_argument('--train_blur_prob', type=float, default=0.0)
  parser.add_argument('--do_pca_random_shifts', type=bool, default=False)
  parser.add_argument('--grayscale_interpolate_prob', type=float, default=0.0)
  parser.add_argument('--sample_weighter_options',
      default=json.dumps({sample_weighting.NAME: sample_weighting.UNIFORM}))
  parser.add_argument('--dry_run', type=bool, default=False)
  parser.add_argument('--settings_id', default='')  
  parser.add_argument('--cuda_device_id', type=int, default=0)

  args = parser.parse_args()

  all_settings = {
    training_helpers.SETTINGS_ID: args.settings_id,
    models.NET_NAME: args.net_name,
    training_helpers.INPUT_NAMES: args.net_input_names.split(','),
    training_helpers.LABEL_NAMES: args.net_label_names.split(','),
    training_helpers.IN_CHANNELS: args.in_channels,
    training_helpers.TARGET_HEIGHT: args.target_height,
    training_helpers.TARGET_WIDTH: args.target_width,
    models.NET_HEAD_DIMS: args.net_head_dims,
    models.LABEL_DIMENSIONS: args.label_dimensions,
    models.DROPOUT_PROB: args.dropout_prob,
    models.LAYER_BLOCKS_OPTIONS: json.loads(args.net_options),
    training_helpers.LINEAR_BIAS_OPTIONS: json.loads(args.linear_bias_options),
    training_helpers.OPTIMIZER: args.optimizer,
    training_helpers.LEARNING_RATE: args.learning_rate,
    training_helpers.LOSS_NORM_POW: args.loss_norm_pow,
    training_helpers.PLATEAU_PATIENCE_EPOCHS: args.plateau_patience_epochs,
    training_helpers.MAX_HORIZONTAL_SHIFT_PIXELS: args.max_horizontal_shift_pixels,
    training_helpers.HORIZONTAL_LABEL_SHIFT_RATE: [
        float(x) for x in args.horizontal_label_shift_rate.split(',')],
    training_helpers.TRAIN_BLUR_SIGMA: args.train_blur_sigma,
    training_helpers.TRAIN_BLUR_PROB: args.train_blur_prob,
    training_helpers.GRAYSCALE_INTERPOLATE_PROB: args.grayscale_interpolate_prob,
    training_helpers.BATCH_SIZE: args.batch_size,
    training_helpers.SAMPLE_WEIGHTER_OPTIONS:
        json.loads(args.sample_weighter_options),
    training_helpers.DO_PCA_RANDOM_SHIFTS: args.do_pca_random_shifts
  }

  if args.dry_run:
    print(json.dumps(all_settings, indent=2, sort_keys=True))
    sys.exit(0)

  preload_names = io_helpers.PreloadModelNames(
      args.base_preload_dir, args.num_nets_to_train)

  data_element_names = (
      all_settings[training_helpers.INPUT_NAMES] +
      all_settings[training_helpers.LABEL_NAMES])
  train_data = io_helpers.LoadDatasetNumpyFiles(
      args.data_dirs.split(','),
      data_element_names,
      data_suffix=args.data_file_suffix)
  val_data = io_helpers.LoadDatasetNumpyFiles(
      args.validation_data_dirs.split(','),
      data_element_names,
      data_suffix=args.data_file_suffix)

  learners, train_loader, val_loader, train_settings = training_helpers.MakeTrainer(
      train_data,
      val_data,
      all_settings,
      args.num_nets_to_train,
      args.epochs,
      cuda_device_id=args.cuda_device_id,
      preload_weight_names=preload_names)

  optimize.TrainModels(
      learners,
      train_loader,
      val_loader,
      train_settings,
      args.out_dir,
      cuda_device_id=args.cuda_device_id,
      batch_use_prob=args.batch_use_prob,
      log_dir=args.log_dir)
