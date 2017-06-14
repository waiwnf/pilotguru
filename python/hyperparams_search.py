import argparse
import collections
import json
import os
import random
import time

import augmentation
import image_helpers
import io_helpers
import models
import optimize
import training_helpers

import numpy as np
import torch.multiprocessing
import torch.utils.data

global train_data
global train_labels
global val_data
global val_labels
global pca_directions

TrainingFoldSettings = collections.namedtuple(
  'TrainingFoldSettings',
  ['settings_id', 'fold', 'out_dir', 'target_height', 'target_width',
      'dropout_prob', 'batch_size', 'epochs', 'augment_settings'])

def ParseRange(range_str):
  result = [float(x) for x in range_str.split(':')]
  assert len(result) == 2
  assert result[0] <= result[1]
  return result

def SampleStringRange(range_str):
  left, right = ParseRange(range_str)
  x = random.uniform(0.0, 1.0)
  return left * (1.0 - x) + right * x

def RunTraining(settings):
  train_loader, val_loader = training_helpers.MakeDataLoaders(
      train_data,
      train_labels,
      val_data,
      val_labels,
      settings.target_width,
      settings.augment_settings,
      settings.batch_size,
      0.0)  # TODO support examples reweighting here

  net = models.NvidiaSingleFrameNet(
      [3, settings.target_height, settings.target_width], settings.dropout_prob)
  net.cuda()
  
  train_settings = optimize.TrainSettings(
      optimize.LossSettings(optimize.WeightedMSELoss()),
      torch.optim.Adam(net.parameters()),
      settings.epochs)

  full_fold_id = '%s-%d' %(settings.settings_id, settings.fold)
  out_prefix = os.path.join(settings.out_dir, full_fold_id)
  train_log = optimize.TrainModel(
      net,
      train_loader,
      val_loader,
      train_settings,
      out_prefix,
      False)
  out_log_name = out_prefix + '-log.json'
  settings_dict = settings._asdict()
  settings_dict['augment_settings'] = settings.augment_settings._asdict()
  settings_dict['augment_settings']['random_shift_directions'] = (
      settings.augment_settings.random_shift_directions is not None)
  fold_log = {'train_log': train_log, 'settings': settings_dict}
  with open(out_log_name, 'w') as out_log_file:
    json.dump(fold_log, out_log_file, indent=2)
  return train_log

def MaybeComputePcaDirections(data, actually_compute):
  if actually_compute:
    return image_helpers.GetPcaRgbDirections(data.astype(np.float32) / 255.0)
  else:
    return None

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--data_dirs', required=True)
  parser.add_argument('--validation_data_dirs', required=True)
  parser.add_argument('--labels_file_suffix', default='inverse-radius')
  parser.add_argument('--num_combinations', type=int, required=True)
  parser.add_argument('--batch_size', type=int, required=True)
  parser.add_argument('--epochs', type=int, required=True)
  parser.add_argument('--samples_per_combination', type=int, required=True)
  parser.add_argument('--target_height', type=int, required=True)
  parser.add_argument('--target_width', type=int, required=True)
  parser.add_argument('--out_dir', required=True)
  parser.add_argument('--parallelism', type=int, default=1)

  parser.add_argument('--dropout_prob_range', default='0.0:0.0')
  parser.add_argument('--max_horizontal_shift_pixels_range', default='0.0:0.0')
  parser.add_argument('--horizontal_label_shift_rate_range', default='0.0:0.0')
  parser.add_argument('--train_blur_sigma_range', default='2.0:2.0')
  parser.add_argument('--train_blur_prob_range', default='0.0:0.0')
  parser.add_argument('--maybe_do_pca_random_shifts', type=bool, default=False)
  parser.add_argument('--grayscale_interpolate_prob_range', default='0.0:0.0')
  
  args = parser.parse_args()

  train_data, train_labels = io_helpers.LoadDatasetNumpyFiles(
      args.data_dirs.split(','), label_suffix=args.labels_file_suffix)
  val_data, val_labels = io_helpers.LoadDatasetNumpyFiles(
      args.validation_data_dirs.split(','),
      label_suffix=args.labels_file_suffix)
  pca_directions = MaybeComputePcaDirections(
      train_data, args.maybe_do_pca_random_shifts)

  id_prefix = time.strftime('%Y%m%d-%H%M%S')
  work_queue = []
  for combination_id in range(args.num_combinations):
    random_shift_directions = None
    if args.maybe_do_pca_random_shifts and random.randint(0,1) > 0:
      random_shift_directions = pca_directions
    augment_settings = augmentation.AugmentSettings(
        target_width=args.target_width,
        max_horizontal_shift_pixels=round(SampleStringRange(
            args.max_horizontal_shift_pixels_range)),
        horizontal_label_shift_rate=SampleStringRange(
            args.horizontal_label_shift_rate_range),
        blur_sigma=SampleStringRange(args.train_blur_sigma_range),
        blur_prob=SampleStringRange(args.train_blur_prob_range),
        grayscale_interpolate_prob=SampleStringRange(
            args.grayscale_interpolate_prob_range),
        random_shift_directions=random_shift_directions)
    drouput_prob = SampleStringRange(args.dropout_prob_range)

    for fold in range(args.samples_per_combination):
      settings_id = '%s-%d' % (id_prefix, combination_id)
      work_queue.append(TrainingFoldSettings(
          settings_id, fold, args.out_dir,
          args.target_height, args.target_width,
          drouput_prob, args.batch_size, args.epochs, augment_settings))

  with torch.multiprocessing.Pool(args.parallelism) as p:
    p.map(RunTraining, work_queue)
