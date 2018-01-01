import argparse
import collections
import glob
import json
import os

import io_helpers
import models
import optimize
import training_helpers

import torch.multiprocessing
import torch.utils.data

global train_data
global val_data

TrainingFoldSettings = collections.namedtuple(
    'TrainingFoldSettings',
    [
        'training_settings_json',
        'epochs',
        'base_out_dir',
        'base_log_dir',
        'base_preload_dir',
        'num_nets_to_train',
        'batch_use_prob'
    ])


def RunTraining(fold_settings):
  preload_names = None
  # TODO factor out last model naming.
  if fold_settings.base_preload_dir is not None:
    preload_names = [
        os.path.join(
          fold_settings.base_preload_dir,
          fold_settings.training_settings_json[training_helpers.SETTINGS_ID],
          'model-' + str(i) + '-last.pth')
        for i in range(fold_settings.num_nets_to_train)]

  learners, train_loader, val_loader, train_settings = (
      training_helpers.MakeTrainer(
          train_data,
          val_data,
          fold_settings.training_settings_json,
          fold_settings.num_nets_to_train,
          fold_settings.epochs,
          preload_weight_names=preload_names))

  out_dir = os.path.join(
    fold_settings.base_out_dir,
    fold_settings.training_settings_json[training_helpers.SETTINGS_ID])
  if not os.path.isdir(out_dir):
    os.mkdir(out_dir)
  out_prefix = os.path.join(out_dir, 'model')

  log_dir = os.path.join(
    fold_settings.base_log_dir,
    fold_settings.training_settings_json[training_helpers.SETTINGS_ID])

  optimize.TrainModels(
      learners,
      train_loader,
      val_loader,
      train_settings,
      out_prefix,
      batch_use_prob=fold_settings.batch_use_prob,
      print_log=False,
      log_dir=log_dir)
  print(fold_settings.training_settings_json[training_helpers.SETTINGS_ID])


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--data_dirs', required=True)
  parser.add_argument('--validation_data_dirs', required=True)
  parser.add_argument('--data_file_suffix', default='data.npz')
  parser.add_argument('--train_settings_json_glob', required=True)
  parser.add_argument('--epochs', type=int, required=True)
  parser.add_argument('--preload_dir', default=None)
  parser.add_argument('--out_dir', required=True)
  parser.add_argument('--log_dir', required=True)
  parser.add_argument('--parallelism', type=int, default=1)
  parser.add_argument('--num_nets_to_train', type=int, default=1,
      help='How many identically structured models to train simultaneously.')
  parser.add_argument('--batch_use_prob', type=float, default=1.0)
 
  args = parser.parse_args()
  train_settings_json_names = glob.glob(args.train_settings_json_glob)
  train_settings_jsons = []
  for train_settings_json_name in train_settings_json_names:
    with open(train_settings_json_name) as f:
      train_settings_jsons.append(json.load(f))

  per_fold_settings = [
    TrainingFoldSettings(
        training_settings_json=train_settings,
        epochs=args.epochs,
        base_out_dir=args.out_dir,
        base_log_dir=args.log_dir,
        base_preload_dir=args.preload_dir,
        num_nets_to_train=args.num_nets_to_train,
        batch_use_prob=args.batch_use_prob)
    for train_settings in train_settings_jsons]

  # TODO check that all this matches across settings.
  data_element_names = per_fold_settings[0].training_settings_json[training_helpers.INPUT_NAMES] + per_fold_settings[0].training_settings_json[training_helpers.LABEL_NAMES]
  train_data = io_helpers.LoadDatasetNumpyFiles(
      args.data_dirs.split(','),
      data_element_names,
      data_suffix=args.data_file_suffix)
  val_data = io_helpers.LoadDatasetNumpyFiles(
      args.validation_data_dirs.split(','),
      data_element_names,
      data_suffix=args.data_file_suffix)


  with torch.multiprocessing.Pool(args.parallelism) as p:
    p.map(RunTraining, per_fold_settings)
