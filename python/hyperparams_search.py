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
  # Select the least used cuda device.
  cuda_device_assign_lock.acquire()
  # ctypes arrays do not support the standard list methods, so make a local
  # copy for convenience.
  cuda_device_use_count_copy = [x for x in cuda_device_use_count]
  cuda_device_count_idx = cuda_device_use_count_copy.index(
      min(cuda_device_use_count_copy))
  cuda_device_id = cuda_device_ids[cuda_device_count_idx]
  cuda_device_use_count[cuda_device_count_idx] += 1
  cuda_device_assign_lock.release()

  preload_names = None
  if fold_settings.base_preload_dir is not None:
    full_preload_dir = os.path.join(
        fold_settings.base_preload_dir,
        fold_settings.training_settings_json[training_helpers.SETTINGS_ID])
    preload_names = io_helpers.PreloadModelNames(
        full_preload_dir, fold_settings.num_nets_to_train)

  learners, train_loader, val_loader, train_settings = (
      training_helpers.MakeTrainer(
          train_data,
          val_data,
          fold_settings.training_settings_json,
          fold_settings.num_nets_to_train,
          fold_settings.epochs,
          cuda_device_id=cuda_device_id,
          preload_weight_names=preload_names))

  out_dir = os.path.join(
    fold_settings.base_out_dir,
    fold_settings.training_settings_json[training_helpers.SETTINGS_ID])
  if not os.path.isdir(out_dir):
    os.mkdir(out_dir)

  log_dir = os.path.join(
    fold_settings.base_log_dir,
    fold_settings.training_settings_json[training_helpers.SETTINGS_ID])

  optimize.TrainModels(
      learners,
      train_loader,
      val_loader,
      train_settings,
      out_dir,
      cuda_device_id=cuda_device_id,
      batch_use_prob=fold_settings.batch_use_prob,
      print_log=False,
      log_dir=log_dir)
  print(fold_settings.training_settings_json[training_helpers.SETTINGS_ID])

  # Release CUDA device.
  cuda_device_assign_lock.acquire()
  cuda_device_use_count[cuda_device_count_idx] -= 1
  cuda_device_assign_lock.release()

def setup(ids, use_count, cuda_lock):
    global cuda_device_ids
    global cuda_device_use_count
    global cuda_device_assign_lock
    cuda_device_ids = ids
    cuda_device_use_count = use_count
    cuda_device_assign_lock = cuda_lock

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
  parser.add_argument('--cuda_device_ids', default='0')
 
  args = parser.parse_args()
  train_settings_jsons = []
  for train_settings_json_glob in args.train_settings_json_glob.split(','):
    for train_settings_json_name in glob.glob(args.train_settings_json_glob):
      with open(train_settings_json_name) as f:
        train_settings_jsons.append(json.load(f))

  cuda_device_ids = torch.multiprocessing.Array(
      'i', [int(i) for i in args.cuda_device_ids.split(',')])
  cuda_device_use_count = torch.multiprocessing.Array(
      'i', len(cuda_device_ids))
  cuda_device_assign_lock = torch.multiprocessing.Lock()

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
  first_settings_json = per_fold_settings[0].training_settings_json
  data_element_names = (
      first_settings_json[training_helpers.INPUT_NAMES] + 
      first_settings_json[training_helpers.LABEL_NAMES])
  train_data = io_helpers.LoadDatasetNumpyFiles(
      args.data_dirs.split(','),
      data_element_names,
      data_suffix=args.data_file_suffix)
  val_data = io_helpers.LoadDatasetNumpyFiles(
      args.validation_data_dirs.split(','),
      data_element_names,
      data_suffix=args.data_file_suffix)

  with torch.multiprocessing.Pool(
      args.parallelism,
      setup,
      [cuda_device_ids, cuda_device_use_count, cuda_device_assign_lock]) as p:
    p.map(RunTraining, per_fold_settings)
