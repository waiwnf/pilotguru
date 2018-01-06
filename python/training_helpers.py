import augmentation
import io_helpers
import models
import optimize

import numpy as np

import torch.utils.data

SETTINGS_ID = 'settings_id'
NET_NAME = 'net_name'
INPUT_NAMES = 'input_names'
LABEL_NAMES = 'label_names'
IN_CHANNELS = 'in_channels'
TARGET_HEIGHT = 'target_height'
TARGET_WIDTH = 'target_width'
NET_HEAD_DIMS = 'net_head_dims'
LABEL_DIMENSIONS = 'label_dimensions'
DROPOUT_PROB = 'dropout_prob'
NET_OPTIONS = 'net_options'
LEARNING_RATE = 'learning_rate'
MAX_HORIZONTAL_SHIFT_PIXELS = 'max_horizontal_shift_pixels'
HORIZONTAL_LABEL_SHIFT_RATE = 'horizontal_label_shift_rate'
TRAIN_BLUR_SIGMA = 'train_blur_sigma'
TRAIN_BLUR_PROB = 'train_blur_prob'
DO_PCA_RANDOM_SHIFTS = 'do_pca_random_shifts'
GRAYSCALE_INTERPOLATE_PROB = 'grayscale_interpolate_prob'
BATCH_SIZE = 'batch_size'
EXAMPLE_LABEL_EXTRA_WEIGHT_SCALE = 'example_lable_extra_weight_scale'
DO_PCA_RANDOM_SHIFTS = 'do_pca_random_shifts'

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
  
def MakeTrainer(
    train_data,
    val_data,
    all_settings,
    num_nets_to_train,
    epochs,
    cuda_device_id=0,
    preload_weight_names=None):
  learners = []
  for net_idx in range(num_nets_to_train):
    net = models.MakeNetwork(
        all_settings[NET_NAME],
        in_shape=[all_settings[IN_CHANNELS], all_settings[TARGET_HEIGHT], all_settings[TARGET_WIDTH]],
        head_dims=all_settings[NET_HEAD_DIMS],
        out_dims=all_settings[LABEL_DIMENSIONS],
        dropout_prob=all_settings[DROPOUT_PROB],
        options=all_settings[NET_OPTIONS])
    assert net.InputNames() == all_settings[INPUT_NAMES]
    assert net.LabelNames() == all_settings[LABEL_NAMES]

    if preload_weight_names is not None:
      assert len(preload_weight_names) == num_nets_to_train
      net.load_state_dict(torch.load(preload_weight_names[net_idx]))

    net.cuda(cuda_device_id)
    optimizer = torch.optim.Adam(net.parameters(), lr=all_settings[LEARNING_RATE])
    learners.append(optimize.Learner(net, optimizer))

  train_settings = optimize.TrainSettings(
      optimize.SingleLabelLoss(optimize.WeightedMSELoss()),
      epochs)
  
  data_element_names = all_settings[INPUT_NAMES] + all_settings[LABEL_NAMES]
  image_element_idx = data_element_names.index(models.FRAME_IMG)
  weight_label_idx = data_element_names.index(models.STEERING)

  random_shift_directions = None
  if all_settings[DO_PCA_RANDOM_SHIFTS]:
    random_shift_directions = image_helpers.GetPcaRgbDirections(
          train_data[image_element_idx].astype(np.float32) / 255.0)

  horizontal_label_shift_rate = np.array(
      all_settings[HORIZONTAL_LABEL_SHIFT_RATE], dtype=np.float32)

  augment_settings = augmentation.AugmentSettings(
      target_width=all_settings[TARGET_WIDTH],
      max_horizontal_shift_pixels=all_settings[MAX_HORIZONTAL_SHIFT_PIXELS],
      horizontal_label_shift_rate=horizontal_label_shift_rate,
      blur_sigma=all_settings[TRAIN_BLUR_SIGMA],
      blur_prob=all_settings[TRAIN_BLUR_PROB],
      grayscale_interpolate_prob=all_settings[GRAYSCALE_INTERPOLATE_PROB],
      random_shift_directions=random_shift_directions)
  
  train_loader, val_loader = MakeDataLoaders(
      train_data,
      val_data,
      image_element_idx,
      weight_label_idx,
      all_settings[TARGET_WIDTH],
      augment_settings,
      all_settings[BATCH_SIZE],
      all_settings[EXAMPLE_LABEL_EXTRA_WEIGHT_SCALE])
  
  return learners, train_loader, val_loader, train_settings