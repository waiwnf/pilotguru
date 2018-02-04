import image_helpers
import models
import training_helpers

import numpy as np

import torch

def AddCropArgs(parser):
  parser.add_argument('--crop_top', type=int, default=0)
  parser.add_argument('--crop_bottom', type=int, default=0)
  parser.add_argument('--crop_left', type=int, default=0)
  parser.add_argument('--crop_right', type=int, default=0)

def UpdateFutureTrajectoryPrediction(previous_prediction, current_update, lr):
  assert len(current_update.shape) == 2
  assert current_update.shape[0] == 1
  assert lr > 0
  assert lr <= 1

  if previous_prediction is None:
    return np.copy(current_update)

  assert previous_prediction.shape == current_update.shape
  result = np.copy(previous_prediction)
  result[0,:-1] = (
    lr * current_update[0,:-1] + (1.0 - lr) * previous_prediction[0,1:])
  result[0, -1] = current_update[0,-1]
  return result

def EvalModelsEnsemble(models, in_variables):
  result_components = np.array([
      net(in_variables)[0].cpu().data.numpy() for net in models])
  return np.mean(result_components, axis=0, keepdims=False)

def RawFrameToModelInput(
    raw_frame, crop_settings, net_settings, convert_to_yuv, cuda_device_id):
  frame_cropped = image_helpers.CropHWC(
      raw_frame,
      crop_settings.crop_top,
      crop_settings.crop_bottom,
      crop_settings.crop_left,
      crop_settings.crop_right)
  frame_resized = image_helpers.MaybeResizeHWC(
      frame_cropped,
      net_settings[training_helpers.TARGET_HEIGHT],
      net_settings[training_helpers.TARGET_WIDTH])
  frame_colorspace = None
  if convert_to_yuv:
    frame_colorspace = image_helpers.RgbToYuv(frame_resized)
  else:
    frame_colorspace = frame_resized
  frame_chw = np.transpose(frame_colorspace, (2,0,1))
  frame_float = frame_chw.astype(np.float32) / 255.0
  # Add a dummy dimension to make it a "batch" of size 1.
  frame_variable = torch.autograd.Variable(
      torch.from_numpy(frame_float[np.newaxis,...]))
  return frame_variable.cuda(cuda_device_id), frame_resized

def LoadPredictionModels(in_model_weights_names, net_settings, cuda_device_id):
  nets = []
  for weights_filename in in_model_weights_names:
    # TODO Also load bias post transform modules.
    net = models.MakeNetwork(
        in_shape=[
            net_settings[training_helpers.IN_CHANNELS],
            net_settings[training_helpers.TARGET_HEIGHT],
            net_settings[training_helpers.TARGET_WIDTH]],
        options=net_settings,
        post_transform_modules=[])
    net.load_state_dict(torch.load(weights_filename), strict=False)
    net.cuda(cuda_device_id)
    net.eval()
    nets.append(net)
  return nets