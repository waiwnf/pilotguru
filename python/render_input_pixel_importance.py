import argparse
import json

import image_helpers
import io_helpers
import models
import training_helpers

import numpy as np
import skvideo.io
import torch
from torch.autograd import Variable

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument(
    '--in_video', required=True,
    help='MPEG video file from Pilotguru Recorder.')
  parser.add_argument('--out_video', required=True)
  parser.add_argument('--forward_axis_json', required=True)
  parser.add_argument('--net_settings_json', required=True)
  parser.add_argument(
    '--in_model_weights', required=True, help='Pytorch model weights files.')
  parser.add_argument('--convert_to_yuv', type=bool, default=False)
  parser.add_argument('--batch_size', type=int, default=10)
  parser.add_argument('--frames_to_skip', type=int, default=0)
  parser.add_argument('--max_out_frames', type=int, default=-1)
  parser.add_argument('--cuda_device_id', type=int, default=0)
  parser.add_argument(
      '--saturation_gradient_magnitude', type=float, default=0.01)
  
  # Crop settings
  parser.add_argument('--crop_top', type=int, default=0)
  parser.add_argument('--crop_bottom', type=int, default=0)
  parser.add_argument('--crop_left', type=int, default=0)
  parser.add_argument('--crop_right', type=int, default=0)

  args = parser.parse_args()

  with open(args.net_settings_json, 'r') as f:
    net_settings = json.load(f)

  # Load the forward axis, replicate it batch_size times and transfer to GPU.
  forward_axis = io_helpers.LoadForwardAxis(args.forward_axis_json)
  forward_axis_batch = np.repeat(
      forward_axis[np.newaxis, ...], args.batch_size, axis=0)
  forward_axis_tensor = Variable(
      torch.from_numpy(forward_axis_batch)).cuda(args.cuda_device_id)
    
  # Gradients wrt output label to be used for backpropagation. We will only look
  # at gradient wrt the first frame, so set all the other gradients to zero.
  output_gradient = np.zeros(
      [args.batch_size, net_settings[training_helpers.LABEL_DIMENSIONS]],
      dtype=np.float32)
  output_gradient[:,0] = 1.0
  output_gradient_tensor = Variable(torch.from_numpy(output_gradient)).cuda(
      args.cuda_device_id)

  # Load the model and transfer to GPU.
  # TODO factor out from here and predict_video.py
  net = models.MakeNetwork(
      net_settings[training_helpers.NET_NAME],
      in_shape=[
          net_settings[training_helpers.IN_CHANNELS],
          net_settings[training_helpers.TARGET_HEIGHT],
          net_settings[training_helpers.TARGET_WIDTH]],
      out_dims=net_settings[training_helpers.LABEL_DIMENSIONS],
      options=net_settings[training_helpers.NET_OPTIONS])
  net.load_state_dict(torch.load(args.in_model_weights))
  net.cuda(args.cuda_device_id)
  net.eval()

  out_video = skvideo.io.FFmpegWriter(
    args.out_video,
    inputdict={'-r': '30.0'},
    outputdict={'-r': '30.0', '-crf': '17', '-preset': 'slow'})

  # Upsampler from model input dimensions to the cropped source video.
  # Lazily initialized inside the loop as we do not know the source video
  # dimensions yet.
  upsampler = None

  # Numpy batch for storing cropped (but not yet resized to model input shape)
  # input frames. Lazily initialized inside the loop.
  frames_cropped = None
  
  # Numpy batch of input frames resized to model input dimensions in RGB
  # colorspace.
  frames_resized = np.zeros(
    shape=[args.batch_size,
        net_settings[training_helpers.TARGET_HEIGHT],
        net_settings[training_helpers.TARGET_WIDTH],
        3], dtype=np.uint8)
  
  # Numpy batch of input frames resized to model input dimensions in model
  # colorspace (i.e. potentially converted to YUV).
  frame_colorspace = np.zeros(
    shape=[args.batch_size,
        net_settings[training_helpers.TARGET_HEIGHT],
        net_settings[training_helpers.TARGET_WIDTH],
        3], dtype=np.uint8)

  frames_generator = skvideo.io.vreader(args.in_video)
  for frame_index, raw_frame in enumerate(frames_generator):
    if frame_index < args.frames_to_skip:
      continue

    rendered_frames = frame_index - args.frames_to_skip
    in_batch_idx = rendered_frames % args.batch_size

    cropped_frame = image_helpers.CropHWC(
        raw_frame,
        args.crop_top, args.crop_bottom, args.crop_left, args.crop_right)
    
    # If required, initialize the parts that depend on the input video
    # dimensions.
    if upsampler is None:
      upsampler = torch.nn.Upsample(
        cropped_frame.shape[0:2], mode='bilinear').cuda()
    if frames_cropped is None:
      frames_cropped = np.zeros(
          shape=((args.batch_size,) + cropped_frame.shape), dtype=np.uint8)

    frames_cropped[in_batch_idx, ...] = cropped_frame
    frames_resized[in_batch_idx, ...] = image_helpers.MaybeResizeHWC(
        frames_cropped[in_batch_idx, ...],
        net_settings[training_helpers.TARGET_HEIGHT],
        net_settings[training_helpers.TARGET_WIDTH])
    if args.convert_to_yuv:
      frame_colorspace[in_batch_idx, ...] = image_helpers.RgbToYuv(
          frames_resized[in_batch_idx, ...])
    else:
      frame_colorspace[in_batch_idx, ...] = frames_resized[in_batch_idx, ...]
    
    if in_batch_idx != args.batch_size - 1:
      continue

    frame_chw = np.transpose(frame_colorspace, (0, 3, 1 ,2))
    frame_float = frame_chw.astype(np.float32) / 255.0
    frame_input_tensor = Variable(
        torch.from_numpy(frame_float), requires_grad=True).cuda(
            args.cuda_device_id)
    prediction = net([frame_input_tensor, forward_axis_tensor])
    input_gradient = torch.autograd.grad(
        prediction, [frame_input_tensor], output_gradient_tensor)[0]
    # We are interest in gradient magnitude, not direction. Take the absolute
    # gradient, and a max across color channels.
    gradient_abs = torch.abs(input_gradient)
    gradient_1ch, _ = torch.max(gradient_abs, dim=1, keepdim=True)
    # Upsample to the original crop size for overlaying.
    gradient_upsampled = upsampler(gradient_1ch)
    gradient_upsampled = gradient_upsampled.squeeze(1)
    gradient_np = gradient_upsampled.cpu().data.numpy()
    gradient_normalized = np.clip(
        gradient_np / args.saturation_gradient_magnitude * 255.0, 0.0, 255.0)

    effective_green = np.maximum(
      frames_cropped[:, :, :, 1], gradient_normalized.astype(np.uint8))
    frames_cropped[:, :, :, 1] = effective_green

    for k in range(args.batch_size):
      out_video.writeFrame(frames_cropped[k,...])
    
    if args.max_out_frames > 0 and rendered_frames >= args.max_out_frames:
      break

  out_video.close()