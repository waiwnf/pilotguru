import argparse

import io_helpers
import models
import optimize

import torch.nn
import torch.optim
import torch.utils.data

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--data_dirs', required=True)
  parser.add_argument('--validation_data_dirs', required=True)
  parser.add_argument('--batch_size', type=int, required=True)
  parser.add_argument('--epochs', type=int, required=True)
  parser.add_argument('--target_height', type=int, required=True)
  parser.add_argument('--target_width', type=int, required=True)
  parser.add_argument('--out_prefix', required=True)
  parser.add_argument('--dropout_prob', type=float, default=0.0)
  parser.add_argument('--max_horizontal_shift_pixels', type=int, default=0)
  parser.add_argument('--horizontal_label_shift_rate', type=float, default=0.0)
  args = parser.parse_args()

  plain_train_data = io_helpers.InMemoryNumpyFileDataset(
      args.data_dirs.split(','))
  trainset = io_helpers.SteeringShiftAugmenterDataset(
      io_helpers.ImageFrameDataset(plain_train_data),
      args.target_width,
      args.max_horizontal_shift_pixels,
      args.horizontal_label_shift_rate)
  trainloader = torch.utils.data.DataLoader(
      trainset, batch_size=args.batch_size, shuffle=True)
  
  valset = io_helpers.ImageFrameDataset(
      io_helpers.InMemoryNumpyFileDataset(
          args.validation_data_dirs.split(',')),
      target_crop_width=args.target_width)
  validation_loader = torch.utils.data.DataLoader(
      valset, batch_size=args.batch_size, shuffle=False)

  net = models.NvidiaSingleFrameNet(
      [3, args.target_height, args.target_width], args.dropout_prob)
  net.cuda()
  
  loss = torch.nn.MSELoss()
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
