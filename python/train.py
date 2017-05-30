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
  parser.add_argument('--in_height', type=int, required=True)
  parser.add_argument('--in_width', type=int, required=True)
  parser.add_argument('--batch_size', type=int, required=True)
  parser.add_argument('--epochs', type=int, required=True)
  args = parser.parse_args()

  trainset = io_helpers.ImageFrameDataset(args.data_dirs.split(','))
  trainloader = torch.utils.data.DataLoader(
      trainset, batch_size=args.batch_size, shuffle=True)

  net = models.ToyConvNet([3, args.in_height, args.in_width])
  net.cuda()
  
  loss = torch.nn.MSELoss()
  optimizer = torch.optim.Adam(net.parameters())

  optimize.TrainModel(net, trainloader, loss, optimizer, args.epochs)
