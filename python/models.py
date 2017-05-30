import math

import torch.nn as nn

def ConvOutSize(in_size, kernel_size, stride=1, padding=0, dilation=1):
  return math.floor(
      (in_size + 2 * padding - dilation * (kernel_size - 1) - 1) / stride + 1)

def ConvOutShape(in_shape, kernel_size, stride=1, padding=0, dilation=1):
  return [
      ConvOutSize(x, kernel_size, stride, padding, dilation) for x in in_shape]

def AddConv2d(shapes, out_channels, kernel_size):
  prev_shape = shapes[-1]
  in_channels = prev_shape[0]
  out_shape = [out_channels] + ConvOutShape(prev_shape[1:], kernel_size)
  shapes.append(out_shape)
  
  return nn.Conv2d(in_channels, out_channels, kernel_size)

def AddMaxPool2d(shapes, kernel_size):
  prev_shape = shapes[-1]
  in_channels = prev_shape[0]
  out_shape = [in_channels] + ConvOutShape(
      prev_shape[1:], kernel_size, stride=kernel_size)
  shapes.append(out_shape)
  
  return nn.MaxPool2d(kernel_size, kernel_size)

class ToyConvNet(nn.Module):
  """Simple 3-conv + 3-fc model, mostly for debugging purposes."""

  def __init__(self, in_shape):
    super(ToyConvNet, self).__init__()
    self.shapes = [in_shape]

    self.conv1 = AddConv2d(self.shapes, 6, 5)
    self.pool1 = AddMaxPool2d(self.shapes, 2)

    self.conv2 = AddConv2d(self.shapes, 16, 5)
    self.pool2 = AddMaxPool2d(self.shapes, 2)
    
    self.conv3 = AddConv2d(self.shapes, 1, 1)
    self.pool3 = AddMaxPool2d(self.shapes, 2)

    conv_out_shape = self.shapes[-1]
    self.fc_in_elements = (
        conv_out_shape[0] * conv_out_shape[1] * conv_out_shape[2])
    
    self.fc1 = nn.Linear(self.fc_in_elements, 120)
    self.fc2 = nn.Linear(120, 84)
    self.fc3 = nn.Linear(84, 1)


  def forward(self, x):
    x = self.pool1(nn.functional.relu(self.conv1(x)))
    x = self.pool1(nn.functional.relu(self.conv2(x)))
    x = self.pool1(nn.functional.relu(self.conv3(x)))
    x = x.view(-1, self.fc_in_elements)
    x = nn.functional.relu(self.fc1(x))
    x = nn.functional.relu(self.fc2(x))
    x = self.fc3(x)
    return x

