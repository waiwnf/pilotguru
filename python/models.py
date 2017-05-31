import math

import torch.nn as nn

def ConvOutSize(in_size, kernel_size, stride=1, padding=0, dilation=1):
  return math.floor(
      (in_size + 2 * padding - dilation * (kernel_size - 1) - 1) / stride + 1)

def ConvOutShape(in_shape, kernel_size, stride=1, padding=0, dilation=1):
  return [
      ConvOutSize(x, kernel_size, stride, padding, dilation) for x in in_shape]

def TotalElements(shape):
  assert len(shape) > 0
  result = 1
  for x in shape:
    result = result * x
  return result

def MakeConv2d(in_shape, out_channels, kernel_size):
  in_channels = in_shape[0]
  out_shape = [out_channels] + ConvOutShape(in_shape[1:], kernel_size)
  return nn.Conv2d(in_channels, out_channels, kernel_size), out_shape

def MakeMaxPool2d(in_shape, kernel_size):
  in_channels = in_shape[0]
  out_shape = ([in_channels] + 
      ConvOutShape(in_shape[1:], kernel_size, stride=kernel_size))
  return nn.MaxPool2d(kernel_size, kernel_size), out_shape

def MakeRelu(in_shape):
  out_shape = in_shape
  layer = lambda x : nn.functional.relu(x)
  return layer, out_shape

def MakeLinear(in_shape, out_size):
  assert len(in_shape) == 1
  return nn.Linear(in_shape[0], out_size), [out_size]

def MakeFlatten(in_shape):
  assert len(in_shape) > 0
  out_size = TotalElements(in_shape)
  layer = lambda x : x.view(-1, out_size)
  return layer, [out_size]

def MakeBatchNorm2d(in_shape):
  assert len(in_shape) == 3
  return nn.BatchNorm2d(in_shape[0]), in_shape

class SequentialNet(nn.Module):
  def __init__(self, in_shape):
    super(SequentialNet, self).__init__()
    self.layers = []
    self.out_shapes = [in_shape]
  
  def OutShape(self):
    return self.out_shapes[-1]
  
  def AddConv2d(self, out_channels, kernel_size):
    return self.AddLayer(MakeConv2d(self.OutShape(), out_channels, kernel_size))

  def AddMaxPool2d(self, kernel_size):
    return self.AddLayer(MakeMaxPool2d(self.OutShape(), kernel_size))

  def AddRelu(self):
    return self.AddLayer(MakeRelu(self.OutShape()))

  def AddLinear(self, out_size):
    return self.AddLayer(MakeLinear(self.OutShape(), out_size))
  
  def AddFlatten(self):
    return self.AddLayer(MakeFlatten(self.OutShape()))
  
  def AddBatchNorm2d(self):
    return self.AddLayer(MakeBatchNorm2d(self.OutShape()))

  def AddLayer(self, layer_tuple):
    layer, out_shape = layer_tuple
    self.layers.append(layer)
    self.out_shapes.append(out_shape)
    return layer
  
  def forward(self, x):
    result = x
    for layer in self.layers:
      result = layer(result)
    return result


class ToyConvNet(SequentialNet):
  """Simple 3-conv + 3-fc model, mostly for debugging purposes."""

  def __init__(self, in_shape):
    super(ToyConvNet, self).__init__(in_shape)

    self.conv1 = self.AddConv2d(6, 5)
    self.c1bn = self.AddBatchNorm2d()
    self.AddRelu()
    self.pool1 = self.AddMaxPool2d(2)

    self.conv2 = self.AddConv2d(16, 5)
    self.c2bn = self.AddBatchNorm2d()
    self.AddRelu()
    self.pool2 = self.AddMaxPool2d(2)

    self.conv3 = self.AddConv2d(1, 5)
    self.c3bn = self.AddBatchNorm2d()
    self.AddRelu()
    self.pool3 = self.AddMaxPool2d(2)

    self.AddFlatten()

    self.fc1 = self.AddLinear(120)
    self.AddRelu()
    self.fc2 = self.AddLinear(84)
    self.AddRelu()
    self.fc3 = self.AddLinear(1)
