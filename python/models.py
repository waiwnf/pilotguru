import math

import torch
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

def MakeConv2d(in_shape, out_channels, kernel_size, stride):
  in_channels = in_shape[0]
  out_shape = [out_channels] + ConvOutShape(in_shape[1:], kernel_size, stride)
  layer = nn.Conv2d(in_channels, out_channels, kernel_size, stride=stride) 
  return layer, out_shape

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

def MakeBatchNorm1d(in_shape):
  assert len(in_shape) <= 2
  return nn.BatchNorm1d(in_shape[0]), in_shape

def MakeDropout2d(in_shape, p):
  return nn.Dropout2d(p), in_shape

def MakeDropout(in_shape, p):
  return nn.Dropout(p), in_shape

class SequentialNet(nn.Module):
  def __init__(self, in_shape):
    super(SequentialNet, self).__init__()
    self.layers = []
    self.out_shapes = [in_shape]
  
  def OutShape(self):
    return self.out_shapes[-1]
  
  def AddConv2d(self, out_channels, kernel_size, stride=1):
    return self.AddLayer(MakeConv2d(self.OutShape(), out_channels, kernel_size, stride))

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
  
  def AddBatchNorm1d(self):
    return self.AddLayer(MakeBatchNorm1d(self.OutShape()))
  
  def AddDrouput2d(self, p):
    return self.AddLayer(MakeDropout2d(self.OutShape(), p))
  
  def AddDropout(self, p):
    return self.AddLayer(MakeDropout(self.OutShape(), p))

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

class NvidiaSingleFrameNet(SequentialNet):

  def __init__(self, in_shape, drouput_prob):
    super(NvidiaSingleFrameNet, self).__init__(in_shape)
    self.conv1 = self.AddConv2d(24, 5, stride=2)
    self.c1bn = self.AddBatchNorm2d()
    self.AddRelu()
    self.drop1 = self.AddDrouput2d(drouput_prob)

    self.conv2 = self.AddConv2d(36, 5, stride=2)
    self.c2bn = self.AddBatchNorm2d()
    self.AddRelu()
    self.drop2 = self.AddDrouput2d(drouput_prob)

    self.conv3 = self.AddConv2d(48, 5, stride=2)
    self.c3bn = self.AddBatchNorm2d()
    self.AddRelu()
    self.drop3 = self.AddDrouput2d(drouput_prob)

    self.conv4 = self.AddConv2d(64, 3)
    self.c4bn = self.AddBatchNorm2d()
    self.AddRelu()
    self.drop4 = self.AddDrouput2d(drouput_prob)

    self.conv5 = self.AddConv2d(64, 3)
    self.c5bn = self.AddBatchNorm2d()
    self.AddRelu()
    self.drop5 = self.AddDrouput2d(drouput_prob)

    self.AddFlatten()

    self.fc1 = self.AddLinear(1164)
    self.fc1bn = self.AddBatchNorm1d()
    self.AddRelu()
    self.fc1_drop = self.AddDropout(drouput_prob)

    self.fc2 = self.AddLinear(100)
    self.fc2bn = self.AddBatchNorm1d()
    self.AddRelu()
    self.fc2_drop = self.AddDropout(drouput_prob)

    self.fc3 = self.AddLinear(50)
    self.fc3bn = self.AddBatchNorm1d()
    self.AddRelu()

    self.fc4 = self.AddLinear(10)
    self.AddRelu()

    self.fc5 = self.AddLinear(1)

class UdacityRamboNet(nn.Module):
  def __init__(self, in_shape, dropout_prob):
    super(UdacityRamboNet, self).__init__()

    self.comma_layers = []
    self.comma_shapes = [in_shape]
    self.nv1_layers = []
    self.nv1_shapes = [in_shape]
    self.nv2_layers = []
    self.nv2_shapes = [in_shape]

    self.comma_conv1, self.comma_bn1, self.comma_relu1, self.comma_drop1 = self.AddConvBlock(
        16, 8, 4, dropout_prob, self.comma_layers, self.comma_shapes)
    self.comma_conv2, self.comma_bn2, self.comma_relu2, self.comma_drop2 = self.AddConvBlock(
        32, 5, 2, dropout_prob, self.comma_layers, self.comma_shapes)
    self.comma_conv3, self.comma_bn3, self.comma_relu3, self.comma_drop3 = self.AddConvBlock(
        64, 5, 2, dropout_prob, self.comma_layers, self.comma_shapes)
    self.comma_flatten = self.AddLayer(
        MakeFlatten(self.comma_shapes[-1]),
        self.comma_layers, self.comma_shapes)
    self.comma_fc1, self.comma_fc1_bn, self.comma_fc1_relu = self.AddFcBlock(
        512, self.comma_layers, self.comma_shapes)
    self.comma_fc1_drop = self.AddLayer(
        MakeDropout(self.comma_shapes[-1], dropout_prob),
        self.comma_layers, self.comma_shapes)
    self.comma_fc2 = self.AddLayer(
        MakeLinear(self.comma_shapes[-1], 10),
        self.comma_layers, self.comma_shapes)
    
    self.nv1_conv1, self.nv1_bn1, self.nv1_relu1, self.nv1_drop1 = self.AddConvBlock(
        24, 5, 2, dropout_prob, self.nv1_layers, self.nv1_shapes)
    self.nv1_conv2, self.nv1_bn2, self.nv1_relu2, self.nv1_drop2 = self.AddConvBlock(
        36, 5, 2, dropout_prob, self.nv1_layers, self.nv1_shapes)
    self.nv1_conv3, self.nv1_bn3, self.nv1_relu3, self.nv1_drop3 = self.AddConvBlock(
        48, 5, 2, dropout_prob, self.nv1_layers, self.nv1_shapes)
    self.nv1_conv4, self.nv1_bn4, self.nv1_relu4, self.nv1_drop4 = self.AddConvBlock(
        64, 3, 2, dropout_prob, self.nv1_layers, self.nv1_shapes)
    self.nv1_conv5, self.nv1_bn5, self.nv1_relu5, self.nv1_drop5 = self.AddConvBlock(
        64, 3, 2, dropout_prob, self.nv1_layers, self.nv1_shapes)
    self.nv1_flatten = self.AddLayer(
        MakeFlatten(self.nv1_shapes[-1]),
        self.nv1_layers, self.nv1_shapes)
    self.nv1_fc1, self.nv1_fc1_bn, self.nv1_fc1_relu = self.AddFcBlock(
        100, self.nv1_layers, self.nv1_shapes)
    self.nv1_fc1_drop = self.AddLayer(
        MakeDropout(self.nv1_shapes[-1], dropout_prob),
        self.nv1_layers, self.nv1_shapes)
    self.nv1_fc2, self.nv1_fc2_bn, self.nv1_fc2_relu = self.AddFcBlock(
        50, self.nv1_layers, self.nv1_shapes)
    self.nv1_fc3 = self.AddLayer(
        MakeLinear(self.nv1_shapes[-1], 10),
        self.nv1_layers, self.nv1_shapes)
    
    self.nv2_conv2, self.nv2_bn2, self.nv2_relu2, self.nv2_drop2 = self.AddConvBlock(
        36, 5, 2, dropout_prob, self.nv2_layers, self.nv2_shapes)
    self.nv2_conv3, self.nv2_bn3, self.nv2_relu3, self.nv2_drop3 = self.AddConvBlock(
        48, 5, 2, dropout_prob, self.nv2_layers, self.nv2_shapes)
    self.nv2_conv4, self.nv2_bn4, self.nv2_relu4, self.nv2_drop4 = self.AddConvBlock(
        64, 3, 2, dropout_prob, self.nv2_layers, self.nv2_shapes)
    self.nv2_conv5, self.nv2_bn5, self.nv2_relu5, self.nv2_drop5 = self.AddConvBlock(
        64, 3, 2, dropout_prob, self.nv2_layers, self.nv2_shapes)
    self.nv2_flatten = self.AddLayer(
        MakeFlatten(self.nv2_shapes[-1]), self.nv2_layers, self.nv2_shapes)
    self.nv2_fc1, self.nv2_fc1_bn, self.nv2_fc1_relu = self.AddFcBlock(
        100, self.nv2_layers, self.nv2_shapes)
    self.nv2_fc1_drop = self.AddLayer(
        MakeDropout(self.nv2_shapes[-1], dropout_prob),
        self.nv2_layers, self.nv2_shapes)
    self.nv2_fc2, self.nv2_fc2_bn, self.nv2_fc2_relu = self.AddFcBlock(
        50, self.nv2_layers, self.nv2_shapes)
    self.nv2_fc3 = self.AddLayer(
        MakeLinear(self.nv2_shapes[-1], 10),
        self.nv2_layers, self.nv2_shapes)
    
    self.merged_shape = [
        self.comma_shapes[-1][0] + 
        self.nv1_shapes[-1][0] + 
        self.nv2_shapes[-1][0]]
    self.merged_linear = nn.Linear(self.merged_shape[0], 1)
  
  def forward(self, x):
    comma_result = x
    for layer in self.comma_layers:
      comma_result = layer(comma_result)
    
    nv1_result = x
    for layer in self.nv1_layers:
      nv1_result = layer(nv1_result)

    nv2_result = x
    for layer in self.nv2_layers:
      nv2_result = layer(nv2_result)
    
    merged_result = torch.cat([comma_result, nv1_result, nv2_result], 1)
    result = self.merged_linear(merged_result)
    return result

  def AddConvBlock(self, out_channels, kernel, stride, dropout_prob, layers, shapes):
    conv = self.AddLayer(
        MakeConv2d(shapes[-1], out_channels, kernel, stride), layers, shapes)
    bn = self.AddLayer(MakeBatchNorm2d(shapes[-1]), layers, shapes)
    relu = self.AddLayer(MakeRelu(shapes[-1]), layers, shapes)
    dropout = self.AddLayer(MakeDropout2d(shapes[-1], dropout_prob), layers, shapes)
    return conv, bn, relu, dropout

  def AddFcBlock(self, out_dim, layers, shapes):
    fc = self.AddLayer(MakeLinear(shapes[-1], out_dim), layers, shapes)
    bn = self.AddLayer(MakeBatchNorm1d(shapes[-1]), layers, shapes)
    relu = self.AddLayer(MakeRelu(shapes[-1]), layers, shapes)
    return fc, bn, relu

  def AddLayer(self, layer_tuple, layers_list, shapes_list):
    layer, out_shape = layer_tuple
    layers_list.append(layer)
    shapes_list.append(out_shape)
    return layer

NVIDIA_NET_NAME = 'nvidia'
RAMBO_NET_NAME = 'rambo'

IN_SHAPE = 'in_shape'
DROPOUT_PROB = 'dropout_prob'

def MakeNetwork(net_name, **kwargs):
  if net_name == NVIDIA_NET_NAME:
    return NvidiaSingleFrameNet(kwargs[IN_SHAPE], kwargs[DROPOUT_PROB])
  elif net_name == RAMBO_NET_NAME:
    return UdacityRamboNet(kwargs[IN_SHAPE], kwargs[DROPOUT_PROB])
  else:
    assert False, ('Unknown network name: %s' % (net_name,))