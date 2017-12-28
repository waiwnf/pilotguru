import math

import torch
import torch.nn as nn

CONV = 'conv'
FC = 'fc'

ACTIVATION = 'activation'
RELU = 'relu'
SELU = 'selu'

DROPOUT = 'dropout'
DROPOUT_VANILLA = 'vanilla'
DROPOUT_2D = '2d'
DROPOUT_ALPHA = 'alpha'

BATCHNORM = 'batchnorm'

FORWARD_AXIS = 'forward_axis'
FRAME_IMG = 'frame_img'
STEERING = 'steering'

FORWARD_AXIS_DIMS = 3

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

def MakeActivation(in_shape, activation_type):
  out_shape = in_shape
  layer = None
  if activation_type == RELU:
    layer = lambda x : nn.functional.relu(x)
  elif activation_type == SELU:
    layer = lambda x : nn.functional.selu(x)
  else:
    assert False, 'Unknown activation type: %s' % (activation_type,)
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

def MakeDropout(in_shape, p, dropout_type):
  if dropout_type == DROPOUT_VANILLA:
    return nn.Dropout(p), in_shape
  elif dropout_type == DROPOUT_2D:
    return nn.Dropout2d(p), in_shape
  elif dropout_type == DROPOUT_ALPHA:
    return nn.AlphaDropout(p), in_shape
  else:
    assert False, 'Unknown dropout type: %s' % (dropout_type,)

class SequentialNet(nn.Module):
  def __init__(self, in_shape, options):
    super(SequentialNet, self).__init__()
    self.layers = []
    self.out_shapes = [in_shape]
    self.options = options
  
  def OutShape(self):
    return self.out_shapes[-1]
  
  def AddConv2d(self, out_channels, kernel_size, stride=1):
    return self.AddLayer(
        MakeConv2d(self.OutShape(), out_channels, kernel_size, stride))

  def AddMaxPool2d(self, kernel_size):
    return self.AddLayer(MakeMaxPool2d(self.OutShape(), kernel_size))

  def AddActivation(self, activation_type):
    return self.AddLayer(MakeActivation(self.OutShape(), activation_type))

  def AddLinear(self, out_size):
    return self.AddLayer(MakeLinear(self.OutShape(), out_size))
  
  def AddFlatten(self):
    return self.AddLayer(MakeFlatten(self.OutShape()))
  
  def AddBatchNorm2d(self):
    return self.AddLayer(MakeBatchNorm2d(self.OutShape()))
  
  def AddBatchNorm1d(self):
    return self.AddLayer(MakeBatchNorm1d(self.OutShape()))

  def AddDropout(self, p, dropout_type):
    return self.AddLayer(MakeDropout(self.OutShape(), p, dropout_type))

  def AddConvBlock(self, out_channels, kernel_size, stride, dropout_prob):
    conv = self.AddConv2d(out_channels, kernel_size, stride)
    bn = None
    if self.options[CONV][BATCHNORM] == True:
      bn = self.AddBatchNorm2d()
    activation = self.AddActivation(self.options[CONV][ACTIVATION])
    dropout = None
    if dropout_prob > 0:
      dropout = self.AddDropout(dropout_prob, self.options[CONV][DROPOUT])
    return conv, bn, activation, dropout

  def AdFcBlock(self, out_channels, dropout_prob):
    fc = self.AddLinear(out_channels)
    bn = None
    if self.options[FC][BATCHNORM] == True:
      bn = self.AddBatchNorm1d()
    activation = self.AddActivation(self.options[FC][ACTIVATION])
    dropout = None
    if dropout_prob > 0:
      dropout = self.AddDropout(dropout_prob, self.options[FC][DROPOUT])
    return fc, bn, activation, dropout

  def AddLayer(self, layer_tuple):
    layer, out_shape = layer_tuple
    self.layers.append(layer)
    self.out_shapes.append(out_shape)
    return layer
  
  def forward(self, x):
    assert len(x) == 1
    result = x[0]
    for layer in self.layers:
      result = layer(result)
    return [result]

class SequentialNetNamedData(SequentialNet):
  def __init__(self, in_shape, options, input_names, label_names):
    super(SequentialNetNamedData, self).__init__(in_shape, options)
    self.input_names = input_names
    self.label_names = label_names
  
  def InputNames(self):
    return self.input_names

  def LabelNames(self):
    return self.label_names

class ImageWithAxisNet(SequentialNetNamedData):
  """Descendants MUST define self.fc_final."""

  def __init__(self, in_shape, options):
    super(ImageWithAxisNet, self).__init__(
      in_shape, options, [FRAME_IMG, FORWARD_AXIS], [STEERING])
    self.in_img_idx = self.InputNames().index(FRAME_IMG)
    self.in_forward_axis_idx = self.InputNames().index(FORWARD_AXIS)
  
  def forward(self, x):
    img = x[self.in_img_idx]
    image_features_list = super(ImageWithAxisNet, self).forward([img])
    axis = x[self.in_forward_axis_idx]
    return [self.fc_final(torch.cat([image_features_list[0], axis], dim=1))]  

class ToyConvNet(SequentialNetNamedData):
  """Simple 3-conv + 3-fc model, mostly for debugging purposes."""

  def __init__(self, in_shape, options):
    super(ToyConvNet, self).__init__(in_shape, options, [FRAME_IMG], [STEERING])

    self.conv1, self.c1_norm, self.c1_act, self.c1_drop = self.AddConvBlock(
        6, 5, 1, 0)
    self.pool1 = self.AddMaxPool2d(2)

    self.conv2, self.c2_norm, self.c2_act, self.c2_drop = self.AddConvBlock(
        16, 5, 1, 0)
    self.pool2 = self.AddMaxPool2d(2)

    self.conv3, self.c3_norm, self.c3_act, self.c3_drop = self.AddConvBlock(
        1, 5, 1, 0)
    self.pool3 = self.AddMaxPool2d(2)

    self.AddFlatten()

    self.fc1 = self.AddLinear(120)
    self.AddActivation(self.options[FC][ACTIVATION])
    self.fc2 = self.AddLinear(84)
    self.AddActivation(self.options[FC][ACTIVATION])
    self.fc3 = self.AddLinear(1)


class NvidiaSingleFrameNet(ImageWithAxisNet):

  def __init__(self, in_shape, out_dims, dropout_prob, options, head_dims=10):
    super(NvidiaSingleFrameNet, self).__init__(in_shape, options)
    self.conv1, self.c1_norm, self.c1_act, self.c1_drop = self.AddConvBlock(
        24, 5, 2, dropout_prob)
    self.conv2, self.c2_norm, self.c2_act, self.c2_drop = self.AddConvBlock(
        36, 5, 2, dropout_prob)
    self.conv3, self.c3_norm, self.c3_act, self.c3_drop = self.AddConvBlock(
        48, 5, 2, dropout_prob)
    self.conv4, self.c4_norm, self.c4_act, self.c4_drop = self.AddConvBlock(
        64, 3, 1, dropout_prob)
    self.conv5, self.c5_norm, self.c5_act, self.c5_drop = self.AddConvBlock(
        64, 3, 1, dropout_prob)

    self.AddFlatten()

    self.fc1, self.fc1_norm, self.fc1_act, self.fc1_drop = self.AdFcBlock(
        1164, dropout_prob)
    self.fc2, self.fc2_norm, self.fc2_act, self.fc2_drop = self.AdFcBlock(
        100, dropout_prob)
    self.fc3, self.fc3_norm, self.fc13_act, self.fc3_drop = self.AdFcBlock(
        50, 0)
    self.fc4, self.fc4_norm, self.fc14_act, self.fc4_drop = self.AdFcBlock(
        head_dims, 0)

    self.fc_final = nn.Linear(head_dims + FORWARD_AXIS_DIMS, out_dims)


class UdacityRamboNet(nn.Module):
  def __init__(self, in_shape, out_dims, dropout_prob, head_dims=10):
    super(UdacityRamboNet, self).__init__()

    self.comma_layers = []
    self.comma_shapes = [in_shape]
    self.nv1_layers = []
    self.nv1_shapes = [in_shape]
    self.nv2_layers = []
    self.nv2_shapes = [in_shape]

    self.comma_conv1, self.comma_bn1, self.comma_relu1, self.comma_drop1 = (
        self.AddConvBlock(
            16, 8, 4, dropout_prob, self.comma_layers, self.comma_shapes))
    self.comma_conv2, self.comma_bn2, self.comma_relu2, self.comma_drop2 = (
        self.AddConvBlock(
            32, 5, 2, dropout_prob, self.comma_layers, self.comma_shapes))
    self.comma_conv3, self.comma_bn3, self.comma_relu3, self.comma_drop3 = (
        self.AddConvBlock(
            64, 5, 2, dropout_prob, self.comma_layers, self.comma_shapes))
    self.comma_flatten = self.AddLayer(
        MakeFlatten(self.comma_shapes[-1]),
        self.comma_layers, self.comma_shapes)
    self.comma_fc1, self.comma_fc1_bn, self.comma_fc1_relu = self.AddFcBlock(
        512, self.comma_layers, self.comma_shapes)
    self.comma_fc1_drop = self.AddLayer(
        MakeDropout(self.comma_shapes[-1], dropout_prob),
        self.comma_layers, self.comma_shapes)
    self.comma_fc2 = self.AddLayer(
        MakeLinear(self.comma_shapes[-1], head_dims),
        self.comma_layers, self.comma_shapes)
    
    self.nv1_conv1, self.nv1_bn1, self.nv1_relu1, self.nv1_drop1 = (
        self.AddConvBlock(
            24, 5, 2, dropout_prob, self.nv1_layers, self.nv1_shapes))
    self.nv1_conv2, self.nv1_bn2, self.nv1_relu2, self.nv1_drop2 = (
        self.AddConvBlock(
            36, 5, 2, dropout_prob, self.nv1_layers, self.nv1_shapes))
    self.nv1_conv3, self.nv1_bn3, self.nv1_relu3, self.nv1_drop3 = (
        self.AddConvBlock(
            48, 5, 2, dropout_prob, self.nv1_layers, self.nv1_shapes))
    self.nv1_conv4, self.nv1_bn4, self.nv1_relu4, self.nv1_drop4 = (
        self.AddConvBlock(
            64, 3, 2, dropout_prob, self.nv1_layers, self.nv1_shapes))
    self.nv1_conv5, self.nv1_bn5, self.nv1_relu5, self.nv1_drop5 = (
        self.AddConvBlock(
            64, 3, 2, dropout_prob, self.nv1_layers, self.nv1_shapes))
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
        MakeLinear(self.nv1_shapes[-1], head_dims),
        self.nv1_layers, self.nv1_shapes)
    
    self.nv2_conv2, self.nv2_bn2, self.nv2_relu2, self.nv2_drop2 = (
        self.AddConvBlock(
            36, 5, 2, dropout_prob, self.nv2_layers, self.nv2_shapes))
    self.nv2_conv3, self.nv2_bn3, self.nv2_relu3, self.nv2_drop3 = (
        self.AddConvBlock(
            48, 5, 2, dropout_prob, self.nv2_layers, self.nv2_shapes))
    self.nv2_conv4, self.nv2_bn4, self.nv2_relu4, self.nv2_drop4 = (
        self.AddConvBlock(
            64, 3, 2, dropout_prob, self.nv2_layers, self.nv2_shapes))
    self.nv2_conv5, self.nv2_bn5, self.nv2_relu5, self.nv2_drop5 = (
        self.AddConvBlock(
            64, 3, 2, dropout_prob, self.nv2_layers, self.nv2_shapes))
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
        MakeLinear(self.nv2_shapes[-1], head_dims),
        self.nv2_layers, self.nv2_shapes)
    
    self.merged_shape = [
        self.comma_shapes[-1][0] + 
        self.nv1_shapes[-1][0] + 
        self.nv2_shapes[-1][0]]
    self.merged_linear = nn.Linear(self.merged_shape[0], out_dims)
  
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

  def AddConvBlock(
        self, out_channels, kernel, stride, dropout_prob, layers, shapes):
    conv = self.AddLayer(
        MakeConv2d(shapes[-1], out_channels, kernel, stride), layers, shapes)
    bn = self.AddLayer(MakeBatchNorm2d(shapes[-1]), layers, shapes)
    relu = self.AddLayer(MakeRelu(shapes[-1]), layers, shapes)
    dropout = self.AddLayer(
        MakeDropout(shapes[-1], dropout_prob, DROPOUT_2D), layers, shapes)
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

  def input_names(self):
    return [FRAME_IMG]

  def label_names(self):
    return [STEERING]


class RamboCommaNet(ImageWithAxisNet):

  def __init__(self, in_shape, out_dims, dropout_prob, options, head_dims=10):
    super(RamboCommaNet, self).__init__(in_shape, options)

    self.conv1, self.c1_norm, self.c1_act, self.c1_drop = self.AddConvBlock(
        16, 8, 4, dropout_prob)

    self.conv2, self.c2_norm, self.c2_act, self.c2_drop = self.AddConvBlock(
        32, 5, 2, dropout_prob)

    self.conv3, self.c3_norm, self.c3_act, self.c3_drop = self.AddConvBlock(
        64, 5, 2, dropout_prob)

    self.AddFlatten()

    self.fc1, self.fc1_norm, self.fc1_act, self.fc1_drop = self.AdFcBlock(
        512, dropout_prob)

    self.fc2 = self.AddLinear(head_dims)
    self.AddActivation(RELU)

    self.fc_final = nn.Linear(head_dims + FORWARD_AXIS_DIMS, out_dims)


class RamboNVidiaNet(ImageWithAxisNet):

  def __init__(
        self,
        skip_first_conv_layer,
        in_shape,
        out_dims,
        dropout_prob,
        options,
        head_dims=10):
    super(RamboNVidiaNet, self).__init__(in_shape, options)

    if not skip_first_conv_layer:
      self.conv1, self.c1_norm, self.c1_act, self.c1_drop = self.AddConvBlock(
          24, 5, 2, dropout_prob)

    self.conv2, self.c2_norm, self.c2_act, self.c2_drop = self.AddConvBlock(
        36, 5, 2, dropout_prob)

    self.conv3, self.c3_norm, self.c3_act, self.c3_drop = self.AddConvBlock(
        48, 5, 2, dropout_prob)

    self.conv4, self.c4_norm, self.c4_act, self.c4_drop = self.AddConvBlock(
        64, 3, 2, dropout_prob)

    self.conv5, self.c5_norm, self.c5_act, self.c5_drop = self.AddConvBlock(
        64, 3, 2, dropout_prob)

    self.AddFlatten()

    self.fc1, self.fc1_norm, self.fc1_act, self.fc1_drop = self.AdFcBlock(
        100, dropout_prob)
    self.fc2, self.fc2_norm, self.fc2_act, self.fc2_drop = self.AdFcBlock(
        1164, 0)

    self.fc3 = self.AddLinear(head_dims)
    self.AddActivation(RELU)

    self.fc_final = nn.Linear(head_dims + FORWARD_AXIS_DIMS, out_dims)


class DeepNVidiaNet(ImageWithAxisNet):

  def __init__(self, in_shape, out_dims, dropout_prob, options, head_dims=10):
    super(DeepNVidiaNet, self).__init__(in_shape, options)

    self.conv1, self.c1_norm, self.c1_act, self.c1_drop = self.AddConvBlock(
        36, 5, 2, dropout_prob)
    self.conv2, self.c2_norm, self.c2_act, self.c2_drop = self.AddConvBlock(
        48, 5, 2, dropout_prob)
    self.conv3, self.c3_norm, self.c3_act, self.c3_drop = self.AddConvBlock(
        48, 5, 1, dropout_prob)
    self.conv4, self.c4_norm, self.c4_act, self.c4_drop = self.AddConvBlock(
        64, 3, 1, dropout_prob)
    self.conv5, self.c5_norm, self.c5_act, self.c5_drop = self.AddConvBlock(
        64, 3, 2, dropout_prob)
    self.conv6, self.c6_norm, self.c6_act, self.c6_drop = self.AddConvBlock(
        64, 3, 1, dropout_prob)
    self.conv7, self.c7_norm, self.c7_act, self.c7_drop = self.AddConvBlock(
        64, 3, 1, dropout_prob)
    self.conv8, self.c8_norm, self.c8_act, self.c8_drop = self.AddConvBlock(
        64, 3, 1, dropout_prob)

    self.AddFlatten()

    self.fc1, self.fc1_norm, self.fc1_act, self.fc1_drop = self.AdFcBlock(
        100, dropout_prob)
    self.fc2, self.fc2_norm, self.fc2_act, self.fc2_drop = self.AdFcBlock(
        1164, dropout_prob)

    self.fc3 = self.AddLinear(head_dims)
    self.AddActivation(self.options[FC][ACTIVATION])

    self.fc_final = nn.Linear(head_dims + FORWARD_AXIS_DIMS, out_dims)


NVIDIA_NET_NAME = 'nvidia'
RAMBO_NET_NAME = 'rambo'
RAMBO_COMMA_NET_NAME = 'rambo-comma'
RAMBO_NVIDIA_DEEP_NET_NAME = 'rambo-nvidia-deep'
RAMBO_NVIDIA_SHALLOW_NET_NAME = 'rambo-nvidia-shallow'
DEEP_NVIDIA_NET_NAME = 'nvidia-deep'

def MakeNetwork(
  net_name, in_shape, head_dims, out_dims, dropout_prob, options=None):
  if net_name == NVIDIA_NET_NAME:
    return NvidiaSingleFrameNet(
        in_shape, out_dims, dropout_prob, options, head_dims=head_dims)
  elif net_name == RAMBO_NET_NAME:
    return UdacityRamboNet(
        in_shape, out_dims, dropout_prob, head_dims=head_dims)
  elif net_name == RAMBO_COMMA_NET_NAME:
    return RamboCommaNet(
        in_shape, out_dims, dropout_prob, options, head_dims=head_dims)
  elif net_name == RAMBO_NVIDIA_DEEP_NET_NAME:
    return RamboNVidiaNet(
        False, in_shape, out_dims, dropout_prob, options, head_dims=head_dims)
  elif net_name == RAMBO_NVIDIA_SHALLOW_NET_NAME:
    return RamboNVidiaNet(
        True, in_shape, out_dims, dropout_prob, options, head_dims=head_dims)
  elif net_name == DEEP_NVIDIA_NET_NAME:
    return DeepNVidiaNet(
        in_shape, out_dims, dropout_prob, options, head_dims=head_dims)
  else:
    assert False, ('Unknown network name: %s' % (net_name,))