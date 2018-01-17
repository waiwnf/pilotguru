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
DROPOUT_PROB = 'dropout_prob'

BATCHNORM = 'batchnorm'

FORWARD_AXIS = 'forward_axis'
FRAME_IMG = 'frame_img'
STEERING = 'steering'

NET_NAME = 'net_name'

NET_HEAD_DIMS = 'net_head_dims'
LABEL_DIMENSIONS = 'label_dimensions'
LAYER_BLOCKS_OPTIONS = 'layer_blocks_options'

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
  def __init__(self, in_shape, layer_blocks_options):
    super(SequentialNet, self).__init__()
    self.layers = []
    self.out_shapes = [in_shape]
    self.layer_blocks_options = layer_blocks_options
  
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
    if self.layer_blocks_options[CONV][BATCHNORM] == True:
      bn = self.AddBatchNorm2d()
    activation = self.AddActivation(self.layer_blocks_options[CONV][ACTIVATION])
    dropout = None
    if dropout_prob > 0:
      dropout = self.AddDropout(
          dropout_prob, self.layer_blocks_options[CONV][DROPOUT])
    return conv, bn, activation, dropout

  def AdFcBlock(self, out_channels, dropout_prob):
    fc = self.AddLinear(out_channels)
    bn = None
    if self.layer_blocks_options[FC][BATCHNORM] == True:
      bn = self.AddBatchNorm1d()
    activation = self.AddActivation(self.layer_blocks_options[FC][ACTIVATION])
    dropout = None
    if dropout_prob > 0:
      dropout = self.AddDropout(
          dropout_prob, self.layer_blocks_options[FC][DROPOUT])
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

class LinearBias(nn.Module):
  def __init__(self, in_dims, out_dims, in_name):
    super(LinearBias, self).__init__()
    self.w = nn.Linear(in_dims, out_dims)
    self.w.weight.data[...] = 0
    self.in_name = in_name
  
  def InputNames(self):
    return [self.in_name]

  def forward(self, x):
    pre_bias_value = x[0]
    bias_in = x[1]
    return torch.add(pre_bias_value, self.w(bias_in))

class ImageNetWithPostTransforms(SequentialNet):
  """Runs the image through a regular convnet, then applies optional transforms.
  """

  def __init__(self, in_shape, options, post_transform_modules=[]):
    super(ImageNetWithPostTransforms, self).__init__(in_shape, options)
    self.input_names = [FRAME_IMG] + [
      name for m in post_transform_modules for name in m.InputNames()]
    self.post_transform_modules = nn.ModuleList(post_transform_modules)
    self.in_img_idx = self.input_names.index(FRAME_IMG)
    next_idx = self.in_img_idx + 1
    self.in_post_transform_idx = []
    for m in post_transform_modules:
      self.in_post_transform_idx.append(
          [i + next_idx for i in range(len(m.InputNames()))])
      next_idx += len(m.InputNames())
  
  def forward(self, x):
    img = x[self.in_img_idx]
    result = super(ImageNetWithPostTransforms, self).forward([img])[0]
    for module_idx, post_transform_module in enumerate(
          self.post_transform_modules):
      post_transform_in = [x[i] for i in self.in_post_transform_idx[module_idx]]
      result = post_transform_module.forward([result] + post_transform_in)[0]
    return [result]

  def InputNames(self):
    return self.input_names

  def LabelNames(self):
    return [STEERING]


class ToyConvNet(ImageNetWithPostTransforms):
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
    self.AddActivation(self.layer_blocks_options[FC][ACTIVATION])
    self.fc2 = self.AddLinear(84)
    self.AddActivation(self.layer_blocks_options[FC][ACTIVATION])
    self.fc3 = self.AddLinear(1)


class NvidiaSingleFrameNet(ImageNetWithPostTransforms):

  def __init__(
      self,
      in_shape,
      options,
      post_transform_modules=[]):
    super(NvidiaSingleFrameNet, self).__init__(
        in_shape,
        options[LAYER_BLOCKS_OPTIONS],
        post_transform_modules=post_transform_modules)
    dropout_prob = options[DROPOUT_PROB]
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
        options[NET_HEAD_DIMS], 0)

    self.fc_final = self.AddLinear(options[LABEL_DIMENSIONS])


class UdacityRamboNet(nn.Module):
  def __init__(self, in_shape, options):
    super(UdacityRamboNet, self).__init__()

    dropout_prob = options[DROPOUT_PROB]

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
        MakeLinear(self.comma_shapes[-1], options[NET_HEAD_DIMS]),
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
        MakeLinear(self.nv1_shapes[-1], options[NET_HEAD_DIMS]),
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
        MakeLinear(self.nv2_shapes[-1], options[NET_HEAD_DIMS]),
        self.nv2_layers, self.nv2_shapes)
    
    self.merged_shape = [
        self.comma_shapes[-1][0] + 
        self.nv1_shapes[-1][0] + 
        self.nv2_shapes[-1][0]]
    self.merged_linear = nn.Linear(
        self.merged_shape[0], options[LABEL_DIMENSIONS])
  
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


class RamboCommaNet(ImageNetWithPostTransforms):

  def __init__(
      self,
      in_shape,
      options,
      post_transform_modules=[]):
    super(RamboCommaNet, self).__init__(
        in_shape,
        options[LAYER_BLOCKS_OPTIONS],
        post_transform_modules=post_transform_modules)

    dropout_prob = options[DROPOUT_PROB]

    self.conv1, self.c1_norm, self.c1_act, self.c1_drop = self.AddConvBlock(
        16, 8, 4, dropout_prob)

    self.conv2, self.c2_norm, self.c2_act, self.c2_drop = self.AddConvBlock(
        32, 5, 2, dropout_prob)

    self.conv3, self.c3_norm, self.c3_act, self.c3_drop = self.AddConvBlock(
        64, 5, 2, dropout_prob)

    self.AddFlatten()

    self.fc1, self.fc1_norm, self.fc1_act, self.fc1_drop = self.AdFcBlock(
        512, dropout_prob)

    self.fc2 = self.AddLinear(options[NET_HEAD_DIMS])
    self.AddActivation(RELU)

    self.fc_final = self.AddLinear(options[LABEL_DIMENSIONS])


class RamboNVidiaNet(ImageNetWithPostTransforms):

  def __init__(
        self,
        skip_first_conv_layer,
        in_shape,
        options,
        post_transform_modules=[]):
    super(RamboNVidiaNet, self).__init__(
        in_shape,
        options[LAYER_BLOCKS_OPTIONS],
        post_transform_modules=post_transform_modules)

    dropout_prob = options[DROPOUT_PROB]

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

    self.fc3 = self.AddLinear(options[NET_HEAD_DIMS])
    self.AddActivation(RELU)

    self.fc_final = self.AddLinear(options[LABEL_DIMENSIONS])


class DeepNVidiaNet(ImageNetWithPostTransforms):

  def __init__(
      self,
      in_shape,
      options,
      post_transform_modules=[]):
    super(DeepNVidiaNet, self).__init__(
        in_shape,
        options[LAYER_BLOCKS_OPTIONS],
        post_transform_modules=post_transform_modules)
    
    dropout_prob = options[DROPOUT_PROB]
    
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

    self.fc3 = self.AddLinear(options[NET_HEAD_DIMS])
    self.AddActivation(self.layer_blocks_options[FC][ACTIVATION])

    self.fc_final = self.AddLinear(options[LABEL_DIMENSIONS])


NVIDIA_NET_NAME = 'nvidia'
RAMBO_NET_NAME = 'rambo'
RAMBO_COMMA_NET_NAME = 'rambo-comma'
RAMBO_NVIDIA_DEEP_NET_NAME = 'rambo-nvidia-deep'
RAMBO_NVIDIA_SHALLOW_NET_NAME = 'rambo-nvidia-shallow'
DEEP_NVIDIA_NET_NAME = 'nvidia-deep'

def MakeNetwork(in_shape, options):
  forward_bias = LinearBias(3, options[LABEL_DIMENSIONS], FORWARD_AXIS)
  net_name = options[NET_NAME]
  if net_name == NVIDIA_NET_NAME:
    return NvidiaSingleFrameNet(
        in_shape, options, post_transform_modules=[forward_bias])
  elif net_name == RAMBO_NET_NAME:
    return UdacityRamboNet(in_shape, options)
  elif net_name == RAMBO_COMMA_NET_NAME:
    return RamboCommaNet(
        in_shape, options, post_transform_modules=[forward_bias])
  elif net_name == RAMBO_NVIDIA_DEEP_NET_NAME:
    return RamboNVidiaNet(
        False, in_shape, options, post_transform_modules=[forward_bias])
  elif net_name == RAMBO_NVIDIA_SHALLOW_NET_NAME:
    return RamboNVidiaNet(
        True, in_shape, options, post_transform_modules=[forward_bias])
  elif net_name == DEEP_NVIDIA_NET_NAME:
    return DeepNVidiaNet(
        in_shape, options, post_transform_modules=[forward_bias])
  else:
    assert False, ('Unknown network name: %s' % (net_name,))