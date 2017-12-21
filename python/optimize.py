import random
import time
from collections import namedtuple

import tensorboard_logger
import torch
import torch.nn

from torch.autograd import Variable

TRAIN_LOSS = 'train_loss'
VAL_LOSS = 'val_loss'
EPOCH_DURATION_SEC = 'epoch_duration_sec'
EXAMPLES_PER_SEC = 'examples_per_sec'

def FlattenInnerChunk(x):
  """Flattens the input tensor to merge first two dimensions into one.

  This helper is useful for unrolling multi-frame training examples into 
  separate single-frame model inputs.
  """
  in_shape = [i for i in x.size()]
  assert len(in_shape) > 2
  dest_shape = [in_shape[0] * in_shape[1]] + in_shape[2:]
  return x.resize_(*dest_shape)

def IdentityTransform(x):
  return x

LossSettings = namedtuple(
    'LossSettings', ['loss', 'data_chunk_transform', 'label_chunk_transform'])
LossSettings.__new__.__defaults__ = (
    None, FlattenInnerChunk, FlattenInnerChunk)

TrainSettings = namedtuple(
    'TrainSettings', ['loss_settings', 'optimizer', 'epochs'])

class UnweightedLoss(torch.nn.Module):
  """Wraps loss functions that do not support examples weights for TrainModel().
  """

  def __init__(self, base_loss):
    super(UnweightedLoss, self).__init__()
    self.base_loss = base_loss
  
  def forward(self, predicted, labels, weights):
    return self.base_loss.forward(predicted, labels)

class WeightedMSELoss(torch.nn.Module):
  """MSE loss with per-example weights."""

  def __init__(self):
    super(WeightedMSELoss, self).__init__()
  
  def forward(self, predicted, labels, weights):
    diff_squares = torch.pow(torch.add(predicted, torch.neg(labels)), 2.0)
    return torch.mean(torch.mul(diff_squares, weights.expand_as(labels)))

def TrainLogEventToString(event):
  return 'loss %g;  val loss: %g;  %0.2f sec/epoch; %0.2f examples/sec' % (
      event[TRAIN_LOSS],
      event[VAL_LOSS],
      event[EPOCH_DURATION_SEC],
      event[EXAMPLES_PER_SEC])

def AverageLosses(total_losses, total_examples):
  return [
      x / y if y > 0 else float('inf')
      for x, y in zip(total_losses, total_examples)]

def TrainModels(
    nets,
    train_loader,
    val_loader,
    train_settings,
    out_prefix,
    batch_use_prob=1.0,
    print_log=True,
    log_dir=''):
  if log_dir != '':
    tensorboard_logger.configure(log_dir, flush_secs=5)

  train_log = []
  min_validation_losses = [float('inf') for net in nets]
  min_validation_loss = float('inf')
  data_transform = train_settings[0].loss_settings.data_chunk_transform
  label_transform = train_settings[0].loss_settings.label_chunk_transform
  for epoch in range(train_settings[0].epochs):
    running_losses = [0.0 for net in nets]
    train_examples_per_net = [0 for net in nets]
    for net_train_settings in train_settings:
      net_train_settings.optimizer.zero_grad()

    epoch_start_time = time.time()
    for (inputs_raw, labels_raw, inputs_weights) in train_loader:
      inputs_var = Variable(data_transform(inputs_raw)).cuda()
      labels_var = Variable(label_transform(labels_raw)).cuda()
      weights_var = Variable(label_transform(inputs_weights)).cuda()

      for net_idx, net in enumerate(nets):
        if random.uniform(0.0, 1.0) < batch_use_prob:
          # forward + backward + optimize
          outputs = net(inputs_var)
          loss_value = train_settings[net_idx].loss_settings.loss(
              outputs, labels_var, weights_var)
          loss_value.backward()
          train_settings[net_idx].optimizer.step()
          train_settings[net_idx].optimizer.zero_grad()

          # Accumulate statistics
          batch_size = inputs_var.size()[0]
          train_examples_per_net[net_idx] += batch_size
          running_losses[net_idx] += loss_value.data[0] * batch_size
    epoch_end_time = time.time()

    epoch_duration = epoch_end_time - epoch_start_time
    examples_per_sec = sum(train_examples_per_net) / epoch_duration
    avg_losses = AverageLosses(running_losses, train_examples_per_net)
    avg_loss = sum(running_losses) / sum(train_examples_per_net)

    validation_total_losses = [0.0 for net in nets]
    validation_examples = [0 for net in nets]

    for net in nets:
      net.eval()

    for (inputs_raw, labels_raw, validation_weights) in val_loader:
      inputs_var = Variable(data_transform(inputs_raw)).cuda()
      labels_var = Variable(label_transform(labels_raw)).cuda()
      weights_var = Variable(label_transform(validation_weights)).cuda()
    
      for net_idx, net in enumerate(nets):
        outputs = net(inputs_var)
        loss_value = train_settings[net_idx].loss_settings.loss(
            outputs, labels_var, weights_var)

        batch_size = inputs_var.size()[0]
        validation_examples[net_idx] += batch_size
        validation_total_losses[net_idx] += loss_value.data[0] * batch_size
    
    for net in nets:
      net.train()
    
    validation_avg_losses = AverageLosses(
        validation_total_losses, validation_examples)
    validation_avg_loss = (
        sum(validation_total_losses) / sum(validation_examples))

    epoch_metrics = {
      TRAIN_LOSS: avg_loss,
      VAL_LOSS: validation_avg_loss,
      EPOCH_DURATION_SEC: epoch_duration,
      EXAMPLES_PER_SEC: examples_per_sec
    }
    train_log.append(epoch_metrics)

    val_improved_marker = ''
    if validation_avg_loss < min_validation_loss:
      val_improved_marker = ' **'
      min_validation_loss = validation_avg_loss

    for net_idx, net in enumerate(nets):
      if validation_avg_losses[net_idx] < min_validation_losses[net_idx]:
        net.cpu()
        torch.save(
            net.state_dict(), out_prefix + '-' + str(net_idx) + '-best.pth')
        net.cuda()
        min_validation_losses[net_idx] = validation_avg_losses[net_idx]
    
    # Maybe print metrics to screen.
    if print_log:
      print('Epoch %d;  %s%s' %
          (epoch, TrainLogEventToString(epoch_metrics), val_improved_marker))
    # Maybe log metrics to tensorboard.
    if log_dir != '':
      tensorboard_logger.log_value('train_loss', avg_loss, epoch)
      tensorboard_logger.log_value('val_loss', validation_avg_loss, epoch)
  
  for net_id, net in enumerate(nets):
    net.cpu()
    torch.save(net.state_dict(), out_prefix + '-' + str(net_id) + '-last.pth')
    net.cuda()

  return train_log