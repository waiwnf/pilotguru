import time
from collections import namedtuple

import torch
import torch.nn

from torch.autograd import Variable

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
    'LossSettings', 'loss data_chunk_transform label_chunk_transform')
LossSettings.__new__.__defaults__ = (
    None, FlattenInnerChunk, FlattenInnerChunk)

TrainSettings = namedtuple('TrainSettings', 'loss_settings optimizer epochs')

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
    return torch.mean(torch.mul(diff_squares, weights))


def TrainModel(
    net,
    train_loader,
    val_loader,
    train_settings,
    out_prefix):
  min_validation_loss = float('inf')
  loss_settings = train_settings.loss_settings
  data_transform = loss_settings.data_chunk_transform
  label_transform = loss_settings.label_chunk_transform
  for epoch in range(1, train_settings.epochs + 1):
    running_loss = 0.0
    total_examples = 0
    train_settings.optimizer.zero_grad()

    epoch_start_time = time.time()
    for (inputs_raw, labels_raw, inputs_weights) in train_loader:
      inputs_var = Variable(data_transform(inputs_raw)).cuda()
      labels_var = Variable(label_transform(labels_raw)).cuda()
      weights_var = Variable(label_transform(inputs_weights)).cuda()

      # forward + backward + optimize
      outputs = net(inputs_var)
      loss_value = loss_settings.loss(outputs, labels_var, weights_var)
      loss_value.backward()
      train_settings.optimizer.step()
      train_settings.optimizer.zero_grad()

      # Accumulate statistics
      batch_size = inputs_var.size()[0]
      total_examples += batch_size
      running_loss += loss_value.data[0] * batch_size
    epoch_end_time = time.time()

    epoch_duration = epoch_end_time - epoch_start_time
    examples_per_sec = total_examples / epoch_duration
    avg_loss = running_loss / total_examples

    net.eval()
    validation_total_loss = 0.0
    validation_examples = 0
    for (inputs_raw, labels_raw, validation_weights) in val_loader:
      inputs_var = Variable(data_transform(inputs_raw)).cuda()
      labels_var = Variable(label_transform(labels_raw)).cuda()
      weights_var = Variable(label_transform(validation_weights)).cuda()
     
      outputs = net(inputs_var)
      loss_value = loss_settings.loss(outputs, labels_var, weights_var)

      batch_size = inputs_var.size()[0]
      validation_examples += batch_size
      validation_total_loss += loss_value.data[0] * batch_size
    net.train()
    
    validation_avg_loss = validation_total_loss / validation_examples

    print('Epoch %d;  loss %g;  val loss: %g;  %0.2f sec/epoch; %0.2f examples/sec' %
          (epoch, avg_loss, validation_avg_loss, epoch_duration, examples_per_sec))

    if validation_avg_loss < min_validation_loss:
      save_start_time = time.time()
      net.cpu()
      torch.save(net.state_dict(), out_prefix + '-best.pth')
      net.cuda()
      min_validation_loss = validation_avg_loss
      save_duration = time.time() - save_start_time
      print('Saved model snapshot. Took %0.2f seconds' % save_duration)
  
  print('Finished Training')
  net.cpu()
  torch.save(net.state_dict(), out_prefix + '-last.pth')
  net.cuda()
