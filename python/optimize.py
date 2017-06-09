import time

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
    trainloader,
    validation_loader,
    loss,
    optimizer,
    epochs,
    out_prefix, 
    input_batch_transform=IdentityTransform,
    labels_batch_transform=IdentityTransform):
  min_validation_loss = float('inf')
  for epoch in range(1, epochs + 1):
    running_loss = 0.0
    total_examples = 0
    optimizer.zero_grad()

    epoch_start_time = time.time()
    for (inputs_raw, labels_raw, inputs_weights) in trainloader:
      inputs_var = Variable(input_batch_transform(inputs_raw)).cuda()
      labels_var = Variable(labels_batch_transform(labels_raw)).cuda()
      weights_var = Variable(labels_batch_transform(inputs_weights)).cuda()

      # forward + backward + optimize
      outputs = net(inputs_var)
      loss_value = loss(outputs, labels_var, weights_var)
      loss_value.backward()
      optimizer.step()
      optimizer.zero_grad()

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
    for (inputs_raw, labels_raw, validation_weights) in validation_loader:
      inputs_var = Variable(input_batch_transform(inputs_raw)).cuda()
      labels_var = Variable(labels_batch_transform(labels_raw)).cuda()
      weights_var = Variable(labels_batch_transform(validation_weights)).cuda()
     
      outputs = net(inputs_var)
      loss_value = loss(outputs, labels_var, weights_var)

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
