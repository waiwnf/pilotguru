import time

import torch

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
    for (inputs_raw, labels_raw) in trainloader:
      inputs_var = Variable(input_batch_transform(inputs_raw)).cuda()
      labels_var = Variable(labels_batch_transform(labels_raw)).cuda()

      # forward + backward + optimize
      outputs = net(inputs_var)
      loss_value = loss(outputs, labels_var)
      loss_value.backward()
      optimizer.step()
      optimizer.zero_grad()

      # Accumulate statistics
      total_examples += inputs_var.size()[0]
      running_loss += loss_value.data[0] * inputs_var.size()[0]    
    epoch_end_time = time.time()

    epoch_duration = epoch_end_time - epoch_start_time
    examples_per_sec = total_examples / epoch_duration
    avg_loss = running_loss / total_examples

    net.eval()
    validation_total_loss = 0.0
    validation_examples = 0
    for (inputs_raw, labels_raw) in validation_loader:
      inputs_var = Variable(input_batch_transform(inputs_raw)).cuda()
      labels_var = Variable(labels_batch_transform(labels_raw)).cuda()
     
      outputs = net(inputs_var)

      validation_examples += inputs_var.size()[0]
      validation_total_loss += loss(outputs, labels_var).data[0] * inputs_var.size()[0]
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
