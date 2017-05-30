import time

from torch.autograd import Variable

# TODO save learned model to file, checkpoints.
# TODO validation loss.
def TrainModel(net, trainloader, loss, optimizer, epochs):
  for epoch in range(1, epochs + 1):
    running_loss = 0.0
    total_examples = 0
    optimizer.zero_grad()
    
    epoch_start_time = time.time()
    
    for i, data in enumerate(trainloader, 0):
      # get the inputs
      inputs, labels = data
      # TODO parametrize GPU usage.
      inputs_var = Variable(inputs).cuda()
      labels_var = Variable(labels).cuda()

      # forward + backward + optimize
      outputs = net(inputs_var)
      loss_value = loss(outputs, labels_var)
      loss_value.backward()
      optimizer.step()
      optimizer.zero_grad()

      # Accumulate statistics
      examples_in_batch = inputs.size()[0]
      total_examples += examples_in_batch
      running_loss += loss_value.data[0] * examples_in_batch
    
    epoch_end_time = time.time()
    epoch_duration = epoch_end_time - epoch_start_time
    examples_per_sec = total_examples / epoch_duration
    avg_loss = running_loss / total_examples
    
    print('Epoch %d;  loss %g;  %0.2f sec/epoch; %0.2f examples/sec' %
          (epoch, avg_loss, epoch_duration, examples_per_sec))

  print('Finished Training')
