import time

from torch.autograd import Variable

def WrapTrainExamples(data):
  # get the inputs
  inputs, labels = data
  # TODO parametrize GPU usage.
  inputs_var = Variable(inputs).cuda()
  labels_var = Variable(labels).cuda()
  return inputs_var, labels_var

# TODO save learned model to file, checkpoints.
def TrainModel(net, trainloader, validation_loader, loss, optimizer, epochs):
  for epoch in range(1, epochs + 1):
    running_loss = 0.0
    total_examples = 0
    optimizer.zero_grad()

    epoch_start_time = time.time()
    for i, data in enumerate(trainloader, 0):
      inputs_var, labels_var = WrapTrainExamples(data)

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
    for data in validation_loader:
      inputs_var, labels_var = WrapTrainExamples(data)
      
      outputs = net(inputs_var)

      validation_examples += inputs_var.size()[0]
      validation_total_loss += loss(outputs, labels_var).data[0] * inputs_var.size()[0]
    net.train()
    
    validation_avg_loss = validation_total_loss / validation_examples
    
    print('Epoch %d;  loss %g;  val loss: %g;  %0.2f sec/epoch; %0.2f examples/sec' %
          (epoch, avg_loss, validation_avg_loss, epoch_duration, examples_per_sec))

  print('Finished Training')
