# Helpers for per-sample weighting during training.

import numpy as np

NAME = 'name'

UNIFORM = 'uniform'
LABEL_L1 = 'label_l1'
EXP_RECENT_LOSS = 'exp_recent_loss'

LABEL_L1_WEIGHT_SCALE = 'label_l1_weight_scale'

RECENT_LOSS_LR = 'recent_loss_lr'
RECENT_LOSS_EXP_SCALE = 'recent_loss_exp_scale'
RAW_WEIGHT_CLIP = 'raw_weight_clip'

class UniformWeighter(object):
  def GetWeights(self, indices):
    assert len(indices.shape) == 1, str(indices.shape)
    return np.ones(indices.shape, dtype = np.float32)

  def RegisterLosses(self, indices, losses):
    pass
  
  def Step(self):
    pass

class LabelL1Weighter(object):
  # Increases the weight proportionally to the label magnitude.

  def __init__(self, extra_weight_scale, labels):
    assert len(labels.shape) == 1
    assert extra_weight_scale >= 0
    self.weights = np.abs(labels) * extra_weight_scale + 1.0
    # Normalize so that the average example weight is 1.0
    total_weight = np.sum(self.weights.astype(np.float64))
    avg_weight = total_weight / labels.size
    self.weights /= avg_weight

  def GetWeights(self, indices):
    assert len(indices.shape) == 1
    return self.weights[indices]

  def RegisterLosses(self, indices, losses):
    pass
  
  def Step(self):
    pass

class ExpRecentLossWeighter(object):
  # Increases the weight proportionally to exp(recent example loss).
  # Very loosely inspired by AdaBoost weighting.
  # Example weights are also clipped to avoid extreme differences in gradient
  # magnitude across different minibatches.

  def __init__(
      self, num_samples, recent_loss_lr, loss_scale, max_raw_weight_clip):
    assert num_samples > 0
    assert recent_loss_lr >= 0
    assert loss_scale >= 0
    assert max_raw_weight_clip >= 1.0
    self.total_losses = np.zeros([num_samples], dtype=np.float64)
    self.lr = recent_loss_lr
    self.loss_scale = loss_scale
    self.max_raw_weight_clip = max_raw_weight_clip
    self.weights = np.ones([num_samples], dtype=np.float32)
    self.Step()

  def GetWeights(self, indices):
    assert len(indices.shape) == 1
    return self.weights[indices]

  def RegisterLosses(self, indices, losses):
    self.total_losses[indices] *= (1.0 - self.lr)
    self.total_losses[indices] += (losses * self.lr)
  
  def Step(self):
    raw_weights = np.exp(self.loss_scale * self.total_losses)
    raw_weights_clipped = np.clip(raw_weights, 1.0, self.max_raw_weight_clip)
    normalization = np.sum(raw_weights_clipped) / np.size(self.total_losses)
    self.weights = (raw_weights_clipped / normalization).astype(np.float32)

def MakeSampleWeighter(options, labels):
  if options[NAME] == UNIFORM:
    return UniformWeighter()
  elif options[NAME] == LABEL_L1:
    return LabelL1Weighter(options[LABEL_L1_WEIGHT_SCALE], labels)
  elif options[NAME] == EXP_RECENT_LOSS:
    return ExpRecentLossWeighter(
        num_samples=labels.shape[0],
        recent_loss_lr=options[RECENT_LOSS_LR],
        loss_scale=options[RECENT_LOSS_EXP_SCALE],
        max_raw_weight_clip=options[RAW_WEIGHT_CLIP])
  else:
    assert False, ('Unknown weighter name: ' + options[NAME])
    return None