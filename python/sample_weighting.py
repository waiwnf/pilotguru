# Helpers for per-sample weighting during training.

import numpy as np

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