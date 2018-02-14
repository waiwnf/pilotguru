import threading

class SynchronizedTimestampedValue(object):
  def __init__(self):
    self.timestamp = None
    self.value = None
    self.lock = threading.Lock()
    self.lock_condition = threading.Condition(self.lock)

  # TODO update with built-in timestamping.

  def Update(self, timestamp, value):
    assert timestamp is not None
    with self.lock_condition:
      self.timestamp = timestamp
      self.value = value
      self.lock_condition.notify_all()
  
  def WaitGetNext(self, prev_timestamp, timeout=None):
    with self.lock_condition:
      timeout_expired = False
      if self.timestamp is None or self.timestamp == prev_timestamp:
        timeout_expired = (not self.lock_condition.wait(timeout))
      if timeout_expired:
        return None
      else:
        return (self.timestamp, self.value.copy())
