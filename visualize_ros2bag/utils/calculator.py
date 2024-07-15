from math import dist
import numpy as np

def point_dist(x1, y1, x2, y2):
  return dist([x1, y1], [x2, y2])

def check_numerical_evaluation(error:list):
  '''
  @error: [value1, value2, ...]
  @return mean, variance
  '''
  # Calculate sample mean (average)
  mean_X = np.mean(error)
  print("Sample mean:", mean_X)

  # Calculate sample variance (covariance with itself)
  variance_X = np.var(error, ddof=1)  # ddof=1 for sample variance (unbiased estimator)
  print("Sample variance (covariance with itself):", variance_X)
  return mean_X, variance_X