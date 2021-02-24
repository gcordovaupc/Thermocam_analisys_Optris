#!/usr/bin/env python
# -*- coding: utf-8
"""Python ROS module for brick detection in MBZIRC Challenge 2."""

## Python
import sys
import numpy as np
from colorutils import Color


def get_hsv_from_file(hsv_file):

  hsv_list = list()

  with open(hsv_file, "r") as f:
      hex_values = f.read().splitlines()
  for hex_value in hex_values:
      hsv = list(Color(hex=hex_value).hsv)
      hsv[0] /= 2
      hsv[1] *= 255
      hsv[2] *= 255
      hsv_list.append(hsv)
  return hsv_list


def get_hsv_range(hsv_values):

  lower = np.array([180, 255, 255])
  upper = np.array([0, 0, 0])
  num_hsv_values = 0

  for hsv in hsv_values:
    for idx, val in enumerate(hsv):
      if val < lower[idx]:
          lower[idx] = val
      if val > upper[idx]:
          upper[idx] = val
      num_hsv_values = num_hsv_values+1

  print "Calculate out of ", num_hsv_values," HSV values:"
  print "Lower range: ",lower
  print "Upper range: ",upper


if __name__ == '__main__':

  if len(sys.argv) < 2:
    print "Please provide the location of the HSV texfile as argument!"
  elif len(sys.argv) > 2:
    print "Too many arguments given, just provide the location of the HSV textfile!"
  else:
    file_path = sys.argv[1]
    hsv_values = get_hsv_from_file(file_path)
    get_hsv_range(hsv_values)
