#!/usr/bin/python3

import os
import re

path = "../"
pattern = "TODO(sensors):"
foundList = []

for r,d,f in os.walk(path):
  for files in f:
    try:
      for n,line in enumerate(open(os.path.join(r,files))):
        if (pattern in line) and ("checker.py" not in files):
          comment = line
          comment = comment.replace(pattern, '')
          comment = comment.replace('//', '')
          comment = comment.strip()
          foundList.append("In file %s at line %d: %s" %(files, n+1, comment))
    except (UnicodeDecodeError):
      pass
if len(foundList) != 0:
  print ("Found " + str(len(foundList)) + " arbitrary values that need to be found manually.\n")
  print("Fix these values before running this code on a robot:")
  for str in foundList:
    print (str)
