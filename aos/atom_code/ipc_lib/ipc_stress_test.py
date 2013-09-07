#!/usr/bin/python

# This is a script that runs all of the tests that deal with the IPC stuff in
# parallel for a while and makes sure that they don't ever fail.

import subprocess
import random
import multiprocessing
import time
import os
import Queue
import sys

TESTS = [
  ['queue_test'],
  ['condition_test'],
  # The fairness test doesn't work under load.
  ['mutex_test', '--gtest_filter=-MutexTest.Fairness'],
  #['ipc_queue_test'],
]
TESTS_PATH = '../../../out_atom/Default/tests'
# The tests spend a lot of their time waiting (ie for things to time out), so I
# had to use this many to get the highest CPU utilization.
TESTERS = 35
TEST_TIME = 10

def run(iterations_queue, output_lock, stop_time):
  iterations = 0
  while time.time() < stop_time:
    test = random.choice(TESTS)
    try:
      output = subprocess.check_output(
          ["%s/%s/%s" %(
            os.path.dirname(os.path.abspath(__file__)), TESTS_PATH, test[0])] +
            test[1:],
          stderr=subprocess.STDOUT,
          bufsize=-1)
    except subprocess.CalledProcessError as error:
      with output_lock:
        sys.stderr.write("------Test %s failed with exit %d output:------\n%s" %
            (test, error.returncode, error.output))
    iterations += 1
  iterations_queue.put(iterations)

def main():
  processes = []
  output_lock = multiprocessing.Lock()
  iterations_queue = multiprocessing.Queue()
  stop_time = time.time() + TEST_TIME
  stop_event = multiprocessing.Event()
  for _ in xrange(TESTERS):
    process = multiprocessing.Process(target=run,
      args=(iterations_queue, output_lock, stop_time))
    processes.append(process)
    process.start()
  for process in processes:
    process.join()
  total_iterations = 0
  try:
    while True:
      total_iterations += iterations_queue.get_nowait()
  except Queue.Empty:
    pass
  print("Iterated a total of %d times." % total_iterations)

if __name__ == '__main__':
  main()
