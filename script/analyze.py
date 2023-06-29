# Johannes Arenander, 2023-05-25
# analyze <device> <reference> <fileout> <testvalue>

import sys
import csv
import math

def main(argv):
  x = sample = process(argv[0], argv[1], argv[2])
  u = population_mean = float(argv[3])
  m = sample_mean(x)
  s = sample_standard_deviation(x)
  t = test_statistic(x, u)
  n = sample_size = len(x)
  df = degrees_of_freedom = n - 1
  print(f'µ (population mean): {u}')
  print(f'm (sample mean): {m}')
  print(f's (sample standard deviation): {s}')
  print(f't (t-score): {t}')
  print(f'df (degrees of freedom): {df}')
  
def sample_mean(sample):
  x = sample
  n = len(x)
  return sum(x) / n

def sample_standard_deviation(sample):
  x = sample
  m = sample_mean(x)
  n = len(x)
  squares_of_deviations = map(lambda x: pow(x - m, 2), x)
  return math.sqrt(sum(squares_of_deviations) / (n - 1))

def test_statistic(sample, population_mean):
  u = population_mean
  x = sample
  n = sample_size = len(x)
  m = sample_mean(x)
  s = sample_standard_deviation(x)
  return (m - u) * math.sqrt(n) / s

def process(file1, file2, out):
  sample = []

  with open(file1, 'r') as sample1, \
    open(file2, 'r') as sample2, \
    open(out, 'w', newline='') as pair:
    sample1_csv = csv.reader(sample1)
    sample2_csv = csv.reader(sample2)
    pair_csv = csv.writer(pair)
    header1 = next(sample1_csv)
    header2 = next(sample2_csv)
    pair_csv.writerow([
      header1[0],
      f'device [m]',
      f'reference [m]',
      f'error [m]'])
    sample1_row = next(sample1_csv, False)
    sample2_row = next(sample2_csv, False)

    while sample1_row and sample2_row:
      sample1_col = float(sample1_row[0])
      sample2_col = float(sample2_row[0])
      difference = sample1_col - sample2_col

      if difference < 0:
        sample1_row = next(sample1_csv, False)
      elif difference > 0:
        sample2_row = next(sample2_csv, False)
      else:
        sample_difference = math.fabs(
          float(sample1_row[1]) - float(sample2_row[1]))
        sample.append(sample_difference)
        pair_csv.writerow([
          sample1_col,
          sample1_row[1],
          sample2_row[1],
          sample_difference])
        sample1_row = next(sample1_csv, False)
        sample2_row = next(sample2_csv, False)

  return sample

if __name__ == '__main__':
  main(sys.argv[1:])
