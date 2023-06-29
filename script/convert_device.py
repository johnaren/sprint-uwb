# Johannes Arenander 2023-06-16
# convert_device <filein> <fileout>
import sys
import csv

def main(argv):
  file1 = argv[0]
  file2 = argv[1]

  with open(file1, 'r') as input, \
  open(file2, 'w', newline = '') as output:
    reader = csv.reader(input)
    writer = csv.writer(output)
    header = next(reader)
    point = next(reader, False)
    start_distance = float(point[1])
    distance_offset = -3.00

    while point and round(start_distance + distance_offset, 2) < 0.0:
      point = next(reader, False)
      start_distance = float(point[1])

    start_time = float(point[0])
    distance = start_distance
    writer.writerow(['time [s]', 'distance [m]'])

    while point and distance < 11.0:
      time = float(point[0]) - start_time
      distance = float(point[1]) + distance_offset

      writer.writerow(
        [f'{round(time, 2)}', f'{round(distance, 2)}'])
      point = next(reader, False)

if __name__ == '__main__':
  main(sys.argv[1:])
