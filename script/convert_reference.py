# Johannes Arenander 2023-06-16
# convert_reference<filein> <fileout>
import sys
import csv

def main(argv):
  file1 = argv[0]
  file2 = argv[1]

  with open(file1, 'r') as input, \
  open(file2, 'w', newline = '') as output:
    reader = csv.reader(input)
    writer = csv.writer(output)
    next(reader)
    header = next(reader)
    framerate = float(header[0])
    pixels_per_meter = float(header[1])
    next(reader)
    point = next(reader, False)
    start_frame = int(point[0])
    start_pixel = int(point[1])

    writer.writerow(['time [s]', 'distance [m]'])

    while point:
      frame = int(point[0])
      pixel = int(point[1])
      time = (frame - start_frame) / framerate
      distance = (pixel - start_pixel) / pixels_per_meter

      writer.writerow(
        [f'{round(time, 2)}', f'{round(distance, 2)}'])
      point = next(reader, False)

if __name__ == '__main__':
  main(sys.argv[1:])
