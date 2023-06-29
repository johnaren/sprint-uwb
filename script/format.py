# Generated with Bing by Johannes Arenander 2023-05-25
import os
import sys

def replace_first_line(filename):
    if os.path.isfile(filename):
        with open(filename, 'r') as f:
            lines = f.readlines()
        lines[0] = 'time [s],distance [m]\n'
        with open(filename, 'w') as f:
            f.writelines(lines)
    else:
        print(f'{filename} does not exist.')

if __name__ == '__main__':
    replace_first_line(sys.argv[1])