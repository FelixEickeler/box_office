import argparse
from pathlib import Path
import unittest

from tests.tests import run_test


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Strg+F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--eval-only", action="store_true", help="perform evaluation only")
    parser.add_argument("--input", type=Path, default=Path("."), help="Input Pointcloud -  Tests for input format")
    parser.add_argument("--output", type=Path, default=Path("."), help="Output Boxdimensions, and graphs")
    parser.add_argument("--test", action="store_true", help="runs tests")

    args = parser.parse_args()
    if args.test:
        print("Testing on the way")
        run_test()
    else:
        print("Input path: ", args.input)
        print("output path: ", args.output)
        print("start calculating ...")
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
