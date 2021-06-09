import pathlib
import sys
import unittest


def run_test():
    if "--test" in sys.argv:
        sys.argv.remove("--test")
    loader = unittest.TestLoader()
    # this_file = pathlib.Path(__file__).parent.absolute()
    tests = loader.discover(".")
    testRunner = unittest.runner.TextTestRunner(verbosity=2)
    testRunner.run(tests)
    # runner = unittest.TextTestRunner()
    # runner.run(suite())

