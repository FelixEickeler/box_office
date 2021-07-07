import pathlib
import sys
import unittest


def run_test():
    if "--tests" in sys.argv:
        sys.argv.remove("--tests")
    loader = unittest.TestLoader()
    # this_file = pathlib.Path(__file__).parent.absolute()
    tests = loader.discover(".")
    testRunner = unittest.runner.TextTestRunner(verbosity=2)
    testRunner.run(tests)
    # runner = unittest.TextTestRunner()
    # runner.run(suite())

