import unittest

from object_list import ObjectList


class TestMVBB(unittest.TestCase):

    def test_mvbb_call(self):
        self.assertFalse()


class TestInputOutput(unittest.TestCase):

    def test_load_pcs_200ents(self):
        self.assertEqual('foo'.upper(), 'FOO')

    # Lods objectlist
    def test_load_ol_23ents(self):
        object_list = ObjectList.from_ol("./tests/data/head_test.ol")
        self.assertEqual(len(object_list), 14)
        object_list = ObjectList.from_ol("./tests/data/tail_test.ol")
        self.assertEqual(len(object_list), 14)
