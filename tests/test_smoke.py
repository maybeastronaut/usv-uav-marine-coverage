import unittest

from usv_uav_2_0 import __doc__


class SmokeTestCase(unittest.TestCase):
    def test_package_imports(self) -> None:
        self.assertIsNotNone(__doc__)
