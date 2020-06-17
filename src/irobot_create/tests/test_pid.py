from unittest import TestCase
from irobot.control.pid import LowPassController, RampController
import time
import numpy as np


class TestLowPassController(TestCase):
    def test_lowpass_with_larger_T_take_longer_to_rise(self):
        c1 = LowPassController(T=0.01)
        c2 = LowPassController(T=1)

        c1.set_target(1)
        c2.set_target(1)
        time.sleep(0.01)
        self.assertLess(c1.get_current_value(), 1)
        self.assertLess(c2.get_current_value(), 1)
        self.assertGreater(c1.get_current_value(), 0)
        self.assertGreater(c2.get_current_value(), 0)
        self.assertGreater(c1.get_current_value(), c2.get_current_value())

    def test_lowpass_rises_approximately_as_expected(self):
        c = LowPassController(T=0.001)
        c.set_target(10)
        time.sleep(0.01)
        self.assertAlmostEqual(10, c.get_current_value(), 2)

    def test_lowpass_rises_approximately_the_same_regardless_of_number_of_calls(self):
        c1 = LowPassController(T=0.01)
        c1.set_target(10)
        time.sleep(0.05)
        v1 = c1.get_current_value()

        c2 = LowPassController(T=0.01)
        c2.set_target(10)
        for _ in range(5):
            time.sleep(0.01)
            v2 = c2.get_current_value()
        # time.sleep(0.05)


        self.assertAlmostEqual(v1, v2, 1)

    def test_rises_and_falls(self):
        c = LowPassController(T=0.01)
        c.set_target(10)
        self.assertLess(c.get_current_value(), 1)
        time.sleep(0.1)
        self.assertGreater(c.get_current_value(), 9)
        c.set_target(0)
        self.assertGreater(c.get_current_value(), 9)
        time.sleep(0.1)
        self.assertLess(c.get_current_value(), 1)


class TestRampController(TestCase):
    def test_ramp_controller_ramps_up(self):
        c = RampController(max_accel=100)
        c.set_target(10)
        self.assertLess(c.get_current_value(), 0.1)
        time.sleep(0.05)
        self.assertGreater(c.get_current_value(), 5)
        time.sleep(0.05)
        self.assertEqual(c.get_current_value(), 10)
        time.sleep(0.1)
        self.assertEqual(c.get_current_value(), 10)

        c.set_target(-10)
        self.assertGreater(c.get_current_value(), 9)
        self.assertLess(c.get_current_value(), 10)
        time.sleep(0.2)
        self.assertEqual(c.get_current_value(), -10)


