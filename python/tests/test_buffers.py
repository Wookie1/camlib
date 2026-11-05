from array import array
import math
import struct
import unittest

from camlib._buffers import native


def packed(second_before=10):
    return struct.pack("<IddddIdddd", 0, 10, 10, 6, 6, 1, second_before, 10, 7, 7)


class BufferTest(unittest.TestCase):
    def test_native_delta_validates_late_cells_before_any_mutation(self):
        heights = array("d", [10, 10] * 3)
        before = heights.tobytes()
        with self.assertRaisesRegex(ValueError, "old state"):
            native.apply_delta(heights, packed(9), 0, 2, 3, (0, 10))
        self.assertEqual(heights.tobytes(), before)
        rect = native.apply_delta(heights, b"prefix" + packed(), 6, 2, 3, (0, 10))
        self.assertEqual(rect, (0, 0, 1, 0))
        self.assertEqual(list(heights), [6, 6, 7, 7, 10, 10])
        for offset, count, nx in [(1, 2, 3), (0, 4, 3), (2**64 - 1, 2, 3), (0, 2, 2)]:
            with self.assertRaises((ValueError, OverflowError)):
                native.apply_delta(heights, packed(), offset, count, nx, (0, 10))

    def test_python_buffer_shapes_types_and_readonly_leases_are_checked(self):
        heights = array("d", [10, 10] * 3)
        native.validate_heights(memoryview(heights).toreadonly(), 0, 10)
        with self.assertRaisesRegex(ValueError, "writable"):
            native.apply_delta(memoryview(heights).toreadonly(), packed(), 0, 2, 3, (0, 10))
        for wrong in [memoryview(heights)[::2], memoryview(heights)[::-1], array("f", [10, 10])]:
            with self.assertRaises((ValueError, TypeError, BufferError)):
                native.validate_heights(wrong, 0, 10)
        matrix = memoryview(heights).cast("B").cast("d", shape=(3, 2))
        with self.assertRaisesRegex(ValueError, "one-dimensional"):
            native.validate_heights(matrix, 0, 10)
        self.assertEqual(list(heights), [10] * 6)

    def test_native_bounds_and_volume_match_independent_columns(self):
        heights = array("d", [0, 0, 6, 7, 10, 10])
        self.assertEqual(native.volume(heights, 0, 10, 1), (16, 17, 13, 14))
        for bad in [array("d", [math.nan, 10]), array("d", [8, 7]), array("d", [-1, 10]), array("d", [0, 11])]:
            with self.assertRaisesRegex(ValueError, "height bounds"):
                native.validate_heights(bad, 0, 10)


if __name__ == "__main__":
    unittest.main()
