"""Small stock-patch consumer for native sparse-buffer regression tests.

This adapter has no rendering, server, application or CAD dependency. It retains
an explicit max-upper approximation separately from authoritative stock bounds.
"""

from array import array
import math
import struct

from camlib._buffers import native
from camlib.simulation import PartialStockView

RECORD = struct.Struct("<If")
MAX_DISPLAY_SIDE = 256


class StockGrid:
    """Independent max-upper display approximation, never authoritative stock."""

    def __init__(self, state, max_side=128):
        if type(max_side) is not int or not 16 <= max_side <= MAX_DISPLAY_SIDE:
            raise ValueError("display side must be an integer in [16,256]")
        self.nx, self.ny = min(state.nx, max_side), min(state.ny, max_side)
        self.heights = array("f", [0]) * (self.nx * self.ny)
        self._token = None
        self.info = dict(
            source_grid=[state.nx, state.ny],
            grid=[self.nx, self.ny],
            min_mm=state.min_mm,
            max_mm=state.max_mm,
            source_cell_mm=[state.dx, state.dy],
            policy="max_upper",
            max_block_mm=[math.ceil(state.nx / self.nx) * state.dx, math.ceil(state.ny / self.ny) * state.dy],
            z_rounding_bound_mm=max(abs(state.min_mm[2]), abs(state.max_mm[2])) * 2**-23 + 2**-149,
        )

    def update(self, player):
        state = player.state
        full = self._token is None or player.full_refresh
        if self._token is state._token:
            return b"", False
        if not full and player.dirty_rect is None:
            self._token = state._token
            return b"", False
        if isinstance(state, PartialStockView):
            records = state.display_patch((self.nx, self.ny), None if full else player.dirty_rect)
        else:
            records = native.display_patch(
                state.heights,
                state.nx,
                (self.nx, self.ny),
                None if full else player.dirty_rect,
                (state.min_mm[2], state.max_mm[2]),
            )
        for index, height in RECORD.iter_unpack(records):
            self.heights[index] = height
        self._token = state._token
        return records, full

    def snapshot(self):
        return b"".join(RECORD.pack(i, h) for i, h in enumerate(self.heights))
