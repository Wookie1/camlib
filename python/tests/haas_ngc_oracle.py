"""Literal independent interpreter of the emitted Haas2020 subset.

Imports motion data only, no post/dialect/decoder/command map. Geometry/feeds
come from actual NC. Clearance roles require source attribution and are therefore
not inferred from G1 by this oracle. No firmware/hardware qualification claim.
"""

import math
import re
from camlib.motion import Arc, Helix, Line, Motion


class Oracle:
    def __init__(
        self,
        *,
        tools,
        initial_position,
        program_number=1000,
        work_offset=1,
        output_units='mm',
        coordinate_system='fanuc',
        h_t_agreement=False,
        rotation_scaling=False,
        optional_stop_enabled=False,
        default_to_float=False,
        scale_integer_f='default',
    ):
        if coordinate_system not in ('fanuc', 'haas'):
            raise ValueError('oracle: Setting33')
        self.tools = {t.number: t for t in tools}
        self.position = tuple(initial_position)
        if any(not 1 <= t.number <= 200 or not 1 <= t.length_register <= 200 for t in tools):
            raise ValueError('oracle: physical tool/offset table range')
        self.program_number = program_number
        self.work_offset = work_offset
        self.output_units = output_units
        self.h_t_agreement = h_t_agreement
        self.rotation_scaling = rotation_scaling
        if scale_integer_f not in ('default', 'integer', '.1', '.01', '.001', '.0001'):
            raise ValueError('oracle: Setting77')
        self.default_to_float = default_to_float
        self.scale_integer_f = scale_integer_f
        self.optional_stop_enabled = optional_stop_enabled
        self.open = False
        self.number = False
        self.end_command = False
        self.ended = False
        self.flags = set() if rotation_scaling else {'rotation', 'scaling'}
        self.units = None
        self.selected = None
        self.active = None
        self.length = None
        self.offset = None
        self.rpm = 0
        self.coolant = 'off'
        self.count = 0
        self.pending = False

    def read(self, line):
        if (
            self.ended
            or len(line) > 1024
            or not line.isascii()
            or any(ord(c) < 32 and c != '\t' or ord(c) == 127 for c in line)
        ):
            raise ValueError('oracle: physical block/end')
        text = line.split(';')[0].strip().upper()
        text = re.sub(r'\([^()]*\)', ' ', text).strip()
        if '(' in text or ')' in text:
            raise ValueError('oracle: comment')
        if not text:
            return []
        if text == '%':
            if not self.open:
                self.open = True
            elif self.end_command:
                self.ended = True
            else:
                raise ValueError('oracle: percent before M30')
            return []
        if not self.open or self.end_command:
            raise ValueError('oracle: framing')
        if text.startswith('O'):
            if text != 'O' + str(self.program_number) or self.number:
                raise ValueError('oracle: program number')
            self.number = True
            return []
        if not self.number:
            raise ValueError('oracle: no O number')
        text = re.sub(r'^N[0-9]{1,5}\s*', '', text).replace(' ', '').replace('\t', '')
        pieces = re.findall(r'([A-Z])([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))', text)
        if ''.join(w + v for w, v in pieces) != text or len({w for w, v in pieces}) != len(pieces):
            raise ValueError('oracle: unknown/duplicate syntax')
        values = {w: float(v) for w, v in pieces}
        if any(not math.isfinite(v) or abs(v) > 1e9 for v in values.values()):
            raise ValueError('oracle: number')
        # Literal controller notation semantics, independent of the post's strict
        # explicit-decimal subset. This catches an emitter that forgets the dot.
        quantum = 0.001 if self.output_units == 'mm' else 0.0001
        for w, text in pieces:
            if '.' not in text:
                if w in 'XYZIJK' and not self.default_to_float:
                    values[w] *= quantum
                if w == 'F':
                    values[w] *= (
                        quantum
                        if self.scale_integer_f == 'default'
                        else (1 if self.scale_integer_f == 'integer' else float(self.scale_integer_f))
                    )
        precision = 3 if self.output_units == 'mm' else 4
        if any(w in 'XYZIJKF' and len(v.partition('.')[2]) > precision for w, v in pieces):
            raise ValueError('oracle: precision')
        g = values.get('G')
        m = values.get('M')
        if g is not None and m is not None:
            raise ValueError('oracle: mixed commands')
        if g in (20, 21):
            if set(values) != {'G'} or g != (21 if self.output_units == 'mm' else 20):
                raise ValueError('oracle: units')
            self.units = 1.0 if g == 21 else 25.4
            return []
        resets = {
            90: 'absolute',
            17: 'plane',
            94: 'feed',
            40: 'radius',
            49: 'length',
            80: 'cycle',
            61: 'path',
            187: 'smooth',
            50: 'scaling',
            69: 'rotation',
        }
        if g in resets:
            if set(values) != {'G'} or g in (50, 69) and not self.rotation_scaling:
                raise ValueError('oracle: reset/option')
            self.flags.add(resets[g])
            if g == 49:
                self.length = None
            return []
        if g in (52, 100):
            if set(values) != {'G', 'X', 'Y', 'Z'} or any(values[a] != 0 for a in 'XYZ'):
                raise ValueError('oracle: shift/mirror reset')
            self.flags.add('frame' if g == 52 else 'mirror')
            return []
        if g in (54, 55, 56, 57, 58, 59):
            if set(values) != {'G'} or g - 53 != self.work_offset:
                raise ValueError('oracle: work offset')
            self.offset = int(g - 53)
            return []
        if m in (5, 8, 9):
            if set(values) != {'M'}:
                raise ValueError('oracle: M effect words')
            if m == 5:
                self.rpm = 0
                self.flags.add('spindle')
            elif m == 9:
                self.coolant = 'off'
                self.flags.add('coolant')
            else:
                if self.coolant != 'off':
                    raise ValueError('oracle: exclusive coolant')
                self.coolant = 'flood'
            return []
        if m in (3, 4):
            if (
                set(values) != {'M', 'S'}
                or self.active is None
                or self.length is None
                or values['S'] <= 0
                or values['S'] != int(values['S'])
            ):
                raise ValueError('oracle: spindle')
            self.rpm = values['S']
            return []
        if set(values) == {'T'}:
            if values['T'] != int(values['T']) or int(values['T']) not in self.tools:
                raise ValueError('oracle: tool')
            self.selected = int(values['T'])
            return []
        if m in (0, 1, 6):
            if set(values) != {'M'} or self.rpm or self.coolant != 'off':
                raise ValueError('oracle: source must stop spindle/coolant')
            if m == 1 and not self.optional_stop_enabled:
                raise ValueError('oracle: optional stop disabled')
            if m == 6:
                if self.selected is None:
                    raise ValueError('oracle: tool not selected')
                self.active = self.tools[self.selected]
                self.length = None
                self.pending = True
            return []
        if g == 43:
            if (
                set(values) != {'G', 'H'}
                or self.active is None
                or not self.pending
                or values['H'] != self.active.length_register
                or (self.h_t_agreement and values['H'] != self.active.number)
            ):
                raise ValueError('oracle: H register')
            self.length = int(values['H'])
            self.position = tuple(self.active.position_after_mm)
            self.pending = False
            return []
        if m == 30:
            if set(values) != {'M'} or self.rpm or self.coolant != 'off' or self.length is not None:
                raise ValueError('oracle: shutdown')
            self.end_command = True
            return []
        required = {
            'absolute',
            'plane',
            'feed',
            'radius',
            'length',
            'cycle',
            'path',
            'smooth',
            'rotation',
            'scaling',
            'frame',
            'mirror',
            'spindle',
            'coolant',
        }
        if (
            not required <= self.flags
            or self.units is None
            or self.offset is None
            or self.active is None
            or self.length is None
        ):
            raise ValueError('oracle: incomplete motion state')
        if (
            g not in (1, 2, 3)
            or not set(values) <= {'G', 'X', 'Y', 'Z', 'I', 'J', 'F'}
            or 'F' not in values
            or values['F'] <= 0
        ):
            raise ValueError('oracle: motion words')
        start = self.position
        end = tuple(values.get(a, start[i] / self.units) * self.units for i, a in enumerate('XYZ'))
        # Do not re-round inherited axes (external table pose can be any binary64).
        end = tuple(start[i] if a not in values else end[i] for i, a in enumerate('XYZ'))
        if g == 1:
            if 'I' in values or 'J' in values:
                raise ValueError('oracle: centers on line')
            curve = Line(start, end)
        else:
            if 'I' not in values or 'J' not in values:
                raise ValueError('oracle: both centers required')
            center = (start[0] + values['I'] * self.units, start[1] + values['J'] * self.units, start[2])
            sx, sy = start[0] - center[0], start[1] - center[1]
            ex, ey = end[0] - center[0], end[1] - center[1]
            if math.hypot(sx, sy) == 0 or math.hypot(sx, sy) != math.hypot(ex, ey):
                raise ValueError('oracle: exact radius')
            if start[:2] == end[:2]:
                if 'X' in values or 'Y' in values:
                    raise ValueError('oracle: full circle endpoints')
                sweep = math.tau if g == 3 else -math.tau
            else:
                angle = math.atan2(sx * ey - sy * ex, sx * ex + sy * ey)
                sweep = angle % math.tau if g == 3 else -((-angle) % math.tau)
            curve = (Arc if start[2] == end[2] else Helix)(start, end, center, sweep)
        self.position = end
        self.count += 1
        return [Motion(self.count, 'observed', self.active.tool, curve, values['F'] * self.units)]
