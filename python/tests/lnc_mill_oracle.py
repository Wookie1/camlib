"""Literal A-LNC milling manual (V01.02.000) oracle; imports only motion data.

Reads posted words as the manual defines them (no post or decoder code):
G01 lines, G02/G03 arcs with incremental I/J/K in G17/G18/G19 (p.7) checked
against the parameter 50048 radius error, and explicit G43/G49 blocks whose
physical spindle Z must not move (p.38).
"""

import math
import re
from camlib.motion import Arc, Line, Motion

# (u, v, normal) axes of each plane; positive sweep is counterclockwise
# looking down the normal (G03).
_PLANES = {17: ('xy', 0, 1, 2), 18: ('xz', 2, 0, 1), 19: ('yz', 1, 2, 0)}


class Oracle:
    def __init__(self, *, tools, initial_position, radius_tolerance_mm, work_offset=1, m_codes=None):
        self.tools = {t.number: t for t in tools}
        self.position = tuple(initial_position)
        self.tolerance = radius_tolerance_mm
        self.work_offset = work_offset
        m_codes = m_codes or dict(
            tool_change='M6', spindle_cw='M3', spindle_ccw='M4', spindle_off='M5', coolant_off='M9', flood='M8'
        )
        self.m_codes = {int(code[1:]): effect for effect, code in m_codes.items()}
        self.ended = False
        self.flags = set()
        self.plane = None
        self.active = self.selected = self.length = None
        self.offset = None
        self.rpm = 0
        self.coolant = 'off'
        self.count = 0
        self.transitions = []

    def read(self, line):
        if self.ended:
            raise ValueError('oracle: output after M30')
        compact = ''.join(line.split())
        if not compact:
            return []
        parts = re.findall(r'([A-Z])([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))', compact)
        if ''.join(a + b for a, b in parts) != compact:
            raise ValueError('oracle: syntax')
        gs = [float(v) for a, v in parts if a == 'G']
        words = {a: float(v) for a, v in parts if a != 'G'}
        if len(words) != len(parts) - len(gs):
            raise ValueError('oracle: duplicate address')
        for a, text in parts:
            if a in 'XYZIJKF' and '.' not in text:
                raise ValueError('oracle: dimensional word without a decimal point')
        if gs in ([1, 43], [1, 49]):
            if self.active is None or self.rpm or self.coolant != 'off' or self.offset is None:
                raise ValueError('oracle: compensation setup')
            # The position is the tip; the spindle gauge line is tip + gauge
            # length and must not move in a compensation block.
            before = self.position[2] + self.active.gauge_length_mm
            if gs[-1] == 43:
                if set(words) != {'H', 'Z', 'F'} or words['H'] != self.active.length_register or self.length:
                    raise ValueError('oracle: H activation')
                after = words['Z'] + self.active.gauge_length_mm
                self.length = int(words['H'])
            else:
                if set(words) != {'Z', 'F'} or self.length is None:
                    raise ValueError('oracle: G49')
                after = words['Z']
                self.length = None
            if abs(before - after) > 1e-9:
                raise ValueError('oracle: unmodeled compensation movement')
            self.transitions.append((int(gs[-1]), round(after, 9)))
            return []
        if len(gs) > 1:
            raise ValueError('oracle: combined G codes')
        g = gs[0] if gs else None
        if g == 21:
            self.flags.add('units')
            return []
        if g in (17, 18, 19):
            if words:
                raise ValueError('oracle: plane words')
            self.plane = int(g)
            return []
        if g in (90, 94, 40, 49, 80, 61):
            if words or g == 49 and self.length is not None:
                raise ValueError('oracle: state/implicit length motion')
            self.flags.add(int(g))
            return []
        if g == 52:
            if set(words) != {'X', 'Y', 'Z'} or any(words.values()):
                raise ValueError('oracle: local shift')
            self.flags.add('frame')
            return []
        if g in (54, 55, 56, 57, 58, 59):
            if words or g - 53 != self.work_offset:
                raise ValueError('oracle: wrong WCS')
            self.offset = int(g - 53)
            return []
        if set(words) == {'T'} and g is None:
            if words['T'] not in self.tools:
                raise ValueError('oracle: unknown T')
            self.selected = int(words['T'])
            return []
        if 'M' in words and g is None:
            code = words['M']
            effect = self.m_codes.get(code)
            if effect in ('spindle_cw', 'spindle_ccw'):
                if set(words) != {'M', 'S'} or self.length is None or words['S'] <= 0:
                    raise ValueError('oracle: spindle state')
                self.rpm = words['S']
                return []
            if set(words) != {'M'}:
                raise ValueError('oracle: M words')
            if effect == 'spindle_off':
                self.rpm = 0
                self.flags.add('spindle')
            elif effect == 'coolant_off':
                self.coolant = 'off'
                self.flags.add('coolant')
            elif effect == 'flood':
                self.coolant = 'flood'
            elif effect == 'tool_change':
                if self.rpm or self.coolant != 'off' or self.selected is None or self.length is not None:
                    raise ValueError('oracle: tool change state')
                self.active = self.tools[self.selected]
                self.position = tuple(self.active.position_after_mm)
            elif code == 30:
                if self.rpm or self.coolant != 'off' or self.length is not None:
                    raise ValueError('oracle: end state')
                self.ended = True
            else:
                raise ValueError('oracle: unknown M code')
            return []
        required = {'units', 'frame', 'spindle', 'coolant', 90, 94, 40, 49, 80, 61}
        if not required <= self.flags or self.active is None or self.length is None or self.offset is None:
            raise ValueError('oracle: motion state')
        start = self.position
        if g == 1:
            if not set(words) <= {'X', 'Y', 'Z', 'F'} or words.get('F', 0) <= 0:
                raise ValueError('oracle: G1 words')
            end = tuple(words.get(a, start[i]) for i, a in enumerate('XYZ'))
            self.position = end
            if end == start:
                return []
            self.count += 1
            return [Motion(self.count, 'observed', self.active.tool, Line(start, end), words['F'])]
        if g in (2, 3):
            if self.plane is None:
                raise ValueError('oracle: arc without a plane')
            name, u, v, n = _PLANES[self.plane]
            axis = 'XYZ'
            centre_words = {0: 'I', 1: 'J', 2: 'K'}
            allowed = {axis[u], axis[v], centre_words[u], centre_words[v], 'F'}
            if not set(words) <= allowed or 'F' not in words or axis[n] in words:
                raise ValueError('oracle: arc words')
            end = list(start)
            end[u] = words.get(axis[u], start[u])
            end[v] = words.get(axis[v], start[v])
            centre = list(start)
            centre[u] = start[u] + words.get(centre_words[u], 0.0)
            centre[v] = start[v] + words.get(centre_words[v], 0.0)
            r1 = math.hypot(start[u] - centre[u], start[v] - centre[v])
            r2 = math.hypot(end[u] - centre[u], end[v] - centre[v])
            if abs(r1 - r2) > self.tolerance + 1e-12:
                raise ValueError('oracle: alarm 510204, end point not on the arc')
            a1 = math.atan2(start[v] - centre[v], start[u] - centre[u])
            a2 = math.atan2(end[v] - centre[v], end[u] - centre[u])
            sweep = (a2 - a1) % math.tau
            if g == 2:
                sweep -= math.tau
            if abs(sweep) < 1e-12 or abs(abs(sweep) - math.tau) < 1e-12:
                raise ValueError('oracle: zero or full circle')
            self.position = tuple(end)
            self.count += 1
            return [
                Motion(
                    self.count,
                    'observed',
                    self.active.tool,
                    Arc(start, tuple(end), tuple(centre), sweep, name),
                    words['F'],
                )
            ]
        raise ValueError(f'oracle: unsupported block {line!r}')
