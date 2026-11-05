"""Ordinary Python 808D ISO milling reference, February 2016 manuals.

See docs/siemens808d-iso-post.md for declared machine settings and evidence.
The milling H map is $TC_DPH, not the native per-tool D cutting-edge number.
"""

from dataclasses import dataclass, replace
import math
import re

from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder, LoweringOptions, post_events

_WORDS = re.compile(r'([A-Z])\s*([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))')
_PREFIX = ('G290', 'COMPOF', 'RNDM=0', 'CFTCP', 'G601', 'G291')
_RESETS = ('G50', 'G69', 'G50.1 X0. Y0. Z0.')
_RESERVED = {0, 1, 2, 3, 4, 5, 6, 17, 19, 29, 30, 40, 41, 42, 43, 44, 45, 70, 98, 99}


def _dialect(tool_change, coolant_codes):
    if not set(coolant_codes) <= {'off', 'flood', 'mist'} or 'off' not in coolant_codes:
        raise ValueError('declare OEM off and optional flood/mist coolant M functions')
    for code in coolant_codes.values():
        if not re.fullmatch(r'M(?:0|[1-9][0-9]?)', code) or int(code[1:]) in _RESERVED:
            raise ValueError('coolant must use a free OEM M function in M0..M99')
    return replace(
        GCodeDialect(), tool_change=tool_change, radius_tolerance_mm=0, fixed_modes=('center_incremental', 'rpm_mode')
    ).with_commands(
        rapid=None,
        plane_xz=None,
        plane_yz=None,
        center_incremental=None,
        rpm_mode=None,
        exact_stop='G61',
        end='M30',
        tap_synchronized=None,
        dwell=None,
        radius_left=None,
        radius_right=None,
        drill_cycle=None,
        drill_dwell_cycle=None,
        peck_cycle=None,
        chip_break_cycle=None,
        cycle_return_initial=None,
        bore_cycle=None,
        bore_dwell_cycle=None,
        bore_stop_cycle=None,
        coolant_off=coolant_codes['off'],
        flood=coolant_codes.get('flood'),
        mist=coolant_codes.get('mist'),
    )


@dataclass(frozen=True, kw_only=True)
class ReferenceMachine(Machine):
    reference: str = '808D ADVANCED ISO milling; programming and parameter manuals 02/2016'
    startup_units: str
    extern_toolprog_mode: int
    cutting_edge_default: int
    tool_corr_move_mode: int
    tool_length_const: int
    optional_stop_enabled: bool = False
    curve_policy: str = 'linear'

    def __post_init__(self):
        super().__post_init__()
        if self.startup_units != self.output_units:
            raise ValueError('set output units to the existing controller units; runtime unit switching is not modeled')
        if type(self.extern_toolprog_mode) is not int or self.extern_toolprog_mode not in (0, 4, 64, 68):
            raise ValueError('ISO milling reference accepts MD10890 bits2/6 only; bit2 is turning-only')
        if (
            type(self.cutting_edge_default) is not int
            or self.cutting_edge_default != 0
            or type(self.tool_corr_move_mode) is not int
            or self.tool_corr_move_mode != 0
            or type(self.tool_length_const) is not int
            or self.tool_length_const != 17
        ):
            raise ValueError(
                'reference needs MD20270=0, MD20382=0 and SD42940=17 for explicit deferred Z length compensation'
            )
        if type(self.optional_stop_enabled) is not bool:
            raise ValueError('declare optional stop enablement')
        if self.traverse_feed_mm_min is None:
            raise ValueError('declare traverse_feed_mm_min; independent-axis G0 is not a straight swept path')
        if self.planes != ('xy',) or self.curve_policy not in ('linear', 'exact_ijk'):
            raise ValueError('reference uses G17 and linear/exact_ijk curves')
        if (
            self.arcs != (self.curve_policy == 'exact_ijk')
            or self.helices != self.arcs
            or self.max_sweep_rad > math.tau
        ):
            raise ValueError('reference curve capabilities must match single-turn policy')
        if self.decimals > (3 if self.output_units == 'mm' else 4):
            raise ValueError('reference emits at most IS-B precision: 3 metric or 4 inch decimals')
        if (
            self.work_to_machine.x != (1, 0, 0)
            or self.work_to_machine.y != (0, 1, 0)
            or self.work_to_machine.z != (0, 0, 1)
        ):
            raise ValueError('reference stored work offset must be a Cartesian translation')
        d = self.dialect
        coolant = {'off': d.code('coolant_off')}
        coolant.update({c.effect: c.code for c in d.commands if c.effect in ('flood', 'mist')})
        if d != _dialect(d.tool_change, coolant):
            raise ValueError('reference dialect differs from documented ISO milling subset')
        if self.flood != ('flood' in coolant) or self.mist != ('mist' in coolant):
            raise ValueError('coolant capabilities must match explicit OEM bindings')


def define_machine(
    name,
    tools,
    *,
    startup_units,
    extern_toolprog_mode,
    cutting_edge_default,
    tool_corr_move_mode,
    tool_length_const,
    tool_change,
    coolant_codes,
    traverse_feed_mm_min,
    work_to_machine,
    **settings,
):
    """Bind installed settings without changing the controller or tool table.

    Each ToolSetting.length_register is its global ISO H ($TC_DPH) mapping.
    position_after_mm declares the physical tip pose after the OEM tool change;
    MD20382=0 means G43 H / G49 without axes must not traverse the length offset.
    """
    d = _dialect(tool_change, coolant_codes)
    curve_policy = settings.pop('curve_policy', 'linear')
    return ReferenceMachine(
        name,
        tools,
        dialect=d,
        planes=('xy',),
        arcs=curve_policy == 'exact_ijk',
        helices=curve_policy == 'exact_ijk',
        curve_policy=curve_policy,
        startup_units=startup_units,
        extern_toolprog_mode=extern_toolprog_mode,
        cutting_edge_default=cutting_edge_default,
        tool_corr_move_mode=tool_corr_move_mode,
        tool_length_const=tool_length_const,
        flood='flood' in coolant_codes,
        mist='mist' in coolant_codes,
        traverse_feed_mm_min=traverse_feed_mm_min,
        work_to_machine=work_to_machine,
        **settings,
    )


class Siemens808DISOPost(GCodePost):
    version = '808d-iso-milling-2016-1'

    def on_begin(self, program):
        for line in _PREFIX:
            self.emit(line)
        super().on_begin(program)
        # Cancel scale/rotation/mirror before resetting the coordinate system.
        i = self._lines.index('G92.1')
        self._lines[i : i + 1] = [*_RESETS, 'G92.1 X0. Y0. Z0.']

    def words(self, addresses, values, *, dimensional=True):
        words = super().words(addresses, values, dimensional=dimensional)
        # Literal decimals remove dependence on calculator/standard notation
        # and IS-B/IS-C input scaling, including the distinct F scaling factors.
        return [w if '.' in w else w + '.' for w in words] if dimensional else words


class Siemens808DISODecoder(NativeGCodeDecoder):
    version = '808d-iso-milling-2016-nominal-1'

    def __init__(self, machine, program):
        super().__init__(machine, program)
        self.prefix = 0
        self.resets = set()

    def validate_initialized(self):
        super().validate_initialized()
        if self.prefix != len(_PREFIX) or self.resets != {50, 69, 50.1}:
            raise ValueError('incomplete ISO language/compressor/feed/scale/rotation/mirror initialization')

    def read_line(self, line, context=None):
        if (
            self.ended
            or len(line) > 1024
            or not line.isascii()
            or any(ord(c) < 32 and c != '\t' or ord(c) == 127 for c in line)
        ):
            raise ValueError('invalid ISO physical block or code after end')
        body = line.partition(';')[0].strip().upper()
        # Native parentheses are expressions, ISO parentheses are comments.
        if self.prefix == len(_PREFIX):
            body = re.sub(r'\([^()]*\)', ' ', body).strip()
        if '(' in body or ')' in body:
            raise ValueError('invalid comment for current language')
        body = re.sub(r'^N[0-9]{1,8}(?=\s|[A-Z]|$)', '', body).strip()
        if not body:
            return []
        if self.prefix < len(_PREFIX):
            if body != _PREFIX[self.prefix]:
                raise ValueError('expected standalone native preparation then G291')
            self.prefix += 1
            return []
        if body == 'G291':
            return []  # language reselection preserves tool/length/work offset
        pieces = _WORDS.findall(body)
        if _WORDS.sub('', body).strip() or len({w for w, v in pieces}) != len(pieces):
            raise ValueError('unsupported or duplicate ISO words')
        values = {w: float(v) for w, v in pieces}
        if any(not math.isfinite(v) or abs(v) > 1e9 for v in values.values()) or 'G' in values and 'M' in values:
            raise ValueError('invalid or mixed ISO commands')
        g, m = values.get('G'), values.get('M')
        if g in (0, 290, 291):
            raise ValueError('G0/unprepared language changes are outside this reference')
        precision = 3 if self.machine.output_units == 'mm' else 4
        if any(w in 'XYZIJKF' and ('.' not in v or len(v.partition('.')[2]) > precision) for w, v in pieces):
            raise ValueError('ISO dimensional words require explicit decimals within IS-B precision')
        if g in (20, 21):
            if set(values) != {'G'} or g != (21 if self.machine.startup_units == 'mm' else 20):
                raise ValueError('unit block must be standalone and preserve declared startup units')
        if g in (50, 69):
            if set(values) != {'G'}:
                raise ValueError('scale/rotation cancel must be standalone')
            self.resets.add(g)
            return []
        if g in (50.1, 92.1):
            if set(values) != {'G', 'X', 'Y', 'Z'} or any(values[a] != 0 for a in 'XYZ'):
                raise ValueError('mirror/frame reset requires explicit zero XYZ')
            if g == 50.1:
                self.resets.add(g)
            else:
                if self.resets != {50, 69, 50.1}:
                    raise ValueError('cancel transforms before coordinate reset')
                self.ready.add('frame_reset')
            return []
        if (m in (0, 1, 6) or 'T' in values) and (self.rpm or self.coolant != 'off'):
            raise ValueError('explicit spindle/coolant off required before OEM stop/tool change')
        if m == 1 and not self.machine.optional_stop_enabled:
            raise ValueError('optional stop must be declared enabled')
        if 'S' in values and values['S'] != int(values['S']):
            raise ValueError('reference spindle RPM must be integral')
        return super().read_line(body, context)


def post_iso(program, machine, *, lowering=None, **options):
    return post_events_iso(program, program.events, machine, lowering=lowering, **options)


def post_events_iso(header, events, machine, *, lowering=None, **options):
    if lowering is None and machine.curve_policy == 'linear':
        lowering = LoweringOptions()
    return post_events(
        header,
        events,
        machine=machine,
        post=Siemens808DISOPost,
        decoder=Siemens808DISODecoder,
        lowering=lowering,
        **options,
    )
