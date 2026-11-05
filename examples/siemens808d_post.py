"""Ordinary Python reference post for SINUMERIK 808D ADVANCED native milling4.90.

Manual/provenance/supported subset: docs/siemens808d-post.md. OEM toolchange and
coolant bindings are required inputs. This example never connects to a machine.
"""

from dataclasses import dataclass, replace
import math
import re

from camlib.motion import Helix
from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder, LoweringOptions, post_events


_RESERVED_M = frozenset((0, 1, 2, 3, 4, 5, 6, 17, 19, 30, 40, 41, 42, 43, 44, 45, 70))
_WORD_LIMIT_MM = 99999.999


def _check_coolant(codes):
    if set(codes) - {'off', 'flood', 'mist'} or 'off' not in codes:
        raise ValueError('declare OEM coolant off and optional flood/mist codes')
    if any(not isinstance(code, str) or not re.fullmatch(r'M[0-9]{1,2}', code) for code in codes.values()):
        raise ValueError('reference coolant codes must be documented OEM M0..M99')
    if any(int(code[1:]) in _RESERVED_M for code in codes.values()):
        raise ValueError('OEM coolant binding cannot reinterpret a reserved native M function')


@dataclass(frozen=True, kw_only=True)
class ReferenceMachine(Machine):
    reference: str = '808D ADVANCED milling4.90 native;6FC5398-4DP10-0BA7'
    curve_policy: str = 'linear'
    stop_window: str = 'fine'

    def __post_init__(self):
        super().__post_init__()
        if self.curve_policy not in ('linear', 'exact_ijk') or self.stop_window not in ('fine', 'coarse'):
            raise ValueError('choose linear/exact_ijk curves and fine/coarse stop window')
        if (
            self.planes != ('xy',)
            or self.max_tool_number > 32000
            or self.max_length_register > 9
            or len(self.tools) > 64
        ):
            raise ValueError('reference subset requires G17, T1..32000, D1..9 and <=64 configured tool/edge fields')
        if math.dist(self.work_to_machine.z, (0, 0, 1)) > 1e-12:
            raise ValueError('reference subset requires vertical machine tool axis')
        if self.dialect.length_activation != 'tool_edge' or self.dialect.radius_tolerance_mm != 0:
            raise ValueError('reference uses per-tool D and uncorrected exact-IJK circles only')
        if self.arcs != (self.curve_policy == 'exact_ijk') or self.helices != (self.curve_policy == 'exact_ijk'):
            raise ValueError('curve capabilities must match reference policy')
        expected = {
            'units_mm': 'G710',
            'units_inch': 'G700',
            'absolute': 'G90',
            'plane_xy': 'G17',
            'feed_per_minute': 'G94',
            'radius_off': 'G40',
            'length_off': 'D0',
            'cycles_off': 'MCALL',
            'frame_reset': 'TRANS',
            'exact_stop': 'G60',
            'language_native': 'G290',
            'compression_off': 'COMPOF',
            'rapid': 'G0',
            'linear': 'G1',
            'arc_cw': 'G2',
            'arc_ccw': 'G3',
            'spindle_cw': 'M3',
            'spindle_ccw': 'M4',
            'spindle_off': 'M5',
            'tool_change': 'M6',
            'stop': 'M0',
            'optional_stop': 'M1',
            'end': 'M2',
        }
        if any(self.dialect.code(effect) != code for effect, code in expected.items()):
            raise ValueError('native reference command mapping differs from documented subset')
        if (
            self.dialect.axes != ('X', 'Y', 'Z')
            or self.dialect.centers != ('I', 'J', 'K')
            or (self.dialect.tool_word, self.dialect.length_word, self.dialect.feed_word, self.dialect.speed_word)
            != ('T', 'D', 'F', 'S')
        ):
            raise ValueError('native reference uses XYZ/IJK/T/D/F/S')
        if set(self.dialect.fixed_modes) != {'center_incremental', 'rpm_mode'} or self.dialect.parentheses_comments:
            raise ValueError('only native center/RPM invariants are fixed; parentheses are expressions')
        forbidden = {'center_incremental', 'rpm_mode', 'length_on'}
        if any(command.effect in forbidden for command in self.dialect.commands):
            raise ValueError('reference native invariants have no ISO/LinuxCNC opcode aliases')
        names = {'coolant_off': 'off', 'flood': 'flood', 'mist': 'mist'}
        _check_coolant({names[c.effect]: c.code for c in self.dialect.commands if c.effect in names})
        if max(self.max_feed_mm_min, self.max_spindle_rpm) > _WORD_LIMIT_MM or self.max_sweep_rad > math.tau:
            raise ValueError('native reference exceeds documented feed/RPM or single-turn subset')
        if any(not math.isfinite(v) or abs(v) > _WORD_LIMIT_MM for t in self.tools for v in t.position_after_mm):
            raise ValueError('reference tool/edge work poses exceed bounded software coordinate range')
        if self.dialect.work_offsets != ('G54', 'G55', 'G56', 'G57', 'G58', 'G59'):
            raise ValueError('native reference supports six explicit stored work offsets')


def define_machine(
    name, tools, *, tool_change, coolant_codes, work_to_machine, curve_policy='linear', stop_window='fine', **settings
):
    """Define the machine in Python. Required OEM bindings are not inferred.

    coolant_codes is {'off': 'M...', 'flood': 'M...', 'mist': 'M...'}; flood/mist
    optional. The caller supplies the actual PLC meanings and tool/edge poses.
    Linear policy uses bounded G1 for curves. exact_ijk permits only circles whose
    emitted start/end radii match exactly; unspecified controller corrections reject.
    """
    _check_coolant(coolant_codes)
    d = replace(
        GCodeDialect(),
        length_word='D',
        tool_change=tool_change,
        length_activation='tool_edge',
        parentheses_comments=False,
        radius_tolerance_mm=0,
        fixed_modes=('center_incremental', 'rpm_mode'),
    ).with_commands(
        units_mm='G710',
        units_inch='G700',
        center_incremental=None,
        rpm_mode=None,
        length_off='D0',
        length_on=None,
        cycles_off='MCALL',
        frame_reset='TRANS',
        exact_stop='G60',
        language_native='G290',
        compression_off='COMPOF',
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
    return ReferenceMachine(
        name,
        tools,
        dialect=d,
        planes=('xy',),
        arcs=curve_policy == 'exact_ijk',
        helices=curve_policy == 'exact_ijk',
        max_tool_number=32000,
        max_length_register=9,
        flood='flood' in coolant_codes,
        mist='mist' in coolant_codes,
        work_to_machine=work_to_machine,
        curve_policy=curve_policy,
        stop_window=stop_window,
        **settings,
    )


class Siemens808DPost(GCodePost):
    version = '808d-advanced-490-native-2'

    def on_begin(self, program):
        self._active_number = None
        super().on_begin(program)
        self.emit('G601' if self.machine.stop_window == 'fine' else 'G602')
        self.emit('RNDM=0')
        self.emit('CFTCP')  # manual99: F acts at cutter center

    def on_tool_change(self, event):
        if self._active_number == event.number:
            self.emit('D' + str(event.length_register))
        elif self.machine.dialect.tool_change == 'direct_t':
            self.emit('T' + str(event.number), 'D' + str(event.length_register))
        else:
            self.emit('T' + str(event.number))
            self.emit('M6', 'D' + str(event.length_register))
        self._active_number = event.number

    def words(self, addresses, values, *, dimensional=True):
        if any(abs(v) > _WORD_LIMIT_MM for v in values):
            raise ValueError('native reference coordinate/feed range exceeded')
        return super().words(addresses, values, dimensional=dimensional)

    def on_motion(self, motion):
        super().on_motion(motion)
        if isinstance(motion.curve, Helix):
            line = self._lines.pop()
            self.emit(line, 'TURN=0')  # zero additional turns; manual98-99/345


class Siemens808DDecoder(NativeGCodeDecoder):
    """Strict native4.90 subset; independently reads actual emitted blocks.

    Canonical tool changes explicitly select D before motion. Implicit D1 at tool
    activation is tracked, but an omitted explicit D is rejected in this subset.
    Nonzero rounding/compression, ISO, tilted length planes and circle correction
    are unsupported. No actual hardware or machine-data discovery is implied.
    """

    version = '808d-advanced-490-native-model-1'

    def __init__(self, machine, program):
        super().__init__(machine, program)
        self.native_mode = False
        self.window = None
        self.rounding_off = False
        self.circle_feed_off = False
        self.implicit_edge = None

    def validate_initialized(self):
        super().validate_initialized()
        if (
            not self.native_mode
            or self.window != self.machine.stop_window
            or not self.rounding_off
            or not self.circle_feed_off
        ):
            raise ValueError('native language, stop window, RNDM=0 and CFTCP must be explicit')

    def read_line(self, line, context=None):
        if self.ended:
            raise ValueError('NC after program end')
        if len(line) > 1024 or not line.isascii() or any(ord(c) < 32 and c != '\t' or ord(c) == 127 for c in line):
            raise ValueError('invalid native physical block')
        body = line.split(';', 1)[0].strip().upper()
        if not body:
            return []
        if body.startswith('N'):
            match = re.match(r'N[0-9]{1,8}(?=\s|[A-Z]|$)', body)
            if match is None:
                raise ValueError('native block number must be integer 0..99999999')
            body = body[match.end() :].strip()
        if re.search(r'G\s*0*29[01](?:\.0*)?(?![0-9.])', body):
            if body != 'G290':
                raise ValueError('only standalone G290 native mode supported; ISO is separate')
            self.native_mode = True
        if not self.native_mode:
            raise ValueError('select native G290 before native instructions')
        if body in ('G601', 'G602'):
            self.window = 'fine' if body == 'G601' else 'coarse'
            if self.window != self.machine.stop_window:
                raise ValueError('exact-stop window differs from machine declaration')
            return []
        if body == 'CFTCP':
            self.circle_feed_off = True
            return []
        if re.fullmatch(r'RNDM\s*=\s*0(?:\.0*)?', body):
            self.rounding_off = True
            return []
        has_turn = 'TURN' in body
        if has_turn:
            match = re.fullmatch(r'(G[23]\s+.+?)\s+TURN\s*=\s*0', body)
            if match is None:
                raise ValueError('only explicit TURN=0 native helix subset supported')
            body = match[1]
        if body == 'D0':
            self.implicit_edge = None
        result = super().read_line(body, context)
        for event in result:
            curve = getattr(event, 'curve', None)
            if curve is not None:
                values = curve.end
                if hasattr(curve, 'center'):
                    values = (*values, *(c - a for c, a in zip(curve.center, curve.start)))
                if any(abs(v) > _WORD_LIMIT_MM for v in values):
                    raise ValueError('decoded native reference coordinate range exceeded')
        helical = any(isinstance(getattr(event, 'curve', None), Helix) for event in result)
        if helical != has_turn:
            raise ValueError('native helix subset requires explicit TURN=0; no TURN on other forms')
        return result

    def _activate(self, number):
        super()._activate(number)
        self.length = 1
        self.implicit_edge = 1  # manual115; no traverse before explicit D

    def _length(self, number):
        event = super()._length(number)
        self.implicit_edge = None
        return event


def post_808d(program, machine, *, lowering=None, **options):
    return post_events_808d(program, program.events, machine, lowering=lowering, **options)


def post_events_808d(header, events, machine, *, lowering=None, **options):
    if lowering is None and machine.curve_policy == 'linear':
        lowering = LoweringOptions()
    return post_events(
        header, events, machine=machine, post=Siemens808DPost, decoder=Siemens808DDecoder, lowering=lowering, **options
    )
