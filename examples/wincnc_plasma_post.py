"""WinCNC plasma post (WinCNC 3.x with its torch height control).

References: WinCNC Users Guide 3.1r5 (Command Reference: G0-G4, G20/G21/G22,
G40, G49, G90, G92, L36 "Torch Height Control On", L37 "Torch Height Control
Off"; Plasma Torch Height Control, p. 53-56; comment characters) and WinCNC's own plasma posts (SheetCAM "WinCNC Plasma THC",
Fusion "wincnc_plasma.cps"). See docs/wincnc-plasma-post.md.

Program shape (as WinCNC's posts write it):

    G90 G22 ...                    absolute, millimetres (G21 is CENTIMETRES on WinCNC)
    M65                            start macro (plasma setup)
    G0 X Y                         to the pierce point
    M61                            torch on: touch, pierce height, pierce delay,
                                   cut height, THC per the material setup
    G1/G2/G3 X Y [I J] F           the cut (I/J always relative)
    M62                            torch off
    ... M66                        end macro

Premises (declared, not checked on the machine):
- M61-M66 are user-defined macros in CNC.MAC; the defaults are those of
  WinCNC's posts. Pass your machine's codes to define_machine.
- A table whose own programs have no start macro and are run in the local
  coordinates the operator sets at the sheet (ShopSabre plasma tables with
  ProNest's post: `G90`, the cuts between M61 and M62, `M66`) is declared
  with start_code=None and local_coordinates="keep": no M65 and no G92 are
  written, so the operator's sheet zero stays in force. Each is replaced by
  a comment that states the premise to the operator (and to the decoder):
  "(WORK ZERO AS SET AT THE MACHINE)" and "(TORCH <id>)".
- material_feed_mm_min: the "Feed Rate" of the material setup chosen at the
  control (Users Guide, Plasma Torch Height Control: "XY axes feed rate for
  this material"). A program with no F word cuts at that feed, which is how
  ProNest's programs for these tables are written. Declared, no F word is
  written, and a cut planned at any other feed (a small-hole or corner
  feed) is refused: the program cannot say it. Left undeclared, every
  cutting move carries its F word and overrides the material's feed.
- program_height_control=True: ThermalCut(height_control=True) is posted
  with WinCNC's L36/L37 (height control on over each loop's cut, off over
  leads, small holes and slowed corners), as ProNest's programs for these
  tables carry them. L37 is written before the first torch-on, so the
  program does not depend on the state it starts in, and L36 before the
  end macro, so it leaves height control on as it is found. Without it a
  height control command is refused (M61 runs it by the material setup).
- ThermalCut with height_mode="fixed", pierce_delay_s=0 and
  height_control=False: pierce delay, heights and THC belong to the WinCNC
  material setup. Start the job with the torch at the cut height, so no Z
  word is written.
- WinCNC arcs are XY only (no plane words); its velocity smoothing (G9)
  must keep the path within the machine tolerance.
- Arcs: post with lowering=LoweringOptions(native_arcs=True). At most half
  a turn per arc (as WinCNC's Fusion post). WinCNC takes arcs whose start
  and end radius differ by up to its arc_err setting (WINCNC.INI; Users
  Guide 3.1r5, System Configuration: default .01 unit of measure); declare
  arc_radius_tolerance_mm (at most that, and at most 0.005 mm) and arcs
  whose rounded words differ that little stay arcs. Left at 0, only
  exactly consistent words are written and other arcs become checked
  lines.
No controller connection; verify on the machine before cutting.
"""

from dataclasses import dataclass, replace
import math

from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder

_REFERENCE = 'WinCNC Users Guide 3.1r5; WinCNC plasma posts (SheetCAM, Fusion)'


_KEEP_LOCAL = '(WORK ZERO AS SET AT THE MACHINE)'
# WinCNC Users Guide 3.1r5, Command Reference: L36 Torch Height Control On,
# L37 Torch Height Control Off.
_THC_ON, _THC_OFF = 'L36', 'L37'


def _torch_comment(machine):
    return f'(TORCH {machine.tools[0].tool.upper()})'


def _dialect(end, arc_radius_tolerance_mm=0):
    commands = (
        GCodeDialect()
        .with_commands(
            units_mm='G22',
            units_inch='G20',
            center_incremental=None,
            plane_xz=None,
            plane_yz=None,
            feed_per_minute=None,
            rpm_mode=None,
            cycles_off=None,
            exact_stop=None,
            frame_reset='G92',
            length_on=None,
            tool_change=None,
            spindle_ccw=None,
            mist=None,
            flood=None,
            coolant_off=None,
            stop=None,
            optional_stop=None,
            end=end,
            dwell=None,
            tap_synchronized=None,
            drill_cycle=None,
            drill_dwell_cycle=None,
            peck_cycle=None,
            chip_break_cycle=None,
            cycle_return_initial=None,
            bore_cycle=None,
            bore_dwell_cycle=None,
            bore_stop_cycle=None,
        )
        .commands
    )
    # I/J are always relative; feeds are units per minute; no canned cycles;
    # exact path within tolerance (smoothing setting premise).
    return replace(
        GCodeDialect(),
        commands=commands,
        parentheses_comments=True,
        radius_tolerance_mm=arc_radius_tolerance_mm,
        radius_relative_tolerance=0,
        arc_word_mismatch_mm=arc_radius_tolerance_mm,
        fixed_modes=('center_incremental', 'feed_per_minute', 'rpm_mode', 'cycles_off', 'exact_stop'),
        beam_s_max=1,
    )


@dataclass(frozen=True, kw_only=True)
class WinCNCPlasmaMachine(Machine):
    start_code: str | None = 'M65'
    # "cancel": G92 alone, which removes local coordinates (the program runs
    # in WinCNC's work coordinates). "keep": nothing is written, and the
    # program runs in the local coordinates the operator has set.
    local_coordinates: str = 'cancel'
    # Height control switched from the program with L36/L37.
    program_height_control: bool = False
    # The material setup's feed rate: no F word is written, and every cut
    # must be planned at this feed.
    material_feed_mm_min: float | None = None
    # The start/end radius mismatch an arc's words may have: WinCNC takes
    # arcs within its arc_err setting (WINCNC.INI, "allowable errors in arc
    # specifications", default .01 unit of measure). 0: exactly consistent
    # words only, and arcs that have none are posted as checked lines.
    arc_radius_tolerance_mm: float = 0
    torch_on_code: str = 'M61'
    torch_off_code: str = 'M62'
    end_code: str = 'M66'
    reference: str = _REFERENCE

    def __post_init__(self):
        super().__post_init__()
        codes = (self.torch_on_code, self.torch_off_code, self.end_code)
        codes += () if self.start_code is None else (self.start_code,)
        if len(set(codes)) != len(codes) or any(
            not (isinstance(c, str) and c.startswith('M') and c[1:].isdigit()) for c in codes
        ):
            raise ValueError(
                'declare distinct M macro codes (torch on, torch off, end, and the start macro or start_code=None)'
            )
        if self.material_feed_mm_min is not None and not (
            math.isfinite(self.material_feed_mm_min) and 0 < self.material_feed_mm_min <= self.max_feed_mm_min
        ):
            raise ValueError("material_feed_mm_min must be positive and within the machine's max_feed_mm_min")
        if self.local_coordinates not in ('cancel', 'keep'):
            raise ValueError('local_coordinates must be "cancel" (G92 is written) or "keep" (it is not)')
        if len(self.tools) != 1 or self.planes != ('xy',) or self.max_sweep_rad > math.pi:
            raise ValueError('WinCNC plasma: one torch, XY arcs of at most half a turn')
        if self.dialect != _dialect(self.end_code, self.arc_radius_tolerance_mm) or self.flood or self.mist:
            raise ValueError('dialect differs from the WinCNC plasma subset')


def define_machine(
    name,
    torch,
    *,
    start_code='M65',
    torch_on_code='M61',
    torch_off_code='M62',
    end_code='M66',
    local_coordinates='cancel',
    program_height_control=False,
    arc_radius_tolerance_mm=0,
    material_feed_mm_min=None,
    **settings,
):
    """``torch``: the torch's ToolSetting. The macro codes are those of your
    CNC.MAC (defaults: WinCNC's own plasma posts). ``start_code=None``: the
    table has no start macro. ``local_coordinates="keep"``: no G92 is
    written, so the program runs in the local coordinates the operator set
    at the sheet. ``program_height_control=True``: height control is
    switched from the program with L36/L37 (plan with height_control=True).
    ``arc_radius_tolerance_mm``: the start/end radius mismatch arc words may
    have, at most the table's arc_err (WINCNC.INI) and at most 0.005 mm.
    ``material_feed_mm_min``: the feed rate of the material setup chosen at
    the control; no F word is written and every cut must be planned at it."""
    return WinCNCPlasmaMachine(
        name,
        (torch,),
        dialect=_dialect(end_code, arc_radius_tolerance_mm),
        flood=False,
        mist=False,
        planes=('xy',),
        max_sweep_rad=math.pi,
        tool_change_position_mode='preserve_tip',
        start_code=start_code,
        local_coordinates=local_coordinates,
        program_height_control=program_height_control,
        arc_radius_tolerance_mm=arc_radius_tolerance_mm,
        material_feed_mm_min=material_feed_mm_min,
        torch_on_code=torch_on_code,
        torch_off_code=torch_off_code,
        end_code=end_code,
        **settings,
    )


class WinCNCPlasmaPost(GCodePost):
    version = 'wincnc-3.1-plasma-1'

    def words(self, addresses, values, *, dimensional=True):
        # As WinCNC's posts: dimensional words always with a decimal point.
        result = super().words(addresses, values, dimensional=dimensional)
        return [w if not dimensional or '.' in w else w + '.' for w in result]

    def on_begin(self, program):
        d = self.machine.dialect
        self.geometry_tolerance_mm = program.tolerance_mm
        self.emit('G90')
        self.emit(d.code('units_mm' if self.machine.output_units == 'mm' else 'units_inch'))
        # G92 (no local coordinates) is written once, as the work offset's
        # block, right after this header and before any motion.
        self.emit('G40')
        self.emit('G49')
        # WinCNC arcs are XY only and it has no plane words: XY is the plane.
        self._plane = 'xy'
        self._spindle_on = False
        self._coolant_mode = 'off'

    def on_work_offset(self, event):
        # WinCNC's work coordinates with no local offset (G92 alone). Its
        # G54 selects a Z head, not a work system, and is never written.
        if event.number != 1:
            raise ValueError('WinCNC plasma: work offset 1 only')
        if self.machine.local_coordinates == 'cancel':
            self.emit('G92')
        else:
            # Nothing changes the coordinates: the program says so.
            self.emit(_KEEP_LOCAL)

    def on_tool_change(self, event):
        # The start macro sets up the plasma torch (a table without one has
        # the torch set up already: it is its only tool).
        if self.machine.start_code is not None:
            self.emit(self.machine.start_code)
        else:
            self.emit(_torch_comment(self.machine))

    def on_beam(self, event):
        m = self.machine
        if event.power_percent != 0 and m.program_height_control and getattr(self, '_thc', None) is None:
            # Height control is off over a lead-in: before the first cut
            # that is the program's to say, whatever state it starts in.
            self.emit(_THC_OFF)
            self._thc = False
        self.emit(m.torch_off_code if event.power_percent == 0 else m.torch_on_code)
        self._spindle_on = event.power_percent != 0

    def on_beam_mark(self, event):
        raise ValueError(
            'WinCNC plasma: this post declares no marking method (M61 fires the torch with the '
            'cutting material setup); a ThermalMark cannot be posted for it'
        )

    def on_height_control(self, event):
        if not self.machine.program_height_control:
            raise ValueError(
                'WinCNC plasma: torch height control follows the material setup (M61 starts it '
                'by its THC mode and active delay); plan with height_control=False, or declare '
                'program_height_control=True to switch it with L36/L37'
            )
        self.emit(_THC_ON if event.on else _THC_OFF)
        self._thc = event.on

    def on_dwell(self, event):
        raise ValueError(
            'WinCNC plasma: the pierce delay belongs to the material setup (M61); plan with pierce_delay_s=0'
        )

    def on_motion(self, motion):
        c = motion.curve
        if c.start[2] != c.end[2]:
            raise ValueError(
                'WinCNC plasma programs carry no Z: plan with height_mode="fixed" and start '
                'the job with the torch at the cut height'
            )
        m = self.machine
        material = m.material_feed_mm_min
        if (
            material is not None
            and motion.kind != 'rapid'
            and abs(motion.feed_mm_min - material) > m.feed_tolerance_mm_min
        ):
            raise ValueError(
                f'WinCNC plasma: a move is planned at {motion.feed_mm_min:g} mm/min, but this machine cuts at its '
                f"material setup's feed ({material:g} mm/min, material_feed_mm_min) and writes no F word. Plan every "
                'cut at that feed (no small-hole or corner feed), or leave material_feed_mm_min undeclared so the '
                'program states its feeds'
            )
        super().on_motion(motion)
        d = m.dialect
        # No Z words; no F words where the material setup sets the feed.
        dropped = (d.axes[2], d.feed_word) if material is not None else (d.axes[2],)
        self._lines = [' '.join(w for w in line.split(' ') if not w.startswith(dropped)) for line in self._lines]

    def on_finish(self):
        # As WinCNC's posts: the end macro alone (no length offset was ever
        # turned on: the start macro only sets up the torch).
        if getattr(self, '_spindle_on', False):
            self.emit(self.machine.torch_off_code)
        if getattr(self, '_thc', None) is False:
            # Left on, as ProNest's programs leave it: a program that
            # never switches it runs with height control afterwards.
            self.emit(_THC_ON)
        self.emit(self.machine.end_code)


class WinCNCPlasmaDecoder(NativeGCodeDecoder):
    """Reads the WinCNC plasma words back into the generic decoder's model."""

    def __init__(self, machine, program):
        super().__init__(machine, program)
        # WinCNC: XY arcs only, and the torch macros own spindle-like and
        # coolant state (none is on at the start).
        self.plane = 'xy'
        self.ready |= {'plane', 'spindle_off', 'coolant_off'}

    def validate_initialized(self):
        # The frame reset (G92) is the work offset's own block: until that
        # block it may be pending; every motion needs it (offset set).
        if self.offset is None and 'frame_reset' not in self.ready:
            self.ready.add('frame_reset')
            try:
                super().validate_initialized()
            finally:
                self.ready.discard('frame_reset')
            return
        super().validate_initialized()

    def read_line(self, line, context=None):
        from camlib.motion import HeightControl, WorkOffset

        m = self.machine
        text = line.strip()
        if text == 'G92' and m.local_coordinates == 'cancel':
            # No local coordinates: WinCNC's work coordinates, offset 1.
            self.ready.add('frame_reset')
            self.offset = 1
            return [WorkOffset(1)]
        if m.local_coordinates == 'keep' and text == _KEEP_LOCAL:
            # The operator's local coordinates stay in force: offset 1.
            self.ready.add('frame_reset')
            self.offset = 1
            return [WorkOffset(1)]
        if text == 'G92' and m.local_coordinates == 'keep':
            raise ValueError('WinCNC plasma: this machine keeps its local coordinates; G92 would remove them')
        if text == (m.start_code if m.start_code is not None else _torch_comment(m)):
            # The plasma setup macro selects the torch.
            torch = m.tools[0]
            self.selected = torch.number
            self._activate(torch.number)
            return [self._length(torch.length_register)]
        if text in (_THC_ON, _THC_OFF):
            if not m.program_height_control:
                raise ValueError('WinCNC plasma: this machine does not switch height control from the program')
            # With the torch off it only sets the state the next cut starts
            # in; with it on it is the plan's height control command.
            return [HeightControl(text == _THC_ON)] if self.rpm else []
        if text == m.torch_on_code:
            return super().read_line('M3 S1', context)
        words = text.split(' ')
        if m.material_feed_mm_min is not None and words[0] in ('G1', 'G2', 'G3'):
            d = m.dialect
            if any(w.startswith(d.feed_word) for w in words[1:]):
                raise ValueError(
                    "WinCNC plasma: this machine cuts at its material setup's feed; an F word overrides it"
                )
            # The controller cuts at the material setup's feed.
            unit = 25.4 if m.output_units == 'inch' else 1
            line = f'{text} {d.feed_word}{m.material_feed_mm_min / unit!r}'
        if text == m.torch_off_code:
            return super().read_line('M5', context)
        if text == m.end_code:
            # The torch was selected by a macro, not a length offset.
            self.length = None
            return super().read_line(line, context)
        return super().read_line(line, context)
