"""LinuxCNC QtPlasmaC plasma post (LinuxCNC 2.9 QtPlasmaC documentation).

Writes ``ThermalCut`` programs as QtPlasmaC expects them and reads its own
output back for checking. Setup premises, declared here and in
docs/qtplasmac-post.md:

- The torch is QtPlasmaC tool 0 (``T0 M6``, ``G43 H0``); camlib's tool table
  entry for it is number 1, length register 1. Z belongs to QtPlasmaC, so
  the length offset does not move the cut.
- QtPlasmaC owns Z: its loader removes Z motion, probes the plate and handles
  pierce and cut heights from the material file. Plan with
  ``height_mode="fixed"`` (one Z, the cut height).
- ``M52 P1`` (paused motion) is in the preamble, as the documentation requires.
  Path blending is set explicitly with ``G64 P<blend_mm>``; the blend must be
  within half of the machine's tolerance (the loader would otherwise insert
  ``G64 P0.1``).
- Torch on ``M3 $0 S1``, off ``M5 $0``; torch height control off/on with
  ``M62 P2``/``M63 P2`` (synchronized with motion).
- Feeds are written as F words; QtPlasmaC warns if they differ from the
  selected material's cut feed rate.

No controller connection; verify on the machine before cutting.
"""

from dataclasses import dataclass, replace

from camlib.post import Machine, GCodeDialect, GCodePost, NativeGCodeDecoder


def _dialect():
    commands = (
        GCodeDialect(beam_s_max=1, height_control_word='P2')
        .with_commands(
            exact_stop='G64',
            center_incremental=None,
            rpm_mode=None,
            mist=None,
            flood=None,
            height_control_on='M63',
            height_control_off='M62',
        )
        .commands
    )
    return replace(
        GCodeDialect(beam_s_max=1, height_control_word='P2'),
        commands=commands,
        fixed_modes=('center_incremental', 'rpm_mode'),
    )


@dataclass(frozen=True, kw_only=True)
class QtPlasmaCMachine(Machine):
    blend_mm: float = 0.005
    reference: str = 'LinuxCNC 2.9 QtPlasmaC documentation'

    def __post_init__(self):
        super().__post_init__()
        if not 0 < self.blend_mm <= self.tolerance_mm / 2:
            raise ValueError('G64 blend must be positive and within half of tolerance_mm')
        if len(self.tools) != 1 or self.tools[0].number != 1 or self.tools[0].length_register != 1:
            raise ValueError('QtPlasmaC: one torch, entered as number 1 / length register 1 (posted T0 / H0)')
        if self.dialect != _dialect() or self.flood or self.mist:
            raise ValueError('dialect differs from the QtPlasmaC subset')


def define_machine(name, tool, *, blend_mm=0.005, **settings):
    """``tool``: the torch ToolSetting (number 1, length register 1)."""
    return QtPlasmaCMachine(
        name,
        (tool,),
        dialect=_dialect(),
        flood=False,
        mist=False,
        blend_mm=blend_mm,
        tool_change_position_mode='preserve_tip',
        **settings,
    )


class QtPlasmaCPost(GCodePost):
    version = 'qtplasmac-2.9-1'

    def on_begin(self, program):
        super().on_begin(program)
        blend = f'{self.machine.blend_mm:.4f}'.rstrip('0')
        self._lines = [f'G64 P{blend}' if line == 'G64' else line for line in self._lines]
        self.emit('M52 P1')

    def on_tool_change(self, event):
        self.emit('T0 M6')
        self.emit('G43 H0')

    def on_beam(self, event):
        self.emit('M5 $0' if event.power_percent == 0 else 'M3 $0 S1')
        self._spindle_on = event.power_percent != 0

    def on_finish(self):
        super().on_finish()
        self._lines = ['M5 $0' if line == 'M5' else line for line in self._lines]


class QtPlasmaCDecoder(NativeGCodeDecoder):
    """Reads the QtPlasmaC words back into the generic decoder's subset."""

    def read_line(self, line, context=None):
        text = line.strip()
        if text == 'M52 P1':
            return []
        if text.startswith('G64 P'):
            if float(text[5:]) > self.machine.blend_mm + 1e-12:
                raise ValueError('G64 blend exceeds the declared blend_mm')
            return super().read_line('G64', context)
        if text == 'T0 M6':
            return [*super().read_line('T1', context), *super().read_line('M6', context)]
        if text == 'G43 H0':
            return super().read_line('G43 H1', context)
        if text == 'M3 $0 S1':
            return super().read_line('M3 S1', context)
        if text == 'M5 $0':
            return super().read_line('M5', context)
        if '$' in text:
            raise ValueError('unsupported QtPlasmaC spindle selector')
        return super().read_line(line, context)
