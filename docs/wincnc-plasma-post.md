# WinCNC plasma post

[`examples/wincnc_plasma_post.py`](../examples/wincnc_plasma_post.py) writes
`ThermalCut` programs for WinCNC 3.x plasma tables with WinCNC torch height
control, and reads its own output back for checking. References: the WinCNC
Users Guide 3.1r5 (command reference, plasma torch height control) and
WinCNC's own plasma posts (SheetCAM "WinCNC Plasma THC", Fusion
`wincnc_plasma.cps`).

```python
import wincnc_plasma_post as w

torch = camlib.Torch("torch", kerf_width=1.5)
# camlib assumes nothing about the machine: declare it.
declared = dict(
    output_units="mm",
    decimals=3,
    max_feed_mm_min=15000,
    max_spindle_rpm=1,
    arcs=True,
    helices=False,
    max_tool_number=1,
    max_length_register=1,
    tool_changes=True,
    rapid_mm_min=15000,
)
machine = w.define_machine("Plasma table", ToolSetting.for_tool(torch, number=1, length_register=1,
                                                                 position_after_mm=start), **declared)
program = job.post_nc("sheet", machine=machine, post=w.WinCNCPlasmaPost,
                      decoder=w.WinCNCPlasmaDecoder,
                      lowering=camlib.LoweringOptions(native_arcs=True))
```

| Item | Output | Premise |
| --- | --- | --- |
| Units | `G22` (mm) or `G20` (inch) | On WinCNC `G21` means centimetres |
| Header | `G90 G22 G40 G49`, then `G92` | `G92` alone removes local coordinates: written once, as the work offset (WinCNC's `G54` selects a Z head and is never written) |
| Start | `M65` | Plasma setup macro |
| Each cut | `G0 X Y`, `M61`, the cut, `M62` | M61 fires the torch: touch, pierce height, pierce delay, cut height, THC per the material setup; M62 turns it off |
| Arcs | `G2`/`G3 X Y I J`, at most half a turn | I/J are always relative in WinCNC; no plane words (XY only) |
| Inches | `output_units="inch"`, `decimals=4`: `G20`, every word in inches | A half-turn arc is kept as an arc only where its rounded inch words still span no more than half a turn; otherwise it is written as checked lines (with `LoweringOptions(native_arcs=True)`) |
| Feeds | `F` on every cutting move | |
| Z | None | Plan with `height_mode="fixed"` and start with the torch at the cut height |
| End | `M66` | End macro |

M61–M66 are user-defined macros in WinCNC's `CNC.MAC`. The defaults are the
codes of WinCNC's own posts; pass your machine's codes:
`define_machine(..., start_code="M65", torch_on_code="M61",
torch_off_code="M62", end_code="M66")`.

Tables whose own programs carry no start macro and run in the coordinates
the operator sets at the sheet (ShopSabre plasma tables as ProNest posts for
them: `G90`, each cut between `M61` and `M62`, `M66`) are declared with
`define_machine(..., start_code=None, local_coordinates="keep")`:

| Declared | Written | Premise |
| --- | --- | --- |
| `local_coordinates="keep"` | `(WORK ZERO AS SET AT THE MACHINE)` in place of `G92` | `G92` alone removes local coordinates; on a table zeroed at the sheet with them the program must not. The program then runs wherever the operator set X0 Y0 |
| `start_code=None` | `(TORCH <id>)` in place of `M65` | The table has no plasma setup macro: the torch is its only tool and the material setup is chosen at the control |

`program_height_control=True` posts `ThermalCut(height_control=True)` with
WinCNC's own commands, `L36` (Torch Height Control On) and `L37` (Torch
Height Control Off; Users Guide 3.1r5, Command Reference), as ProNest's
programs for these tables carry them: on over each loop's cut, off over
leads, small holes and slowed corners. `L37` is written before the first
torch-on, so the program does not depend on the state it starts in, and `L36`
before the end macro, so height control is left on. Without the declaration a
height control command is refused, as before.

`arc_radius_tolerance_mm` (default 0) declares the start/end radius mismatch
arc words may have. WinCNC accepts arcs within its `arc_err` setting
(`WINCNC.INI`; Users Guide 3.1r5, System Configuration: "allowable errors in
arc specifications", default .01 unit of measure); declare at most that, and
at most 0.005 mm. With it, arcs whose rounded words differ that little stay
arcs (see [posts](posts.md): `arc_word_mismatch_mm`); at 0 only exactly
consistent words are written and other arcs become checked lines, which at
four decimals of an inch is most shallow arcs.

`material_feed_mm_min` declares the "Feed Rate" of the material setup chosen at
the control (Users Guide, Plasma Torch Height Control: "XY axes feed rate for
this material"). A program with no `F` word cuts at that feed, which is how
ProNest's programs for these tables are written. With it declared no `F` word
is written, the decoder cuts at the declared feed, and a cut planned at any
other feed (a `small_holes` or corner feed) is refused, naming the feed: the
program could not say it. Left undeclared, every cutting move carries its `F`
word, which overrides the material's feed.

Both comments state the premise to the operator; the decoder reads them as
the work offset and the torch, and refuses a `G92` on such a machine.

A `ThermalMark` is refused: the post declares no marking method for WinCNC
(`M61` fires the torch with the cutting material setup).

Torch height control runs in WinCNC (arc voltage); its settings (target
volts, THC active delay, THC mode) and the pierce and cut heights are in the
material setup. Plan with `pierce_delay_s=0` and `height_control=False`: the
post refuses a dwell, and a THC command unless the machine declares
`program_height_control=True`. Arcs need exactly consistent words:
post with `LoweringOptions(native_arcs=True)`. WinCNC's velocity smoothing
(G9) must keep the path within the machine tolerance. Verify on the machine
before cutting.
