# LinuxCNC QtPlasmaC post

[`examples/qtplasmac_post.py`](../examples/qtplasmac_post.py) writes
`ThermalCut` programs for LinuxCNC 2.9 QtPlasmaC and reads its own output back
for checking. Reference: the LinuxCNC 2.9 QtPlasmaC documentation (G-code
conventions, THC codes, G-code filter).

```python
import math
import qtplasmac_post as q

torch = camlib.Torch("torch", kerf_width=1.5)
setting = ToolSetting.for_tool(torch, number=1, length_register=1, position_after_mm=start)
# camlib assumes nothing about the machine: declare it.
declared = dict(
    output_units="mm",
    decimals=3,
    max_feed_mm_min=15000,
    max_spindle_rpm=1,
    planes=("xy",),
    arcs=True,
    helices=False,
    max_sweep_rad=math.tau,
    max_tool_number=1,
    max_length_register=1,
    tool_changes=True,
    rapid_mm_min=15000,
)
machine = q.define_machine("Plasma table", setting, blend_mm=0.005, **declared)
program = job.post_nc("sheet", machine=machine, post=q.QtPlasmaCPost, decoder=q.QtPlasmaCDecoder,
                      lowering=camlib.LoweringOptions(native_arcs=True))
```

| Item | Output | Premise |
| --- | --- | --- |
| Preamble | `G21 G90 G17 G94 G40 G49 G80 G92.1`, `G64 P<blend_mm>`, `M52 P1` | The documentation requires `M52 P1` (paused motion). `G64 P` is written explicitly (the loader would insert `G64 P0.1`); `blend_mm` must be at most half the machine tolerance |
| Torch | `T0 M6`, `G43 H0` | QtPlasmaC tool 0 is the torch (T1 is the scribe); camlib's table entry is number 1, register 1 |
| Torch on / off | `M3 $0 S1` / `M5 $0` | |
| Height control | `M62 P2` off, `M63 P2` on (synchronized with motion) | `ThermalCut(height_control=True)` |
| Z | One Z (the cut height) | QtPlasmaC removes Z motion and handles probing, pierce height, pierce delay and cut height from the material file: plan with `height_mode="fixed"`, `pierce_delay_s=0` |
| Feeds | `F` words | QtPlasmaC warns when they differ from the material's cut feed rate |

Not written: material selection (`M190`/`M66`), velocity percentages (`M67`),
hole sensing and QtPlasmaC's own overcut (camlib plans small-hole feeds and
overburn itself), cutter compensation `G41.1`/`G42.1` (QtPlasmaC disables THC
and overcut with it). Verify on the machine before cutting.
