# SINUMERIK808D ADVANCED native reference post

`examples/siemens808d_post.py` is an ordinary Python machine definition, post class,
decoder and iterator adapter using the generic camlib engine. No Rust controller
registry/branch or copied upstream post code. Target reference: **native milling
software4.90**,07/2018 manual6FC5398-4DP10-0BA7. This is a documented local subset,
not ISO mode, all Siemens software, external firmware conformance or qualification
for any physical machine whose installed version and builder configuration have
not been established.

## Define and post in Python

```python
import math
from camlib.motion import WorkFrame
from camlib.post import ToolSetting
from siemens808d_post import define_machine, post_808d

# camlib assumes nothing about the machine: declare it.
declared = dict(
    output_units="mm",
    decimals=3,
    max_sweep_rad=math.tau,
    tool_changes=True,
    tool_change_position_mode="table",
    rapid_mm_min=10000,
)
machine = define_machine(
    'My808D setup',
    tools=(ToolSetting.for_tool(tool, number=2, length_register=1,
                               position_after_mm=known_tool_change_tip_pose),),
    tool_change='m6',                 # or 'direct_t': match actual machine data
    coolant_codes={'off': 'M9', 'flood': 'M8'},  # actual OEM PLC meanings required
    work_to_machine=WorkFrame(origin_mm=known_base_and_work_offset),
    frame=program.frame,
    max_feed_mm_min=8000,
    max_spindle_rpm=12000, **declared)
for chunk in post_808d(program, machine):
    nc_file.write(chunk.text)
    # Checked decoded motions and original source/fraction/NC mappings.
    simulation.consume(chunk.motions)
```

Use `PYTHONPATH=python:examples` to import this example module. The program's
ToolChange numbers/edge IDs/poses must match the selected machine bindings;
there is no inference/remapping of hardware tool tables. Supply a ToolAssembly
for each cutting-edge geometry/length used in this job and a ToolSetting for
its `(T,D)` pair. Multiple edges may share T; D1 may repeat across different T.
Up to64configured fields, T1..32000,D1..9 in this reference subset. Normalized
programmed endpoints/incremental centers and feed/RPM limits are bounded to99,999.999
(mm,mm/min,RPM) for this subset; actual machine travel/speed limits can be lower.
These software bounds are independently checked on output, not assumed from formatting. Actual stored
calibrations/geometry, base/stored frame and post-selection poses are premises.

The engine's generic per-tool-edge support now accepts distinct `(T,D)` entries,
D-only edge reselection and bundled M6+Dn. Explicit-H dialects retain their global
register/unique-T rules. Source/tool/edge identity and native state still validate.
This is useful to any Python dialect with per-tool edge semantics, not a Siemens
branch. Example post emits `Tn Dd` for direct activation or `Tn` then `M6 Dd`
for preselection/M6. Same-T changes emit only `Dd`. Explicit coactivation avoids
assuming a valid D1 entry when selecting another edge. The decoder tracks implicit
D1 on activation but requires an explicit edge before machining in this subset.

## Supported subset and checks

Startup explicitly selects G290 in its own block; G710/G700 for coordinate **and
feed** units, G90,G17,G94,G40,D0,MCALL,TRANS,COMPOF,G60 with G601/G602,RNDM=0 and
CFTCP. M5 and the caller's OEM coolant-off code are explicit. Shutdown stops
spindle/coolant, cancels D and ends with M2. Native S is RPM and unqualified I/J/K
are relative to the arc start; these native invariants replace LinuxCNC-only
G97/G91.1. No G20/G21/G43/G49/G80/G92.1/G61.1 aliases are emitted as Siemens codes.

The declared length/cutter axis stays vertical/G17. G18/G19 are rejected because
they change the tool-length axis, not merely arc formatting. Initial/reset base
frames, kinematic transforms/couplings and any OEM tool-change macro effects must
match the fixed Cartesian setup; TRANS clears programmable frame components, not
an unknown base frame. The supplied OEM tool-change contract must preserve these initialized modes and
frames (or a custom post/decoder must model and restore them). Tool-change routes
and servo motion are not inferred from endpoint/table values. G60/windows describe nominal stop behavior; real window
sizes are machine data, not zero-error part accuracy or machine-cycle-time proof.

Default `curve_policy='linear'` runs the existing native bounded curve lowering
through ordinary Python hooks. All source planes, circles, helices and multiturn
curves are retained as attributed G1 segments; no G18/G19 switch for the vertical
tool. At most64segments/512KiB per chunk, one pending source, cancellation/rollback,
source hashes, separate posted IDs and continuous source-interval error bounds.
Defaults remain0.01mm total post deviation/0.0025mm chord budget and unit-selected
formatting. Stock allowance is an explicit separate planner input.

`curve_policy='exact_ijk'` opts into native G17 circles and <=one-turn helices,
with explicit endpoints and incremental I/J. Helices include `TURN=0` (zero
additional revolutions). Radii must match **exactly in the declared binary64
model** after output quantization. The manual says accepted mismatched circles
have their center adjusted internally, but does not specify an algorithm here;
the example must not pretend the uncorrected circle is the actual result. Such
cases reject and should use bounded linear policy. This zero-mismatch structural
check is not a request for submicron machine accuracy. `CR`, `AR`, `AC/IC`, `CT`,
`CIP`, extra TURN counts, transforms, blending/compression/rounding, cycles and ISO
syntax remain unsupported. Native center-repair/blend models and external/machine conformance are outside
this reference profile.

`post_events_808d(header, iterator, machine)` supports a live source. To use the
core API directly, pass `post=Siemens808DPost`, `decoder=Siemens808DDecoder`, and
`lowering=LoweringOptions()` for the linear policy. Machine/post/decoder choices
and revisions are included in run definitions; this is not attestation of arbitrary
Python dependencies. Custom Python posts remain trusted executable code.

## References and local checks

The primary reference is the [07/2018 native milling manual](https://cache.industry.siemens.com/dl/files/981/109759981/att_959788/v1/808D_ADV_milling_op_man_0718_en-US.pdf),
6FC5398-4DP10-0BA7, software 4.90. Printed/PDF pages coincide: 79–81 planes and
units; 83–88 frames; 90–99 interpolation and helix; 105–106 stop windows;
114–117 tool/edge activation and length axes; 123–124/338–339 builder M functions;
133 modal call cancellation; 337 language switching. The [02/2016 ISO manual](https://cache.industry.siemens.com/dl/files/339/109736339/att_880241/v1/SINUMERIK_808D_ADVANCED_ISO_022016_eng.pdf)
page 8 additionally documents standalone language switching and preserved state.
That older ISO edition does not establish native 4.90 semantics.

`python/tests/test_siemens808d_post.py` and `python/tests/siemens808d_oracle.py`
check literal output, units/frames/tool edges, bounded lowering, cancellation,
hostile modal changes and actual stock. `fixtures/posts/EXACT490.MPF` is a
literal native quarter/full-circle/helix fixture. The oracle imports motion
contracts without the post/dialect/decoder implementation. Machine data, OEM
routes, physical stop windows, later firmware and external qualification remain
separate requirements. See [postprocessor architecture](posts.md) for workflow
references and [ISO profile](siemens808d-iso-post.md) for the separate language.
