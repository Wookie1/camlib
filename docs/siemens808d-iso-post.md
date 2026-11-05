# Siemens 808D ISO milling Python reference

`examples/siemens808d_iso_post.py` defines a machine, formatter and actual-word
reader in ordinary Python. It uses the generic engine's bounded lowering,
transactional checks, attribution and streaming; there is no new Rust controller
branch. This profile is distinct from `siemens808d_post.py` (native language).

The authority is the acquired **02/2016 ISO Turning/Milling manual** (software
4.7.1/4.6.2), supplemented by the same-edition parameter and native milling manuals.
Primary programming reference: [02/2016 ISO manual](https://cache.industry.siemens.com/dl/files/339/109736339/att_880241/v1/SINUMERIK_808D_ADVANCED_ISO_022016_eng.pdf).
These editions do not establish the installed version of any particular machine.

## Use from Python

```python
import math
from camlib.motion import WorkFrame
from siemens808d_iso_post import define_machine, post_iso

# tools contains ToolSetting objects with the actual T -> ISO H mapping and
# declared tip pose after each OEM tool change. program is a camlib Program.
# camlib assumes nothing about the machine: declare it.
declared = dict(
    decimals=3,
    max_feed_mm_min=10000,
    max_spindle_rpm=12000,
    max_sweep_rad=math.tau,
    max_tool_number=99,
    max_length_register=99,
    tool_changes=True,
    tool_change_position_mode="table",
    rapid_mm_min=10000,
)
machine = define_machine(
    "Three-axis mill", tools,
    startup_units="mm", output_units="mm",
    extern_toolprog_mode=0, cutting_edge_default=0,
    tool_corr_move_mode=0, tool_length_const=17,
    tool_change="m6", coolant_codes={"off": "M9", "flood": "M8"},
    traverse_feed_mm_min=3000,
    work_to_machine=WorkFrame(origin_mm=(100, 200, 300)), **declared)
for chunk in post_iso(program, machine):
    consume_checked_nc(chunk.text, chunk.motions, chunk.data)
```

The values above illustrate a configured machine, not universal machine defaults.
The example module must be on Python's import path. `post_events_iso` accepts a
header and streamed event iterator, and accepts the generic cancellation and
lowering options. Default curves are lowered with the existing 0.0025 mm chord
allocation inside the adjustable 0.01 mm source-to-output budget. `curve_policy=
"exact_ijk"` emits G17 circles and helices up to one revolution when representable
within that same budget, with zero radius-repair allowance. Other-plane and
multiturn paths can use bounded line lowering. No canned drilling cycle is emitted.

## Semantics and setup premises

| Behavior | Manual evidence and implementation |
| --- | --- |
| Language/state | ISO p8: standalone G290/G291 preserve tools, lengths and work offsets. Startup enters native mode, disables compression and modal rounding, selects cutter-center feed and the fine stop window, then enters ISO mode. Native prep uses the 02/2016 native manual's COMPOF, RNDM=0, CFTCP and G601. No mixed-language blocks are accepted. |
| Units | ISO p110: G20/G21 are standalone; changing units requires additional frame/offset handling. This profile emits the already active declared units and rejects a runtime change. Work/tool tables and the declared pose are supplied in mm through the generic API. |
| Numeric notation | ISO pp9–11: MD10884 and MD10886 change undotted numbers. All dimensional XYZ/IJK/F words include a decimal, at up to three metric/four inch decimal places, so both calculator/standard and IS-B/C interpretations agree. |
| ISO milling length | ISO pp114–116: H selects the global `$TC_DPH[t,d]` map, G43 adds length, G49 cancels. Parameter manual MD10890 pp50–51 explicitly scopes bit2's H/D selection to **turning**; the general ISO p8 note is not used to invent a milling D mode. Bits2/6 configurations are recorded; radius compensation remains G40. |
| Offset traversal | MD20270=0 means no implicit cutting edge after T; MD20382=0 means offset traversal requires the corresponding axis to be programmed. SD42940=17 binds length to Z. Thus the standalone G43 H and G49 blocks do not introduce an unreported axis move. Other settings need a different decoder/motion model. |
| Tool changes | Explicit OEM direct-T or T/M6 choice. H must agree with the supplied table. Post-change physical tip poses are supplied by the caller, as for the existing generic engine; OEM tool-change routes are not simulated by this reference. |
| Work frames | ISO pp104–113: G50/G69/G50.1 cancel scale, rotation and mirroring before G92.1 X0. Y0. Z0. resets all three G92/G52 shifts. Bare G92.1 is insufficient. Stored G54–G59 offsets must match the supplied Cartesian translation. |
| Traverse | ISO pp95–96: G0 can move axes independently unless MD20732 enables interpolation. This reference emits explicitly fed G1 traverses and checks their actual paths. It does not guess MD20732. |
| Circles/helix | ISO pp97–100: G17, G2/G3, signed incremental I/J, absolute endpoints, explicit F; one-turn arcs/helix, no native TURN syntax, radius R or contour chamfer/rounding words. |
| Stop/feed/spindle | ISO pp122–125: G94, explicit S with M3/M4, G61. Coolant codes are declared OEM functions. Tool changes and stops require spindle/coolant already off; optional stop requires declared enablement. Shutdown explicitly stops spindle/coolant and cancels length before M30. |

Acquired manuals and locally checked interpretation are separate from external
interpreter and physical machine qualification. Neither external nor physical
qualification is claimed. Workholding, OEM routes, persistent external/handwheel
shifts and calibration remain supplied machine/setup premises. This profile does
not add general five-axis, probing, macros, transformations, cycles or machine
return commands.

## Evidence

`python/tests/test_siemens808d_iso_post.py` exercises unit/notation combinations,
nonzero stored offsets, T/M6 and direct-T, exact arcs/full circles/helices, bounded
all-plane/multiturn lowering, fed traverses, cancellation/attribution, hostile
modal edits and atomic retry. `siemens808d_iso_oracle.py` independently interprets
literal NC without importing the post, dialect or decoder; its integer-word
scaling follows the milling table and can detect omitted decimal points.
A complete pocket runs through native planning, this post, independent literal
interpretation, continuous scene/contact checks and actual posted stock removal.
These are local nominal checks, not external interpreter or machine qualification.
See [postprocessor architecture](posts.md) for workflow references.
