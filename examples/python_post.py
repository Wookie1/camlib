"""An ordinary Python machine definition and customizable postprocessor."""

from camlib.post import GCodePost, Machine, ToolSetting, post_program


class MyShopPost(GCodePost):
    """Customize this file or import any ordinary Python post class you write."""

    version = 'my-shop-1'

    def on_tool_change(self, event):
        self.comment(f'Tool {event.number}, length register {event.length_register}')
        super().on_tool_change(event)

    def on_motion(self, motion):
        # Normal Python conditionals, formatting and reusable helpers.
        if motion.kind == 'rapid':
            self.comment('Position above the work')
        super().on_motion(motion)


def defined_machine(program):
    """Minimal standalone machine definition for another authored program.

    These table/pose values are explicit premises, not read from a real machine.
    Other programs may use only a subset of the configured machine tool library.
    """
    return Machine(
        'My three axis mill',
        tools=(
            ToolSetting.for_tool(
                program.tools[0], number=1, length_register=1, position_after_mm=program.initial_position_mm
            ),
        ),
        max_feed_mm_min=8000,
        max_spindle_rpm=24000,
        tip_travel_mm=((-100, -100, -100), (100, 100, 100)),
    )


def program_chunks(program):
    return post_program(program, machine=defined_machine(program), post=MyShopPost)
