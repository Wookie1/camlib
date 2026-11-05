"""Synthetic CAM regression input."""

from dataclasses import replace

from cadquery_job import build_rest_job, rest_machine_for
from camlib.dynamics import MotionPolicy
from camlib.posted import OperationEvents
from camlib.simulation import Simulation


def scheduled_job():
    job = build_rest_job()
    setup = job.compile().setup('top')
    machine = replace(
        rest_machine_for(job),
        motion_policy=MotionPolicy(tangential_accel_mm_s2=200, rapid_feed_mm_min=6000),
        max_normal_accel_mm_s2=100,
        max_axis_feed_mm_min=(3000, 3000, 1500),
    )
    with Simulation(**setup.simulation_options()) as simulation:
        source = OperationEvents(simulation, reuse_active_tool=True)
        result = machine.schedule_events(setup.program_header(), source)
    return result, machine

    # Use post_program(result.program, machine) for independently checked output.
