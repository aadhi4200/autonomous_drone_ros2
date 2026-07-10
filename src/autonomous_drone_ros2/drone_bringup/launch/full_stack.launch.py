"""full_stack.launch.py — one-command PX4 SITL + MAVROS + camera bridge +
mission nodes bring-up, with event-handler-based readiness gates between
stages instead of fixed timers.

A fixed-delay version (TimerAction at t=25s/35s/40s) does not verify the
previous stage actually succeeded — if PX4 happens to boot slower on a given
run, MAVROS would still fire on schedule and silently fail to connect (the
same QoS-adjacent silent-failure pattern documented elsewhere in this
project). This file watches each stage's actual stdout for the same
readiness strings already proven reliable in launch_full_sim.sh, the bash
reference this is meant to match in reliability:

  1. PX4 SITL + Gazebo -> stdout contains "Startup script returned successfully"
  2. MAVROS            -> stdout contains "Got HEARTBEAT" and "connected"
                           on the same line
  3. full_mission.launch.py (camera bridge + all 5 mission nodes, already
     bundled there per its own docstring) -- included once MAVROS is
     confirmed connected.

Camera-bridge/mission-node startup isn't gated stage-by-stage here: per
CLAUDE.md section 13.3's "acceptable to keep simple" allowance, the project's
own bash reference found log-string matching unreliable for the camera
bridge (it polls `ros2 topic list` instead) and has no single "all ready"
line across the 6 mission nodes at all. Neither has the hard "silently never
connects" failure mode PX4->MAVROS timing does, so they start together via
the existing full_mission.launch.py, unmodified — reusing it rather than
duplicating its node definitions, per section 13.1.

Stage 1 (PX4 SITL + Gazebo) process construction lives in px4_stack.py,
shared with the standalone px4_sitl.launch.py — run that one on its own if
you want to bring up just PX4+Gazebo and watch/drive the rest of the stack
yourself terminal-by-terminal (see README.md), rather than everything
chained automatically through this file. See px4_stack.py's docstring for
the PX4 boot-mechanism details (why it's two processes, not one
`make px4_sitl gz_<model>` call, on this PX4 version).
"""
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from drone_bringup import px4_stack

PX4_READY_RE = re.compile(r"Startup script returned successfully")
MAVROS_READY_RE = re.compile(r"Got HEARTBEAT.*connected")

PX4_READY_TIMEOUT_S = 90.0
MAVROS_READY_TIMEOUT_S = 60.0


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    headless = LaunchConfiguration('headless')
    gz_world = LaunchConfiguration('gz_world')
    fcu_url = LaunchConfiguration('fcu_url')

    bringup_share = get_package_share_directory('drone_bringup')

    # ── Stage 1 — PX4 SITL + Gazebo (shared with px4_sitl.launch.py) ─────
    gazebo_process = px4_stack.make_gazebo_process(gz_world, headless)
    px4_process = px4_stack.make_px4_process(gz_world)

    def on_gazebo_exit(event, context):
        if not px4_ready['fired']:
            return [LogInfo(msg=f'⚠️ Gazebo process exited early (code={event.returncode}) '
                                 f'— PX4 will likely hang waiting for it.')]
        return None

    gazebo_exit_handler = RegisterEventHandler(
        OnProcessExit(target_action=gazebo_process, on_exit=on_gazebo_exit))

    # ── Stage 2 — MAVROS ──────────────────────────────────────────────────
    mavros_process = ExecuteProcess(
        cmd=['ros2', 'launch', 'mavros', 'px4.launch', ['fcu_url:=', fcu_url]],
        output='screen',
    )

    # ── Stage 3 — camera bridge + all mission nodes (existing file, per
    # section 13.1's explicit "don't duplicate those node definitions") ──
    full_mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'full_mission.launch.py')),
        launch_arguments={'mode': mode}.items(),
    )

    # ── Event-handler chain: PX4 ready -> start MAVROS ───────────────────
    px4_ready = {'fired': False}
    px4_stdout = {'text': ''}

    def on_px4_stdout(event):
        px4_stdout['text'] += event.text.decode(errors='replace')
        if px4_ready['fired']:
            return None
        if PX4_READY_RE.search(px4_stdout['text']):
            px4_ready['fired'] = True
            return [LogInfo(msg='✅ PX4 SITL ready — starting MAVROS'),
                    mavros_process, mavros_ready_handler, mavros_timeout]
        return None

    def px4_timeout_check(context):
        if px4_ready['fired']:
            return []
        return [LogInfo(msg=f'❌ PX4 SITL did not report readiness within '
                             f'{PX4_READY_TIMEOUT_S:.0f}s — aborting.'),
                Shutdown(reason='PX4 SITL startup timeout')]

    px4_ready_handler = RegisterEventHandler(
        OnProcessIO(target_action=px4_process, on_stdout=on_px4_stdout))
    px4_timeout = TimerAction(period=PX4_READY_TIMEOUT_S,
                              actions=[OpaqueFunction(function=px4_timeout_check)])

    # ── Event-handler chain: MAVROS ready (connected) -> start mission stack ─
    mavros_ready = {'fired': False}
    mavros_stdout = {'text': ''}

    def on_mavros_stdout(event):
        mavros_stdout['text'] += event.text.decode(errors='replace')
        if mavros_ready['fired']:
            return None
        if MAVROS_READY_RE.search(mavros_stdout['text']):
            mavros_ready['fired'] = True
            return [LogInfo(msg='✅ MAVROS connected — starting camera bridge + mission nodes'),
                    full_mission]
        return None

    def mavros_timeout_check(context):
        if mavros_ready['fired']:
            return []
        return [LogInfo(msg=f'❌ MAVROS did not report a connected HEARTBEAT within '
                             f'{MAVROS_READY_TIMEOUT_S:.0f}s — aborting.'),
                Shutdown(reason='MAVROS connect timeout')]

    mavros_ready_handler = RegisterEventHandler(
        OnProcessIO(target_action=mavros_process, on_stdout=on_mavros_stdout))
    mavros_timeout = TimerAction(period=MAVROS_READY_TIMEOUT_S,
                                 actions=[OpaqueFunction(function=mavros_timeout_check)])

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim',
                              description="'sim' or 'hardware' — forwarded to full_mission.launch.py"),
        DeclareLaunchArgument('headless', default_value='0',
                              description='1 = no Gazebo GUI window (lighter/faster; use if '
                                           'MAVROS timesync/RTT issues reappear under load)'),
        DeclareLaunchArgument('gz_world', default_value='aruco_landing'),
        DeclareLaunchArgument('fcu_url', default_value='udp://:14540@127.0.0.1:14580'),

        gazebo_process,
        gazebo_exit_handler,
        px4_process,
        px4_ready_handler,
        px4_timeout,
        # mavros_ready_handler is registered dynamically by on_px4_stdout once
        # PX4 is actually ready, not here — that's the whole point of gating.
    ])
