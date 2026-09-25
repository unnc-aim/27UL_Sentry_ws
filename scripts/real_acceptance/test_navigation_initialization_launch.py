#!/usr/bin/env python3
"""Check navigation initialization options by inspecting launch actions."""
from pathlib import Path
import os
import runpy
from types import SimpleNamespace
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / "src/pb2025_sentry_nav/pb2025_nav_bringup/launch/rm_navigation_reality_launch.py"


def main():
    generate = runpy.run_path(str(LAUNCH))["generate_launch_description"]
    with patch.dict(
        generate.__globals__,
        os=SimpleNamespace(path=os.path, makedirs=lambda *args, **kwargs: None),
    ):
        description = generate()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations["auto_global_localization"] == "True"
    group, = (action for action in description.entities if isinstance(action, GroupAction))
    node, = (action for action in group.get_sub_entities() if isinstance(action, Node))
    for slam, enabled, expected in (
        ("False", "True", True),
        ("False", "False", False),
        ("True", "True", False),
        ("True", "False", False),
    ):
        context.launch_configurations.update(slam=slam, auto_global_localization=enabled)
        selected = group.condition.evaluate(context) and node.condition.evaluate(context)
        assert selected == expected, (slam, enabled)
    context.launch_configurations["log_dir"] = "/tmp/sentry-navigation-check"
    command = [perform_substitutions(context, item) for item in node.cmd]
    assert command[:5] == [
        "/usr/bin/python3", str(ROOT / "scripts/real_acceptance/localization_probe.py"),
        "--globalize", "--seconds", "15",
    ], command
    assert command[command.index("--output") + 1] == "/tmp/sentry-navigation-check/localization.json"
    print("PASS: default initialization, mapping exclusion, explicit disable, 15 seconds and report path")


if __name__ == "__main__":
    main()
