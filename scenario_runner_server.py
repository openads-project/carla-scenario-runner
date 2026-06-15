"""
Small HTTP wrapper around ``scenario_runner.py`` for containerized setups.

The service exposes ``POST /scenario_runner`` and expects a JSON body with an
``args`` list. Those arguments are appended to:

    python scenario_runner.py --host carla-server --output

Example:

    curl -X POST -H "Content-Type: application/json" \
        -d '{"args": ["--openscenario", "/scenarios/autopilot_town10.xosc"]}' \
        http://carla-scenario-runner:4000/scenario_runner

By default the Flask server binds to all interfaces on port 4000 so it can be
reached through the container/service name ``carla-scenario-runner``.
"""

import os
from pathlib import Path
import subprocess
import sys

from flask import Flask
from flask import request

app = Flask(__name__)
SCRIPT_DIR = Path(__file__).resolve().parent


@app.route('/scenario_runner', methods=['POST'])
def trigger_script():
    try:
        request_body = request.get_json(silent=True) or {}
        script_args = request_body.get('args', [])

        if not isinstance(script_args, list) or not all(isinstance(arg, str) for arg in script_args):
            return "'args' must be a JSON list of strings", 400

        command = [
            sys.executable,
            'scenario_runner.py',
            '--host',
            'carla-server',
            '--output',
        ] + script_args

        result = subprocess.run(command, cwd=SCRIPT_DIR, capture_output=True, text=True)
        response = result.stdout
        if result.stderr:
            response += "\n" + result.stderr

    except Exception as e:
        return str(e), 500

    if result.returncode != 0:
        return response, 500

    return response, 200


if __name__ == '__main__':
    host = os.environ.get('SCENARIO_RUNNER_SERVER_HOST', '0.0.0.0')
    port = int(os.environ.get('SCENARIO_RUNNER_SERVER_PORT', '4000'))
    app.run(host=host, port=port, debug=False)
