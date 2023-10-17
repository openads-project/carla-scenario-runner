from flask import Flask
from flask import request
import subprocess

app = Flask(__name__)

@app.route('/scenario_runner', methods=['POST'])
def trigger_script():
    try:
        # Get arguments from the request
        script_args = request.json.get('args', [])

        # Build the command to run the script with arguments
        command = ['python', 'scenario_runner.py', '--host', 'carla-server', '--output'] + script_args

        # Run your existing Python script here
        result = subprocess.run(command, capture_output=True, text=True)
        
        # You can send the result back as a response
        return result.stdout, 200

    except Exception as e:
        return str(e), 500

if __name__ == '__main__':
    app.run(host='carla-scenario-runner', port=4000, debug=True) # server can be reached on 'carla-scenario-runner' and port 4000

# Examplaric server call:
#   curl -X POST -H "Content-Type: application/json" -d '{"args": ["--openscenario", "/scenarios/autopilot_town10.xosc"]}' http://carla-scenario-runner:4000/scenario_runner