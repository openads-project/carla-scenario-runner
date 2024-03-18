
import subprocess
import os


def main():
    print("Starting")
    folder = "/tmp/scenario-center_simulations"
    files = [file_cand for file_cand in os.listdir(folder) if file_cand.endswith("xosc")]
    
    for file in files:
        try:
            subprocess.run(["python", "scenario_runner.py", "--host", "carla-server",
                "--openscenario", "/tmp/scenario-center_simulations/" + file,
                "--record", "/tmp/log_files"], check=True)
            print("Passed file " + str(file))
        except:
            print("Failed for file: " + str(file))
            
            
if __name__ == "__main__":
    main()