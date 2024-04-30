import subprocess

# Run pigpiod
subprocess.run(['sudo', 'pigpiod'], shell=True)

# Change directory
subprocess.run(['cd', 'horizon_detector'], shell=True)

# Run autoupdater.py
subprocess.run(['python3', 'autoupdater.py'], shell=True)

# Run main.py
subprocess.run(['python3', 'main.py'], shell=True)