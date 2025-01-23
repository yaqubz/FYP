# WORKS 23 Jan - Run this from the main directory to call the scripts, similar to a .bat file

import subprocess

# Paths to the main scripts
ppgui_main = "./PPGUI/main.py"
ppfly_main = "./PPFLY/main.py"

try:
    # Run the first script (PPGUI/main.py)
    print("Running PPGUI main.py...")
    subprocess.run(["python", ppgui_main], check=True)
    print("PPGUI main.py completed successfully.")

    # Run the second script (PPFLY/main.py)
    print("Running PPFLY main.py...")
    subprocess.run(["python", ppfly_main], check=True)
    print("PPFLY main.py completed successfully.")

except subprocess.CalledProcessError as e:
    print(f"An error occurred: {e}")