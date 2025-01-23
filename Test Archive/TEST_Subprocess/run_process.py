import subprocess

# Paths to the main scripts
script1 = "./TEST_Subprocess/script1.py"
script2 = "./TEST_Subprocess/script2.py"

try:
    # Run the first script (PPGUI/main.py)
    print("Running script1.py...")
    subprocess.run(["python", script1], check=True)
    print("script1.py completed successfully.")

    # Run the second script (PPFLY/main.py)
    print("Running script2.py...")
    subprocess.run(["python", script2], check=True)
    print("script2.py completed successfully.")

except subprocess.CalledProcessError as e:
    print(f"An error occurred: {e}")