import subprocess

def call_cpp_program():
    result = subprocess.run(["./robot_arm_ik"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if result.returncode == 0:
        print(result.stdout)
    else:
        print(result.stderr)

if __name__ == "__main__":
    call_cpp_program()
