import subprocess

scene_file = "scene.ttt"
command = f'runcoppelia {scene_file}'

subprocess.run(command, shell=True)