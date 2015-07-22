import subprocess

subprocess.Popen('avconv -an -f video4linux2 -s 560x480  -r 15 -i /dev/video0 -timelimit 10 recordBRM.avi', shell = True)
