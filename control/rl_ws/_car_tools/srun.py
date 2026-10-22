#!/usr/bin/env python3
"""在远端服务器执行命令。

用法:
    python srun.py "命令"
    python srun.py --timeout 600 "长命令"
"""
import sys
import os
import paramiko

CRED = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".sshcred")
conf = {}
for line in open(CRED):
    if "=" in line:
        k, v = line.strip().split("=", 1)
        conf[k] = v

timeout = 300
args = sys.argv[1:]
if args and args[0] == "--timeout":
    timeout = int(args[1])
    args = args[2:]
cmd = " ".join(args)

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect(conf["HOST"], port=int(conf["PORT"]), username=conf["USER"],
            password=conf["PASS"], timeout=30,
            banner_timeout=60, auth_timeout=60,
            look_for_keys=False, allow_agent=False)

stdin, stdout, stderr = cli.exec_command(cmd, timeout=timeout, get_pty=False)
out = stdout.read().decode("utf-8", "replace")
err = stderr.read().decode("utf-8", "replace")
rc = stdout.channel.recv_exit_status()
cli.close()

if out:
    print(out, end="")
if err:
    print("--- STDERR ---", file=sys.stderr)
    print(err, end="", file=sys.stderr)
sys.exit(rc)
