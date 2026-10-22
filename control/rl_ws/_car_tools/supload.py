#!/usr/bin/env python3
"""上传文件/目录到服务器（SFTP）。

用法:
    python supload.py <本地路径> <远端路径>
"""
import sys
import os
import stat
import paramiko

CRED = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".sshcred")
conf = {}
for line in open(CRED):
    if "=" in line:
        k, v = line.strip().split("=", 1)
        conf[k] = v

local, remote = sys.argv[1], sys.argv[2]

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect(conf["HOST"], port=int(conf["PORT"]), username=conf["USER"],
            password=conf["PASS"], timeout=30,
            banner_timeout=120, auth_timeout=60,
            look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()

total_files = 0
total_bytes = 0


def ensure_dir(path):
    parts = path.rstrip("/").split("/")
    cur = ""
    for p in parts:
        if not p:
            cur = "/"
            continue
        cur = cur.rstrip("/") + "/" + p
        try:
            sftp.stat(cur)
        except IOError:
            sftp.mkdir(cur)


def put_file(lp, rp):
    global total_files, total_bytes
    ensure_dir(os.path.dirname(rp))
    sftp.put(lp, rp)
    sz = os.path.getsize(lp)
    total_files += 1
    total_bytes += sz
    if total_files % 200 == 0:
        print(f"  ... {total_files} 文件, {total_bytes/1024**2:.1f} MB", flush=True)


if os.path.isfile(local):
    put_file(local, remote)
else:
    base = os.path.basename(local.rstrip("/"))
    for root, dirs, files in os.walk(local):
        rel = os.path.relpath(root, local)
        rdir = remote if rel == "." else os.path.join(remote, rel)
        ensure_dir(rdir)
        for f in files:
            put_file(os.path.join(root, f), os.path.join(rdir, f))

print(f"完成: {total_files} 个文件, {total_bytes/1024**2:.1f} MB")
sftp.close()
cli.close()
