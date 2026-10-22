import sys, os, stat, paramiko
CRED=os.path.join(os.path.dirname(os.path.abspath(__file__)),".sshcred")
conf={}
for line in open(CRED):
    if "=" in line:
        k,v=line.strip().split("=",1); conf[k]=v
remote, local = sys.argv[1], sys.argv[2]
cli=paramiko.SSHClient(); cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect(conf["HOST"],port=int(conf["PORT"]),username=conf["USER"],password=conf["PASS"],
            timeout=30,banner_timeout=120,auth_timeout=60,look_for_keys=False,allow_agent=False)
sftp=cli.open_sftp()
def get(rp,lp):
    if os.path.isdir(rp):
        os.makedirs(lp,exist_ok=True)
        for f in sftp.listdir_attr(rp):
            get(rp+"/"+f.filename, os.path.join(lp,f.filename))
    else:
        os.makedirs(os.path.dirname(lp) or ".",exist_ok=True)
        sftp.get(rp,lp); print(f"  {lp}  {os.path.getsize(lp)/1024:.0f} KB")
get(remote,local)
sftp.close(); cli.close()
