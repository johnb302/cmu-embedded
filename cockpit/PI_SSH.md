# Pi ssh access

- User `team5`, password `team5`.
- Host `team5.local`, or the IP 172.26.58.163 (campus wifi) when mDNS doesn't resolve, e.g. on the Linux laptop.
- Power: USB-C from the laptop works. Network is wifi; no ethernet needed.

## New machine, one time

```
# One key only: offering every key in ~/.ssh hits the Pi's auth-try limit and it drops the connection.
ssh-copy-id -i ~/.ssh/id_rsa.pub -o IdentitiesOnly=yes team5@172.26.58.163   # enter the password once
ssh team5@172.26.58.163 hostname       # should log in without asking
```

If you connect by IP, `cockpit/deploy.sh` needs the host spelled out: `cockpit/deploy.sh team5@172.26.58.163`.
A key added under `team5.local` isn't trusted under the IP: add `-o HostKeyAlias=team5.local`, or accept the new host key once.

## Console

- Start: `~/lab5/cockpit/run_cockpit.sh`. Log: `~/lab5/cockpit/cockpit.log`.
- Command: `echo "raft log" > ~/lab5/cockpit/cmd.fifo`; the reply is in the log.
