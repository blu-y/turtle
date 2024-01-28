```
sudo apt-get install -y ntp 

server 203.248.240.140 prefer iburst
# server 1.kr.pool.ntp.org prefer iburst minpoll 4 maxpoll 6

sudo systemctl start ntp 
sudo systemctl status ntp
date
sudo timedatectl set-timezone Asia/Seoul
```