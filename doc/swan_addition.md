# Additional Stuff for Beaglebone Ubuntu
## 1. Update and upgrade
```
sudo apt update && sudo apt -y upgrade
```
## 2. screen, htop, vim
By default `rosed` uses vim.
```
sudo apt -y install screen htop vim
```
### 3.  Cool login prompt
```
cd ~/setup/
sudo cp issue.net /etc/issue.net
```
