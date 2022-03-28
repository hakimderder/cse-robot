# Wine installation - Ubuntu 18.04 bionic

This tutorial is based on :  [winehq](https://wiki.winehq.org/Ubuntu)

And this solution : [askubuntu Q&A](https://askubuntu.com/questions/1100351/broken-packages-fix-problem-for-wine)


## Step by step


Run this command to add intel architecture to dpkg :
```bash
sudo dpkg --add-architecture i386 
```

Run these commands to add wine's key :
```bash
wget -nc https://dl.winehq.org/wine-builds/winehq.key
sudo apt-key add winehq.key
```

Run this command to add wine's repository :
```bash
sudo add-apt-repository 'deb https://dl.winehq.org/wine-builds/ubuntu/ bionic main' 
```

Run these commands to libfaudio dependency :
```bash
wget https://download.opensuse.org/repositories/Emulators:/Wine:/Debian/xUbuntu_18.04/amd64/libfaudio0_19.07-0~bionic_amd64.deb
wget https://download.opensuse.org/repositories/Emulators:/Wine:/Debian/xUbuntu_18.04/i386/libfaudio0_19.07-0~bionic_i386.deb
sudo dpkg -i libfaudio0_19.07-0~bionic_amd64.deb libfaudio0_19.07-0~bionic_i386.deb
```

Run this command to update your modifications :
```bash
sudo apt update
```

Run this command to finally install the package :
```bash
sudo apt install --install-recommends winehq-stable
```


