#!/bin/bash
mkdir -p ~/projects
cd ~/projects
git clone https://github.com/zchenpds/Aria.git
cd Aria
make
sudo make install
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/Aria/lib' >> ~/.bashrc
sudo ldconfig