# Multi-Terminal
A simple program designed to facilitate the repetitive writing of SSH connection commands and used commands. I created this project because I felt lazy to open terminals one by one, establish SSH connections, and type the desired commands for each target. Sometimes, this can be quite annoying when done repeatedly, especially during the drone trials I conduct. This program will load several terminals (Virtual Terminal Emulators) into one application. In addition, it establishes SSH connections and executes the desired commands with just a button press. This program is quite useful for me, especially during autonomous drone trials in outdoor environments, where sunlight interferes with our visibility of the monitor screen, and we don't have a good position to type in an outdoor setting.

## Diagram
![image](https://github.com/Balisa16/Multi-Terminal/assets/69964801/7096d361-6573-4cc8-8158-64b1e3da2eaa)


## Dependencies
1. JSONCPP Library
```
git clone https://github.com/open-source-parsers/jsoncpp.git
cd jsoncpp && mkdir build
cd build && cmake ..
make -j4
sudo make install
```
2. VTE (Virtual Terminal Emulator)
```
sudo apt-get install libvte-2.91-dev -y
```
3. GTK3
```
sudo apt-get install libgtk-3-dev -y
dpkg -l libgtk* | grep -e '^i' | grep -e 'libgtk-*[0-9]'
```
## Install
### Build Code
```
git clone https://github.com/Balisa16/Multi-Terminal.git multi-terminal
cd multi-terminal
mkdir build && cd build
cmake .. && make -j4
```
### Add JSON File Directory
```
echo 'export MT_PATH="'"$(pwd)"'"' >> ~/.bashrc
source ~/.bashrc
```
### Run Program
```
cp ../run.sh ~/run.sh
cd ~ && ./run.sh
```

## Noted
Please recheck your ssh target **bash** path with command :
```
which bash
```
And change file *src/main.cpp* in the line **153** with your ssh target bash path. Do it after clone and before build program.
