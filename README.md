## Drone Control
PX4 dronunu offboard modunda kullanımı. 

Yazılımın daha iyi takip edilip geliştirmek için `gdbserver` üzerinden 
VSCode içinde debug yaparak ilerlenebilir. Bu sayede hataların hangi satırlarda yapıldığı veya kodun nerelere gitdiği incelenebilir.

### Kurulum

1. PX4 Autoplot:
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
2. Paket Kurulumu
```
git clone git@github.com:serkanMzlm/Drone_Control.git
cd Drone_Control
git submodule init
git submodule update
```

### Build
- İlk olarak 'px4_msgs' paketi build edilip sisteme dahil edilmelidir.
```
colcon build --packages-select px4_msgs
. install/setup.bash
```
```
colcon build --packages-select command
. install/setup.bash
```

### Run
- Terminal 1:
```
make px4_sitl gz_x500  # PX4-Autoplot Dizininde
```

- Terminal 2:
```
MicroXRCEAgent udp4 -p 8888
```
- Terminal 3:
```
ros2 run joy joy_node
```
- Terminal 4:
```
ros2 run command command_node
```


### Oluşabilecek Sorunlar
1.  "PX4-Autopilot" yazılımının eski versiyonu kullanılıyorsa `ros2 topic echo /fmu/out/vehicle_local_position
` komutunun çalışmaması. Yazılımı güncelleyin
