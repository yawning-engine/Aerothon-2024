Run these commands in ubuntu
dronekit-sitl copter
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out 127.0.0.1:14552

connect to simulator(mission planner) on windows with TCP, Ip- 127.0.0.1, port-5763

open wsl in VS code/any other and run - python3 simulation.py --connect udp:127.0.0.1:14550