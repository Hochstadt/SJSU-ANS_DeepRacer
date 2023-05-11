# Automated Navigation System DeepRacer Control GUI

## Execution
To run the ANS GUI, the SSH Driver package must be launched with `servo_msg_publisher='cmdvel_to_servo_pkg'`.
The SSH Driver provides video, the data capture capability, and servo control. The publisher must be set to `cmdvel_to_servo_pkg` to handle the joystick velocity commands.

1. Follow the standard directions to build the dependencies using the `source_me` script in a separate terminal.
2. In a standalone terminal on the car, activate sudo privileges, source the `source_me` script and launch the SSH driver:
```
~/repos/SJSU-ANS_DeepRacer$ sudo su
~/repos/SJSU-ANS_DeepRacer$ . setup/sourceme.sh car
~/repos/SJSU-ANS_DeepRacer$ ros2 launch ssh_driver ssh_launcher.launch.py
```
3. In a terminal on the host, activate sudo privileges, source the `source_me` script, and launch the ANS GUI:
```
~/repos/SJSU-ANS_DeepRacer$ sudo su
~/repos/SJSU-ANS_DeepRacer$ . setup/sourceme.sh host
~/repos/SJSU-ANS_DeepRacer$ ros2 launch ans_gui_pkg ans_gui_launcher.launch.py
```

## PyQt
The mainwindow.ui file is created and managed in QtDesigner. After making any changes to the `ui` file, run `pyuic5 mainwindow.ui -o MainWindow.py` to regenerate an updated python representative file.