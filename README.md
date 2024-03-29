An Indoor Autonomous Navigation System Implementation using AWS DeepRacer

**SJSU / LM Cohort 2023 Master's Project -- Group 4**

# To setup ssh_driver/controller:
## On the car/deepracer, ssh_driver:
0. To get to the car you can do a number of things, like directly
logging in (through monitor/keyboard/mouse), vncserver setup like
nomachine, or ssh. The best way to operate is using ssh only as the processor
of the deepracer is tiny and opening a vncserver significantly delays the
image streaming. Additionally, a monitor/keyboard/mouse direct setup
won't allow you to drive the car very far! My suggestion is to ssh:

```>>ssh deepracer@ipaddress```

1. Once ssh'd to the deepracer clone this repo:

```
>>git clone git@github.com:Hochstadt/SJSU-ANS_DeepRacer.git
>>cd SJSU-ANS_DeepRacer
```
1.5. To build the automous portion we need to make sure we have the drivers for the
IMU chip. To do so we need to install BMI160-i2c and smbus. We can do this with the following
```
>>pip install BMI160-i2c smbus2
```

2. Now source the sourceme script with the 'car' argument by running the following script.
(You should be in the directory named SJSU-ANS_DeepRacer)

```>>source setup/sourceme.sh car ```


3. When this completes successfully, create a new ssh connection through a new terminal
and become the root user by typing the following:
```
>>ssh deepracer@ipaddress
>>sudo su
```

4. Navigate to the SJSU-ANS_DeepRacer folder, and re-run the sourceme:

```>>source setup/sourceme.sh car```

This is necessary to setup the appropriate paths/environment variables
for actually running the ssh_driver. All things should be built as the
deepracer user, but things should be ran as sudo.

5. Launch the launch script:

```ros2 launch ssh_driver ssh_launcher.launch.py```

This will start up all the nodes required to maneuver the vehicle and
stream the video.

## On host (ie. not deepracer), ssh_controller:
0. If you don't have xterm on your computer install it with the following

```
sudo apt-get update

sudo apt-install xterm
```
1. In a fresh new terminal (without any prior environment variables set) clone this repo:

```
>>git clone git@github.com:Hochstadt/SJSU-ANS_DeepRacer.git
>>cd SJSU-ANS_DeepRacer && git checkout ssh_driver
```

2. Now source the sourceme script with the host argument by running the following script.
(You should be in the directory named SJSU-ANS_DeepRacer)

```>>source setup/sourceme.sh host ```

This will go and build/download everything you need.

3. Once everything successfully builds, you can actually run the ssh_controller. To do so
you need to have sudo access. Open up a fresh new terminal and run the following:

```>>sudo su```

4. Go to the directory, SJSU-ANS_DeepRacer, and re-run the sourceme script:

```source setup/sourceme.sh host```

This will setup the proper environment variables. All the building should already
be done. With sudo access, git clones get funky based on how you have your ssh
tokens set-up so this is the simplest way to do things. (Build as non-sudo, run
as sudo).

5. Launch the ssh_controller

```ros2 launch ssh_controller ssh_controller.launch.py```

6. This will launch a variety of nodes, the first thing you'll notice is
an X-term window where you can actually controller the deepracer.

To go forward:
- put the car in 'drive' by pressing 't'. This allows for commands to be registered
for moving forward
- to initiate the car motion press the 'i' key
- to increase the amount of throttle repeatedly press the 'i' key
- to decrease the amount of throttle repeatedly press the ',' key
- to go left press the 'j' key, the more times you press the stronger the
wheels turn
- to go right press the 'l' key, again the more times you press the stronger
the wheels turn
- to stop the car and take it out of 'drive' mode press the 'k' key. THe light
on the back of the car should also go out

To go backward:
- put the car in 'reverse' by pressing 'b'. All commands will now be
registered for moving backwards
- To initiate reverse motion press the ','
- The steering is the same, 'j' for left and 'l' for right
- To increase the amount/speed/throttle of reverse repeatedly press ','
- To decrease the amount/speed/throttle of reverse repeatedly press 'i'
- To stop press the 'k' key, this will take it out of reverse

An important note, in /opt/aws/deepracer there is a calibration.json. The parameters
underneath motor control the PWM that goes to impact the motor. For my configuration
I had Max: 1680000,  mid: 1400000, and min: 1260000. The min is more for the forward
the max is more for the backward and the mid helps the undelying servo controller
interpolate between the max/mid. The setting listed there work for hard wood, but seem
to need to be continually re-tuned.

7. To view the camera stream and lidar stream rviz should have popped up with the ssH_controller node. No web server is required (as it was before)

you can change the width and height in the URL to change the image size. With that said the
configuration given above gives the most real-time view I could get.

## Implementation note:
It is probably best to first built both the ssh_driver on the car and the ssh_controller
on the host computer before actually running each. The following steps ehre help navigate
exactly what you need to do assuming you've done the steps above necessary for building
the packages.
0. (on physical car) Make sure to turn on the hardware battery underneath vehicle
1. (On host) SSH into the deepracer, `>>ssh deepracer@ipaddress`
2. (On car) Get sudo rights, `>>sudo su`
3. (On car) navigate to the SJSU-ANS_DeepRacer directory and run sourceme, `>>source setup/sourceme.sh car`
4. (On car) launch the ssh_driver, `>>ros2 launch ssh_driver ssh_launcher.launch.py`
5. (On host) open a new terminal and get sudo rights, '>>sudo su'
6. (On host) go to the SJSU-ANS_DeepRacer directory that's on the host computer. Then run sourceme, `>>source setup/sourceme.sh host`
7. (On host) launch the ssh_controller, `>>ros2 launch ssh_controller ssh_controller.launch.py`
8. (ON host) open webpage: http://localhost:8080/stream?topic=/camera_pkg/display_mjpeg&width=320&height=240&quality=50
9. (On host) In the xterm window use i,jlk to control vehicle

When you started the ssh_controller after the ssh_driver was already launched data_collection would have already started. The location of
the data collection is set within the launch file of the ssh_driver. To change it, it is at the top of ssh_driver/launch/launch.py. When the
data collection is complete you can view the data on the car. To get it off, you can run the bash script in data_collector, and make sure you
are running from the root directory. Run as follows:
```
>>data_collector/retrieve_data.sh 192.168.1.26 /media/storage/good_collect
```
Where the first argument is the IP address of the deepracer and the second argument is the data location ont the deepracer. This will scp the
data you want to a data directory in the root folder and then convert the pc2 to numpy arrays as well as create a video. This is done so that
as much processing is moved to the host comptuer as possible.

# To setup the ANS GUI...
Follow the process above to setup the `ssh_driver` which the GUI needs to interface with. Similar to the `ssh_controller`, `setup/sourceme.sh host` will be used to build and setup the ANS GUI
### Run SSH driver on the car
1. Set CWD to the `SJSU-ANS_DeepRacer` directory
2. `source setup/sourceme.sh car`
3. `ros2 launch ssh_driver ssh_launcher.launch.py

### Run the ANS GUI on the host
1. Set CWD to the `SJSU-ANS_DeepRacer` directory
2. `source setup/sourceme.sh host`
3. `ros2 run ans_gui_pkg ans_gui

### Run the cmdvel_to-servo_node package
1. Set the CWD to the `SJSU-ANS_DeepRacer` directory
2. `source setup/sourceme.sh car`
3. `ros2 launch cmdvel_to_servo_pkg cmdvel_to_servo_pkg_launch.py`

** Note ** - I had to Stop the deepracer-core.service else my ROS communication would not work
`scripts/init_stop_core_services.sh`

# Post-data Collection
To run the required things for data collection from the main folder SJSU-ANS_etc, run
```
>>./data_collector/retrieve_data.sh
```

It expects arguments, but not putting any will display the brief help. This should generate the map files you'll need (although
currently does not do a pcl file and creates a numpy array that can be turned into a PointCloud2 through the map_loader
or through the navigator_host. This allows for a lot less dependnecy on things like pcd and pcl.

