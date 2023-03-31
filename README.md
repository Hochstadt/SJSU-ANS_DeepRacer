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

```>>git clone git@github.com:Hochstadt/SJSU-ANS_DeepRacer.git```

As of 3/12, need to also be on the ssh_driver branch. So run the following once the clone
is done:

```>>cd SJSU-ANS_DeepRacer && git checkout ssh_driver```

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

```>>git clone git@github.com:Hochstadt/SJSU-ANS_DeepRacer.git```

As of 3/12, need to also be on the ssh_driver branch. So run the following once the clone
is done:

```>>cd SJSU-ANS_DeepRacer && git checkout ssh_driver```

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



## NExt steps:
need to also collect, stream, and store lidar data and store the images. This would compose a 'collect'
