An Indoor Autonomous Navigation System Implementation using AWS DeepRacer  

**SJSU / LM Cohort 2023 Master's Project -- Group 4**



# To setup ssh_driver/controller:

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
an X-term window where you can actually controller the deepracer. The commands
are 
- 'i' = forward
- ',' = backward
- 'j' = left
- 'l' = right
- 'k' = stop

Each press of the key above will increment the designated motion by .01 (on a scale
where 1 is max and -1 is max in backwards). So to turn more left you may hit the
j button multiple times. 

7. To view the camera stream go to the following website on the host-computer:
http://localhost:8080/stream?topic=/camera_pkg/display_mjpeg&width=320&height=240&quality=50

you can change the width and height in the URL to change the image size. With that said the
configuration given above gives the most real-time view I could get.


