# UDIRC Hacking

This repo presents a small, and very broken, implementation of a controller for
a cheap WiFi FPV
[drone](https://www.amazon.com/Cheerwing-Control-Quadcopter-Altitude-Take-Off/dp/B076SCMRPZ).
There are likely multiple drones that could be controlled with something like
this, as a lot of these manufacturers use the same firmware/control software.

Everything here was teased out of many wireshark captures.  

This code is offered as-is.  Here are some of the biggest issues:

* JPEG video frames seem... incomplete.  The packet reassembly seems
  straightforward, but the images are corrupted, and seem to be missing a good
  chunk of entropy-encoded data.   Even though each reassembled frame has the
  appropriate JPEG header and trailer (FFD8 and FFD9)
* Controller timing is sloppy.  It seems as if some of the control signals
  (takeoff in particular), require being asserted for a period of time.  All of
  the timing in this example is suspect.
* The keyboard controller is complete crap.  It exists only to demonstrate some
  level of control.  

This has only been tested on OS X. 

```bash
# connect to Drone's WiFi AP
mkdir build
cd build
cmake ..
./udirc_example
```

Some commands:

* 't' - takeoff
* 'l' - land (unreliable)
* '<space>' - disable (reliably turns off motors)
* 'q' - throttle down
* 'e' - throttle up
* 'w' - pitch forward
* 's' - pitch back
* 'a' - roll left
* 'l' - roll right

Currently no speed control or yaw for the keyboard controller.
