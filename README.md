# Arduino Telemetry

Arduino-based IMU telemetry system for model rockets. 

Phase 1: Build a weather station with custom PCB to learn basics of PCB design and sensor data

Phase 2: Using knowledge from Phase 1, design custom rocket avionics board using off-the-shelf components and test in rocket.
    -> Assemble flight computer once design is selected

Phase 3: Build ground-control system to communicate with flight computer

Phase 4: Build rocket

Phase 5: Test rocket design and launch! 🚀

# Phase I
## PCB Schematics
### MK I
The MK I PCB was unfortunately unsuccessful, as I used the EasyEDA autoroute feature to try to solve track placement issues, however it did not take into account the possiblity of shorting the circuits, and neither did I, so my Arduino Nano blew a capacitor. Thankfully I have more, but this board was immediately decommissioned.

<img src="Images/Mk_I_F.png" width="60%" />
<img src="Images/Mk_I_B.png" width="60%" />

### MK II
The MK II PCB was carefully redesigned to improve upon the Mk I's failures by utilizing better component placement, built-in grounding pads, and hand-drawn track placement. (Blue also seemed like it might look nicer.) 

<img src="Images/Mk_II_F.png" width="60%" />
<img src="Images/Mk_II_B.png" width="60%" />

## Update:
Custom PCB worked after modification of grounding system, for next PCB design will use a single grounding pin instead of seven which were ultimately soldered together. 

<img src="Images/PCB-test.PNG" width="60%" />


# Phase II & III
## Flight Computer
This is the assembled flight computer with all sensors, now compatible with Li-Poly support! Ground Control system is functional and connects to the flight computer and the node-red server


<img src="Images/flightCPU.PNG" width="60%" />
<img src="Images/groundCtrl.PNG" width="60%" />

> UPDATE 
>
> Main computer has been upgraded to the Adafruit Feather M0 with LoRa Radio @900Mhz, and IMU has been upgraded to MPU 6050 with 6 DOF (and gyroscope!). Code is being updated and overall communication framework is being transitioned to radio transmission.

The AirLift Wi-Fi Co-Processor on both the Adalogger and Metro are able to successfully communicate with the Node-Red server, to send sensor data over UDP for collection and assimilation. The Node-Red dashboard is configured to not only build graphs in real-time, but also saves the data as it is received to a JSON file. 

<img src="Images/nodeRedDash.png" width="60%" />
<img src="Images/nodeRedGraph.png" width="60%" />



# Phase IV
## Rocket Design and Construction 
> Currently doing research and teaching myself rocket mechanics to design and build a stable, efficient rocket. Updates coming soon.
