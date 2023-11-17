# GEODOS01B - Stand alone ionizing radiation monitor

*Open-source scintillation detector of ionizing radiation. The device can be further modified according to specific requirements. Character of its construction makes it especially suitable for placement into mountaints for in-field measurements*

![GEODOS01A Installed on Polednik site](/doc/src/img/GEODOS_Polednik_site.jpg)

If you need a fully automatic ionising radiation dosimeter-spectrometer device (with an internal logging and a backup power supply) or device designed for outdoor use without the possibility of connecting to power supply and data network [GEODOS01](https://github.com/UniversalScientificTechnologies/GEODOS01) might be a better option. We can help you in choosing the most suitable device or even design and develop a new one according to you requirements. In either case, reach us using email [support@ust.cz](mailto:support@ust.cz).

### Locations

GEODOS devices are mounted on multiple significant locations around Europe at this moment.

#### Chernobyl Red Forest

One GEODOS device is installed in [Chernobyl Red Forest site](https://en.wikipedia.org/wiki/Red_Forest).

![GEODOS01A Installed in Chernobyl Red Forest site](/doc/src/img/GEODOS_Chernobyl_redforest_site.jpg "GEODOS01A Installed in Chernobyl Red Forest site")

![GEODOS01A raw data before temperature compensation](/doc/src/img/GEODOS_chernobyl_graph.png)

#### Šumava mountains

GEODOS devices are installed on multiple locations in Šumava. For example example at [Polednik watch tower](https://cs.wikipedia.org/wiki/Poledn%C3%ADk_(%C5%A0umava)).

![GEODOS01A Installed on Polednik site](/doc/src/img/GEODOS_Polednik_site.jpg)


#### Kosetice Atmospheric Tower

The [National Atmospheric Observatory Košetice](https://actris-ri.cz/) (NAOK) was established by the Czech Hydrometeorological Institute as a department specialized in long-term air quality monitoring and research at the background scale in 1988. The GEODOS is located here by [CRREAT project](http://www.ujf.cas.cz/en/research-development/large-research-infrastructures-and-centres/crreat/objectives/).

![GEODOS01B Installed Kosetice Atmospheric Tower](/doc/src/img/kosetice_atmospheric_tower.jpg)


### Technical parameters

* Detection element: scintillation crystal NaI(Tl) 14 mm in diameter, 20 mm in length, integrated with SiPM detector
* Power source: solar panel
* Backup power source: LTO cells
* Data memory: SD card
* Record’s content: energy and time of each event above the specified energy (1MeV by default)
* Record’s periodicity: 10 s (maximal dead time 2 s)
* Time resolution: 20 us
* ADC Conversion time: 104 us
* [Dead time](https://en.wikipedia.org/wiki/Dead_time): 2 us
* Accuracy of event’s time: 500 ns
* Energy range: 0.3 to 18 MeV (deposited energy)
* Resolution: 0.02 MeV
* Open-source HW and SW
* Device status indicator type: LED
* Uniterrupted measurement campaign interval: 6 months for 2 GB SD card
* LoRa connection to IoT network
* Operational temperature range: -20°C to +35°C
* Charging temperature range: -30°C to +50°C
* Dimensions 254x180x111 mm [enclosure TK PS 2518-11-o](https://www.spelsberg.co.uk/industrial-housing/with-/-without-metric-knock-outs/11090801/)
* Weather resistivity IP 33


### Device block diagram

![GEODOS01A block diagram](hw/sch_pcb/GEODOS01B_block.png)

### Sensor element

The core of the detector is a scintillation crystal with [SiPM detector](https://en.wikipedia.org/wiki/Silicon_photomultiplier).

![Scintillation detector in box](/doc/src/img/GEODOS01A_sensor.jpg)

![Scintillation detector housing](/doc/src/img/GEODOS01A_sensor_box.jpg)

### Data storage

The primary raw data is stored in SDcard's memory. The IoT network is used for telemetry data transfer, device monitoring (e.g. temperature, humidity, pressure, battery voltage etc.). We are regulary using the [TTN](https://www.thethingsnetwork.org/), but the device could be configured for use with any IoT LoRa based network.
For remote areas, mounting a IoT gateway in radio range of GEODOS instruments could also be a good option.


#### Messages


#### About message

As a second message, when turned on, a string is sent that uniquely identifies the device.

```
$DOS,GEODOS01B,L02,256,379276a,1290c00806a200914056a000a0000086
```
* `$DOS` - is the first character of the message string
* `GEODOS01B` - Name of the device. In this case, it is GEODOS
* `L02`- FW version
* `256` - ADC offset
* `379276a` - Hash of commit with this firmware
* `1290c00806a200914056a000a0000086` - Unique serial number of dosemeter instrument

#### Data message

The output data are in the form of textual [NMEA-0183](https://en.wikipedia.org/wiki/NMEA_0183) like strings. There are the two main message types - Histogram message ($HIST) and events stream message ($HITS). Both of these messages could be combined in data output from the single device.  

##### Histogram message

```
$HIST,8,118.15,960.09,4.06,2.33,394,803,0,3,30656,2401,1304,5,3,4,7,9,5,4,6,7,4,3,0,8,2,3,1,1,1,4,3
```

* `$HIST` - Marking a message with spectrums
* `count` - Message number since restart or power-up
* `time` - Time in seconds from power-up
* `suppress` -
* `dose` - Number of detected particles
* `offset` - ADC offset. Specifies the zero channel position.
* `energetic channels` - All remaining values indicate a certain energy channel. From the smallest to the largest. The third value is 1st channel.

##### Events message

```
$HITS,20,916,84,2984,37,16386,38,30666,26,38530,48,43043,27,49257,100,53904,43,59650,32,65631,32,65802,46,68555,47,71124,53,73601,48,74179,59,77454,72,90563,30,98074,74,98901,124,99743,50
```
* `$HITS` - Marking a message with individual events
* `hit count` - Number of events captured from the last HITS message
* `time` - Time in seconds from power-up
* `hit channel` - Channel correspondind to the captured event

### Detailed documentation of the used electronic modules

* [STEPUPDC02A](https://github.com/mlab-modules/STEPUPDC02)
* [SIPM02C](https://github.com/mlab-modules/SIPM02) known as "AIRDOSC01A_PCB01C"
* [PCRD06A](https://github.com/mlab-modules/PCRD06)
* [GPS01B](https://www.mlab.cz/module/GPS01B)
* [DATALOGGER01A](http://mlab.cz/module/DATALOGGER01A)
* [ALTIMET01A](https://github.com/mlab-modules/ALTIMET01)
* [ISM02B](https://github.com/mlab-modules/ISM02)
* [TPS63060V01A](https://github.com/mlab-modules/TPS63060V01)
* [LION1CELL01B](https://github.com/mlab-modules/LION1CELL01)
* [SOLARMINIBAT01A](https://github.com/mlab-modules/SOLARMINIBAT01)
* [SOLARMINI01B](https://github.com/mlab-modules/SOLARMINI01)
