# GEODOS01A
*Open-source scintillation detector of ionizing radiation. The device can be further modified according to specific requirements. Character of its construction makes it especially suitable for placement into mountaints to in-field measurement*


![Solar powered GEODOS01A](/doc/src/img/GEODOS01A.jpg "GEODOS01A prototype")


### Technical parameters

* Detection element: scintillation crystal NaI(Tl) 10 mm diameter 20 mm length integrated with SiPM detector
* Power source: solar panel
* Backup power source: rechargable 18650 Li-ion cell
* Data memory: SD card
* Record’s content: energy and time of each event
* Record’s periodicity: 10 s (maximal dead time 2 s)
* Time resolution: 20 us
* Accuracy of event’s time: 500 ns
* Energy range: 0.3 to 1 MeV (0,2 MeV resolution)
* Open-source HW and SW
* Device status indicator type LED
* Uniterrupted measurement interval 365 days minimum
* LoRa connection to IoT network
* Weather resistivity IP 65


### Device block diagram
![GEODOS01A block diagram](hw/sch_pcb/GEODOS01A_block.png)



The core of the detector  - scintillation crystal with [SiPM detector](https://en.wikipedia.org/wiki/Silicon_photomultiplier).

![Scintillation detector](/doc/src/img/GEODOS01A_sensor.jpg)

### Production data of used electronic modules

* [SIPMPOWER01A](https://github.com/UniversalScientificTechnologies/AIRDOSC01/tree/AIRDOSC01A/hw/sch_pcb/SIPMPOWER01A)
* [SIPM01B](https://github.com/UniversalScientificTechnologies/AIRDOSC01/tree/AIRDOSC01A/hw/sch_pcb/SIPM01B/hw/sch_pcb)
* [PCRD05A](http://mlab.ust.cz/module/PCRD05A)
* [GPS01B](http://mlab.ust.cz/module/GPS01B)
* [DATALOGGER01A](http://mlab.cz/module/DATALOGGER01A)
