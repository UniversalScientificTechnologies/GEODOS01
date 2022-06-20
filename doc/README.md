# Advanced user documentation of GEODOS

# Uvedení do provozu

Při instalaci zařízení musí být splněno:

  * Přívod energie - buď fotovoltaickým panelem, nebo napájením z jiného 5V zdroje
  * GPS anténa musí být umístěna tak, aby měla výhled na oblohu (Nalepena na vrchní stranu krabice, do volného okna v observatoři nebo podobně)
  * Do zařízení musí být vložena zformátovaná SDkarta na kterou se budou zapisovat data.  

## Zapnutí přístroje

Po splnění těchto podmínek je možné zařízení zapnout vypínačem. Směrem k LED je zapnuto.

![GEODOS01 power switch](/doc/src/img/GEODOS_power_switch.png)

Po zapnutí se spustí GPS, což je indikováno LED na modulu GPS02B do kterého vede GPS anténa. Následně zařízení přibližně minutu čeká na získání fixu pozice. Získání fixu je indikováno blikáním zelené LED na modulu GPS02B.
Následně každých 10sekund blikne červená LED na levo od vypínače, což indikuje zápis na SDkartu.

## Získávání dat

GEODOS dovede data na SDkartu zaznamenávat velmi dlouho. Šest měsíců záznamu odpovídá přibližně 500MB dat. Běžné velikosti SDkaret umožňují kontinuální záznam řádově v rocích provozu. Z tohoto důvodu je GEODOS vybaven LoRa telemetrií, aby bylo možno přibližně monitorovat jeho funkčnost.  
Používat GEODOS tímto způsobem ale není úplně rozumné, neboť tato vzdálená telemetrie není schopna monitorovat kvalitu dat. Je tak dobré data manuálně stahovat buď pravidelně, nebo na základě výskytu významných událostí.

## Vypnutí přístroje a vyčtení dat

Díky způsobu fungování, kdy přístroj zaznamenává na SDkartu jednou za 10 sekund je vhodné vypnutí provést tak, že **počkáme na dvojbliknutí červené LED vlevo od vypínače a následně vypínačem vypneme napájení**.
Tento postup minimalizuje možnost poškození souborového systému na SDkartě nedokončeným zápisem.

Následně je možná SDkartu vyjmout zatáhnutím nehtem (SDkarta je umístěna v prostoru mezi vypínačem a indikační LED).

![SDcard removing](/doc/src/img/GEODOS_SDcard_pull-out.jpg)

SDkartu lze přečíst v běžné čtečce SDkaret. Vyzkoušeným typem je například [Kingston USB 3.0 High-Speed Card Reader](https://www.kingston.com/us/memory-card-readers/usb-3-0-high-speed-media-reader)

Na SDkartě je pak za normílních okolností pouze jeden soubor DATALOG.TXT, který obsahuje všechny zaznamenané informace.  


## Service information

### Debug output

GEODOS has debug output at RX0, TX0 baudrate is 38400.

  $HIST,63,861.90,97386.50,25.81,3.82,16,473,883,2,12704,21600,80,16,11,0,3,5,1,1,3,0,1,2,1,1,1,0,0,1,0,0,0,2
  $HITS,29,4879,35,5201,30,17202,36,18704,30,21061,102,27156,61,27574,79,27797,26,32746,40,40409,58,48144,36,49356,54,51772,37,52090,69,56941,61,62446,36,63116,132,65634,104,67413,35,68563,32,70039,30,70211,56,83196,28,83445,63,87672,32,88476,29,92791,26,99154,84,99543,29

### Firmware upgrade

For the firmware upgrade, the USB-RS232 converter is needed.  Normally is used the [USB232R02](https://github.com/mlab-modules/USB232R02), but any [USBcable](https://techfun.sk/produkt/kabel-pl2303hx-usb-na-ttl-rs232/) could be used.

![PL2303XHD cable](/doc/src/img/PL2303XHD.jpg)


#### Cable connection

|USB converter cable | DATALOGGER01A |
|--------------|---------------|
|RX | TX0|
|TX | RX0|
|RTS | RST# |
|GND | GND|

Physical realization

![PL2303XHD cable - DATALOGGER01A connection](/doc/src/img/GEODOS_firmware_upgrade.jpg)

#### Upgrade process

    cd fw/bin/FIRMWARE_VERSION
    ./program.sh

Expected output:

    $ ./program.sh

    avrdude: Version 6.3-20190619
             Copyright (c) 2000-2005 Brian Dean, http://www.bdmicro.com/
             Copyright (c) 2007-2014 Joerg Wunsch

             System wide configuration file is "./avrdude.conf"
             User configuration file is "/home/kaklik/.avrduderc"
             User configuration file does not exist or is not a regular file, skipping

             Using Port                    : /dev/ttyUSB0
             Using Programmer              : arduino
             Overriding Baud Rate          : 115200
             AVR Part                      : ATmega1284P
             Chip Erase delay              : 55000 us
             PAGEL                         : PD7
             BS2                           : PA0
             RESET disposition             : dedicated
             RETRY pulse                   : SCK
             serial program mode           : yes
             parallel program mode         : yes
             Timeout                       : 200
             StabDelay                     : 100
             CmdexeDelay                   : 25
             SyncLoops                     : 32
             ByteDelay                     : 0
             PollIndex                     : 3
             PollValue                     : 0x53
             Memory Detail                 :

                                      Block Poll               Page                           Polled
               Memory Type Mode Delay Size  Indx Paged  Size   Size #Pages MinW  MaxW       ReadBack
               ----------- ---- ----- ----- ---- ------ ------ ---- ------ ----- ----- --------    -
               eeprom        65    10   128    0 no       4096    8      0  9000  9000 0xff     0xff
               flash         65    10   256    0 yes    131072  256    512  4500  4500 0xff     0xff
               lock           0     0     0    0 no          1    0      0  9000  9000 0x00     0x00
               lfuse          0     0     0    0 no          1    0      0  9000  9000 0x00     0x00
               hfuse          0     0     0    0 no          1    0      0  9000  9000 0x00     0x00
               efuse          0     0     0    0 no          1    0      0  9000  9000 0x00     0x00
               signature      0     0     0    0 no          3    0      0     0     0 0x00     0x00
               calibration    0     0     0    0 no          1    0      0     0     0 0x00     0x00

             Programmer Type : Arduino
             Description     : Arduino
             Hardware Version: 3
             Firmware Version: 8.0
             Vtarget         : 0.3 V
             Varef           : 0.3 V
             Oscillator      : 28.800 kHz
             SCK period      : 3.3 us

    avrdude: AVR device initialized and ready to accept instructions

    Reading | ################################################## | 100% 0.00s

    avrdude: Device signature = 0x1e9705 (probably m1284p)
    avrdude: safemode: lfuse reads as 0
    avrdude: safemode: hfuse reads as 0
    avrdude: safemode: efuse reads as 0
    avrdude: reading input file "./AIRDOSC_1024_LS.ino.hex"
    avrdude: writing flash (25132 bytes):

    Writing | ################################################## | 100% 3.28s

    avrdude: 25132 bytes of flash written
    avrdude: verifying flash memory against ./AIRDOSC_1024_LS.ino.hex:
    avrdude: load data flash data from input file ./AIRDOSC_1024_LS.ino.hex:
    avrdude: input file ./AIRDOSC_1024_LS.ino.hex contains 25132 bytes
    avrdude: reading on-chip flash data:

    Reading | ################################################## | 100% 2.28s

    avrdude: verifying ...
    avrdude: 25132 bytes of flash verified

    avrdude: safemode: lfuse reads as 0
    avrdude: safemode: hfuse reads as 0
    avrdude: safemode: efuse reads as 0
    avrdude: safemode: Fuses OK (E:00, H:00, L:00)

    avrdude done.  Thank you.

The new firmware version should be marked at beginnig of  DATALOG.TXT file after the power-up and logging some data. e.g.

    $AIRDOS,C_LS_1024_v2,...

### Scintillation detector

Scintilační krystaly použité v GEODOSech mají různé rozměry. Rozměr krystalů je napsán na boku bílé kovové krabičky ve které je krystal umístěn.

**Při otevření krabičky je nutné mít vypnuté napájení křemíkového fotonásobiče** Při zapnutém napájení teče na světle skrz SiPM příliš velký proud a fotonásobič se zničí přehřátím.



### Photovoltaic power source

#### Solar cell array parameters

##### Elektrical parameters

- Isc = 2,3 A
- Voc= 2,5 V
- Impp= 2,1 A
- Vmpp= 2,1 V
- Pm= 4,4 Wp
