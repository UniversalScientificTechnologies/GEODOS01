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

### Firmware upgrade

For the firmware upgrade, the USB-RS232 converter is needed.  Normally is used the [USB232R02](https://github.com/mlab-modules/USB232R02), but any [USBcable](https://techfun.sk/produkt/kabel-pl2303hx-usb-na-ttl-rs232/) could be used.

![PL2303XHD cable](/doc/src/img/PL2303XHD.jpg)

|USB converter cable | DATALOGGER01A |
|--------------|---------------|
|RX | TX0| 
|TX | RX0|
|RTS | RST# |
|GND | GND|

#### Upgrade process

    cd fw/bin/FIRMWARE_VERSION
    ./program.sh
    


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
