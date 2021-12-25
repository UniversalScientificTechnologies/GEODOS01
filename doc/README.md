# Advanced user documentation of GEODOS

# Uvedení do provozu

Při instalaci zařízení musí být splněno:

  * Přívod energie - buď fotovoltaickým panelem, nebo napájením z jiného 5V zdroje
  * GPS anténa musí být umístěna tak, aby měla výhled na oblohu (Nalepena na vrchní stranu krabice, do volného okna v observatoři nebo podobně)
  * Do zařízení musí být vložena zformátovaná SDkarta na kterou se budou zapisovat data.  

Po splnění těchto podmínek je možné zařízení zapnout vypínačem. Směrem k LED je zapnuto.

![GEODOS01 power switch](/src/img/GEODOS_power_switch.png)

Po zapnutí se spustí GPS, což je indikováno LED na modulu GPS02B do kterého vede GPS anténa. Následně zařízení přibližně minutu čeká na získání fixu pozice. Získání fixu je indikováno blikáním zelené LED na modulu GPS02B.
Následně každých 10sekund blikne červená LED na levo od vypínače, což indikuje zápis na SDkartu.

## Service information

### Scintillation detector

Scintilační krystaly použité v GEODOSech mají různé rozměry. Rozměr krystalů je napsán na boku bílé kovové krabičky ve které je krystal umístěn.

**!Při otevření krabičky je nutné mít vypnuté napájení křemíkového fotonásobiče!** Při zapnutém napájení teče na světle skrz SiPM příliš velký proud a fotonásobič se zničí přehřátím.



### Photovoltaic power source

#### Solar cell array parameters

##### Elektrical parameters

- Isc = 2,3 A
- Voc= 2,5 V
- Impp= 2,1 A
- Vmpp= 2,1 V
- Pm= 4,4 Wp
