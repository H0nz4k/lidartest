# ETA Raggio – LiDAR test (LD00_L7)

Samostatné oživení rotačního LiDARu z robotického vysavače **ETA Raggio**, bez zásahů do produkčního HanzHubu.

**Hostitel:** Raspberry Pi 4. **USB–UART:** CP2102. **Napájení LiDARu:** samostatný USB 5V zdroj.

## Aktuální stav (2026-11-12)

- Na desce LiDARu je označení `LD00_L7_PWR_TRANS_180828`.
- Po připojení samostatného USB 5V napájení se rozsvítila LED.
- Hlava se **neroztočila**; na dvou vodičích motoru bylo naměřeno **0 V**.
- Na základní desce původního vysavače je vedle konektoru `RADAR` potisk `RX`, `TX`, `GND`, `+`.
- K dispozici je USB–UART převodník CP2102 s vývody `DTR`, `RXD`, `TXD`, `+5V`, `GND`, `3V3`.
- Zatím **nejsou doloženy** skutečné úrovně signálů, přijatá UART data, baudrate ani povel pro motor. Žádná komunikace dosud nebyla potvrzena.

## Zapojení – po ověření úrovní signálů

| RADAR na základní desce vysavače | CP2102 | Poznámka |
| --- | --- | --- |
| RX | RXD | Předpoklad: RX na původní desce přijímalo data vyslaná LiDARem |
| TX | TXD | Pro aktivní komunikaci až po kontrole úrovní a směru |
| GND | GND | Společná zem obou zařízení |
| + | **nepřipojovat** | LiDAR má vlastní samostatné USB napájení |
| — | DTR, +5V, 3V3 | Nepřipojovat |

**Upozornění:** Název `RX`/`TX` je z pohledu **původní desky vysavače**, nikoli nutně z pohledu modulu LiDARu; směr spojení je nutné ověřit měřením nebo odposlechem. Nikdy nepřipojovat datové vývody na 5 V. CP2102 typicky komunikuje s 3,3V úrovněmi; u konkrétního modulu ověřit napětí v klidu i charakter úrovní. Pro nejbezpečnější první test spojit jen GND a ověřený datový výstup LiDARu na RXD CP2102, jeho TXD ponechat odpojený. Měření multimetrem v klidu samo o sobě neověří špičkové úrovně při přenosu.

## První pasivní odposlech na Raspberry Pi 4

1. LiDAR napájet vlastními 5 V z USB; CP2102 zapojit do USB Raspberry Pi 4. Propojit jejich GND.
2. **Než připojíte RXD CP2102**, ověřit napětí signálu, který skutečně vysílá z LiDARu, vůči GND. Pokud přesahuje 3,3 V nebo je nejasný, použít převodník úrovní / osciloskop, nikoli přímé propojení.
3. Odposlech provádět pro začátek bez připojeného TXD (program nic neposílá).
4. V Bashi na RPi 4:

```bash
sudo apt update
sudo apt install -y python3-serial
ls -l /dev/serial/by-id/ /dev/ttyUSB*
python3 tools/sniff_uart.py --port /dev/ttyUSB0 --baud 115200 --seconds 15
```

`115200` je pouze **první kandidátní rychlost**, ne ověřené nastavení pro tento model. Pokud se nic neobjeví, může jít o jinou rychlost, neaktivní měřicí hlavici nebo řízení motoru mimo UART. Po prvním odposlechu je možné bezpečně pokračovat ověřením druhé linky a podmínky roztočení motoru.

## Další kroky

1. Naměřit napětí na vstupním `+`/GND přímo u LiDARu pod zátěží (ověřit skutečných ~5 V).
2. Ověřit klidová napětí `RADAR RX` a `RADAR TX` proti GND a určit skutečný vysílací směr.
3. Zachytit první sériová data přes CP2102; zaznamenat hex výstup a rychlost.
4. Prověřit, zda se motor ovládá přes UART, PWM nebo samostatný driver (označení `PWM` na desce LiDARu je jen indicie).
5. Až potom dekódovat skeny a zobrazit 360° mapu.

Podrobnosti a průběžné měření: [docs/PRUBEH.md](docs/PRUBEH.md).

**Bezpečnost:** Nehledět do laserového okénka, nepřekračovat ověřené napájení, nepropojovat dvě napájecí 5V větve a neaplikovat 5 V na GPIO nebo UART data.
