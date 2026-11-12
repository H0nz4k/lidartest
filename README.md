# ETA Raggio – LiDAR test (LD00_L7)

Samostatné oživení rotačního LiDARu z robotického vysavače **ETA Raggio**, bez zásahů do produkčního HanzHubu.

**Hostitel:** Raspberry Pi 4. **USB–UART:** CP2102. **Napájení LiDARu:** samostatný USB 5V zdroj.

## Aktuální stav (2026-11-12)

- Na desce LiDARu je označení `LD00_L7_PWR_TRANS_180828`.
- Po připojení samostatného USB 5V napájení se rozsvítila LED.
- Při prvním testu byl motor na 0 V, ale po úpravě zapojení se **LiDAR roztočil**.
- Na základní desce původního vysavače je vedle konektoru `RADAR` potisk `RX`, `TX`, `GND`, `+`.
- K dispozici je USB–UART převodník CP2102 s vývody `DTR`, `RXD`, `TXD`, `+5V`, `GND`, `3V3`.
- **UART funguje při 115200 baud**: ze skutečného odposlechu 12. 11. 2026 bylo dekódováno **1 154 platných CRC rámců / 13 848 bodů** za cca 6,1 sekundy. Formát odpovídá LDROBOT LD06/LD19 (hlavička `54 2C`, 47 B). Napěťové úrovně osciloskopem ani povel pro motor zatím nebyly potvrzeny.

## Zapojení – po ověření úrovní signálů

| RADAR na základní desce vysavače | CP2102 | Poznámka |
| --- | --- | --- |
| RX | RXD | Předpoklad: RX na původní desce přijímalo data vyslaná LiDARem |
| TX | TXD | Pro aktivní komunikaci až po kontrole úrovní a směru |
| GND | GND | Společná zem obou zařízení |
| + | **nepřipojovat** | LiDAR má vlastní samostatné USB napájení |
| — | DTR, +5V, 3V3 | Nepřipojovat |

**Upozornění:** Název `RX`/`TX` je z pohledu **původní desky vysavače**, nikoli nutně z pohledu modulu LiDARu; směr spojení je nutné ověřit měřením nebo odposlechem. Nikdy nepřipojovat datové vývody na 5 V. CP2102 typicky komunikuje s 3,3V úrovněmi; u konkrétního modulu ověřit napětí v klidu i charakter úrovní. Pro nejbezpečnější první test spojit jen GND a ověřený datový výstup LiDARu na RXD CP2102, jeho TXD ponechat odpojený. Měření multimetrem v klidu samo o sobě neověří špičkové úrovně při přenosu.

## Pasivní odposlech a dekódování na Raspberry Pi 4

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

`115200` nyní **funguje s připojeným LiDARem** a rámce procházejí CRC. Pro export všech bodů do CSV spusť:

```bash
cd /opt/lidartest && git pull
python3 tools/decode_lidar.py --port /dev/ttyUSB0 --seconds 10 --csv captures/scan01.csv
```

Dekodér ověřuje CRC-8 rámců, dopočítává úhly 12 bodů v rámci a ukládá vzdálenost v mm, intenzitu, otáčky a čas. Není nutné odesílat žádné UART příkazy. Kód jednotkového testu na skutečném rámci je `tools/test_decoder.py`.

## Živá 360° mapa v prohlížeči (Raspberry Pi 4)

**Nové:** [`tools/live_map.py`](tools/live_map.py) + [`web/index.html`](web/index.html).
Živý webový radar čte CRC-validní rámce LDROBOT přímo z CP2102, průběžně aktualizuje body s rozlišením 0,5°, zobrazuje vzdálenosti a otáčky. Body starší než 1,8 s mizí, 0mm měření maže předchozí bod v odpovídajícím směru. Mapa je **relativní k poloze LiDARu**, nejde o SLAM ani kumulativní mapu při pohybu.

Na Raspberry Pi 4 (kde už je repozitář ve složce `/opt/lidartest`):

```bash
cd /opt/lidartest && git pull && python3 tools/live_map.py
```

Pokud `python3-serial` není nainstalovaný: `sudo apt install -y python3-serial`.
Nech ukončený jiný program pracující s `/dev/ttyUSB0` (např. `sniff_uart.py`).
Web otevři v jiném zařízení ve stejné síti na **`http://IP_RPI4:8765`**; IP zjistíš na Pi pomocí `hostname -I`. Port lze změnit parametrem `--http-port`, sériový port `--port`. Ukončení Ctrl+C.

Případná diagnostika bez skutečného senzoru (již existující binární odposlech):

```bash
python3 tools/live_map.py --replay captures/scan01.bin
```

**Pozor:** Webový server standardně naslouchá v lokální síti (`0.0.0.0:8765`), neobsahuje přihlášení – nezveřejňovat ho do internetu. Pro přístup jen z Pi použij `--host 127.0.0.1`. Čtení UART je pasivní; program neposílá motoru ani LiDARu žádné povely.

## Další kroky

1. Naměřit napětí na vstupním `+`/GND přímo u LiDARu pod zátěží (ověřit skutečných ~5 V).
2. Ověřit klidová napětí `RADAR RX` a `RADAR TX` proti GND a určit skutečný vysílací směr.
3. **Splněno:** zachycena a CRC ověřena měřicí data při 115200 baud.
4. **Připraveno:** živá 360° mapa v prohlížeči (ještě neověřena na uživatelově Raspberry Pi 4).
5. Upřesnit elektrické zapojení a okolnosti automatického roztočení, ideálně ověřit osciloskopem / měřením proudu.

Podrobnosti a průběžné měření: [docs/PRUBEH.md](docs/PRUBEH.md).

**Bezpečnost:** Nehledět do laserového okénka, nepřekračovat ověřené napájení, nepropojovat dvě napájecí 5V větve a neaplikovat 5 V na GPIO nebo UART data.
