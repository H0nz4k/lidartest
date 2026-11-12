# Průběh zprovoznění LiDARu ETA Raggio

## 2026-11-12 – identifikace a elektrický test

- LiDAR vyjmutý z robotického vysavače ETA Raggio.
- Potisk modulu: `LD00_L7_PWR_TRANS_180828`.
- LiDAR napájen z vlastního USB zdroje 5 V: LED svítí.
- Hlava se zatím netočí; mezi červeným a černým vodičem motoru naměřeno 0 V.
- Na původní desce vysavače je u konektoru `RADAR` potisk `RX`, `TX`, `GND`, `+`.
- Pro testování bude použito **Raspberry Pi 4** a převodník **CP2102**; produkční HUB se nepoužije.
- Napěťové úrovně RX/TX, protokol, baudrate a podmínka roztočení zatím nejsou ověřené.

## Test 01 – pasivní příjem přes CP2102

Skript: [`tools/sniff_uart.py`](../tools/sniff_uart.py).

**Aktualizace zapojení:** Uživatel má již fyzicky připojené **RXD i TXD CP2102** a TXD nyní nemůže odpojit. Není nutné ho odpájet jen kvůli testovacímu skriptu; skript **nevolá `write()` a neposílá UART příkazy**. Fyzicky připojené TXD ale není zcela pasivní – v klidu drží logickou úroveň, obvykle cca 3,3 V. Proto **před připojením USB převodníku / zapnutím** ověřit, že TXD CP2102 opravdu vede do vstupu LiDARu (nikoli do jeho výstupu), a že **výstup LiDARu vůči GND nepřekročí 3,3 V**. Pro neověřené směry a úrovně nelze zapojení považovat za bezpečné: při propojení dvou výstupů hrozí jejich elektrický konflikt. Kde je dostupný osciloskop/logický analyzátor s odpovídající úrovní, ověřit přenos i špičky. Označení RX/TX na původní desce vysavače je z její perspektivy, směr zatím měřením nepotvrzen. **CP2102 +5V a 3V3 k LiDARu nepřipojovat**, jeho 5V napájení zůstává samostatné; propojit GND.

Příprava na RPi 4:

```bash
git clone https://github.com/H0nz4k/lidartest.git
cd lidartest
sudo apt install -y python3-serial
ls -l /dev/serial/by-id/ /dev/ttyUSB* 2>/dev/null
```

Pasivní čtení po dobu 15 s:

```bash
python3 tools/sniff_uart.py --port /dev/ttyUSB0 --baud 115200 --seconds 15
```

Volitelně uložit syrový proud pro pozdější dekódování:

```bash
python3 tools/sniff_uart.py --port /dev/ttyUSB0 --baud 115200 --seconds 15 --output captures/test01.bin
```

Skript nevysílá žádné UART bajty a nijak neřídí motor. Rychlost 115200 baudů je **pouze počáteční odhad**. Pokud přijde 0 bajtů, neznamená to samo o sobě nefunkční LiDAR.

### Výsledek

**Nevyzkoušeno na připojeném hardware.** Doplnit po prvním skutečném běhu: datum, změřená napětí, použitý port, baudrate, počet bajtů a ukázku HEX výstupu. Poté rozhodnout o dalších krocích.

## 2026-11-12 – první pokus o spuštění na RPi 4

Na RPi 4 (`mapping`) bylo `python3-serial` nainstalováno, ale spuštění skriptu skončilo `can't open file '/opt/tools/sniff_uart.py'`. Příčina: **chybí Git** (`bash: git: command not found`), takže `git clone` neproběhl, adresář `lidartest` neexistuje a `cd lidartest` selhalo. Nejde zatím o chybu komunikace s LiDARem.

Náprava (z aktuálního shellu `root@mapping:/opt#`):

```bash
apt install -y git && git clone https://github.com/H0nz4k/lidartest.git /opt/lidartest && cd /opt/lidartest && ls -l /dev/ttyUSB* && python3 tools/sniff_uart.py --port /dev/ttyUSB0
```

Výsledek skutečného UART odposlechu zatím **neznámý**. Spuštění až po ověření správného zapojení a 3,3V logiky, jak je uvedeno výše.

## 2026-11-12 – nové měření / rotace

- Uživatel naměřil **0,8 V** při měření datového signálu, dosud ale neupřesnil, zda šlo o RX nebo TX (a zda byl CP2102 napájen). Jediná hodnota z multimetru při UART přenosu **nepotvrzuje** logické úrovně ani bezpečnost zapojení; může jít o průměr proměnného signálu.
- **Nový stav:** LiDAR se nyní **roztočil**. Příčina rozběhu zatím není potvrzena; RX i TX byly už propojené s CP2102.
- Další krok: bez posílání inicializačních povelů provést pasivní odposlech `python3 tools/sniff_uart.py --port /dev/ttyUSB0` na Raspberry Pi 4 **až po ověření bezpečných úrovní**, případně pomocí vhodného převodníku úrovní. Zaznamenat výstup a podmínky roztočení; nezasahovat do roztočené mechaniky a nehledět do laserového okénka.
