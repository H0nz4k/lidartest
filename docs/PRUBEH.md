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

## Napájení LiDARu z USB portu Raspberry Pi 4

**Možné pouze po ověření odběru LiDARu včetně rozběhu motoru.** Raspberry Pi 4 oficiálně poskytuje maximálně **1,2 A dohromady pro všechny USB porty**, nikoli 1,2 A na každém portu. Doporučený zdroj RPi 4 je 5,1 V / 3 A. Proudový odběr tohoto konkrétního LiDARu **zatím není změřen**. Do ověření je bezpečnější samostatný 5V zdroj s proudovým omezením nebo napájený USB hub.

- Lze využít běžný USB-A port RPi 4 pro napájení přes vhodně upravený USB kabel, **ne** napájecí piny CP2102 (jeho `+5V` a `3V3` zůstávají nezapojené).
- Napájení LiDARu nikdy nepřipojovat současně ze samostatného 5V zdroje i z USB RPi 4. Pro propojení UART musí mít LiDAR a CP2102 společnou zem.
- Po ověření odběru a zapojení lze na RPi 4 zkontrolovat případné podpětí: `vcgencmd get_throttled` (hodnota `0x0` znamená, že zatím nejsou evidovány problémy s throttlingem/podpětím od startu).
- Zdroj: oficiální dokumentace Raspberry Pi – <https://www.raspberrypi.com/documentation/computers/raspberry-pi.html>.

## 2026-11-12 – PRVNÍ ÚSPĚŠNÁ KOMUNIKACE (01:43)

Uživatel poslal kompletní textový log ze spuštění `python3 tools/sniff_uart.py --port /dev/ttyUSB0` na Raspberry Pi 4.

- První pokus skončil hlášením `device reports readiness to read but returned no data` (pravděpodobné přechodné odpojení / reset USB); druhý už vysílal souvislá data.
- Nastavení **115200 baud**, `/dev/ttyUSB0`.
- Během ~6,1 sekundy zachyceno **54 272 bajtů / 212 vypsaných bloků** (potom Ctrl+C).
- V binárním proudu nalezeno **1 154 rámců s hlavičkou `54 2C`**, **všech 1 154 prošlo CRC-8 (poly 0x4D)**. Jde o rámec rodiny **LDROBOT LD06/LD19, 47 bajtů, 12 měření**. Konkrétní HW stále označen `LD00_L7`; shoda protokolu sama neprokazuje identický model.
- Celkem **13 848 měření**, z toho **8 841 nenulových vzdáleností**. Dosavadní jednotky: vzdálenost v mm, intenzita 0–255, úhel v setinách stupně.
- Údaj rychlosti v rámcích odpovídá přibližně **617,8 ot./min** (cca 10,3 ot./s, rozsah 612,3–622,0 ot./min); jde o hodnotu vysílanou LiDARem, nikoli nezávislé tachometrické měření.
- Například první platný rámec: start úhel 183,70°, konec 201,69°, vzdálenosti prvních bodů 610, 611, 616, 622 mm; CRC platí.
- Důkaz platného protokolu je výrazně silnější než samotné HEX bajty: v podstatě celé zachycené měření tvoří úplné a CRC-validní rámce.
- **Dosud není ověřena elektrická úroveň 3,3 V osciloskopem ani změřen proud při rozběhu.** Neodvozovat elektrickou bezpečnost pouze z úspěšného UART odposlechu.

Další nástroj na GitHubu: [`tools/decode_lidar.py`](../tools/decode_lidar.py). Dekóduje CRC-validní rámce do `angle_deg`, `distance_mm`, `intensity`, `rpm`, `timestamp_ms` a volitelně ukládá CSV. Neodesílá příkazy.

```bash
cd /opt/lidartest && git pull
python3 tools/decode_lidar.py --port /dev/ttyUSB0 --seconds 10 --csv captures/scan01.csv
```

Pro kontrolu dekodéru bez hardwaru: `python3 -m unittest discover -s tools -p 'test_*.py'`. Jednotkový test používá **skutečný první rámec** ze zaslaného odposlechu. Výsledky reálného běhu nového dekodéru je potřeba ještě ověřit na RPi 4.

Protokol: <https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2/blob/master/ldlidar_driver/include/dataprocess/lipkg.h> a dokumentace LD19 v2.5.
