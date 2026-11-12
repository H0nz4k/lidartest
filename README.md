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

## 2D SLAM – experimentální mapa při pohybu

**Připraveno 12. 11. 2026:** [nástroj 2D SLAM](tools/slam_server.py) a
[webová mapa](web/slam.html). Na rozdíl od radaru nad jednou rotací
skládá postupně obsazenost prostoru a odhaduje relativní polohu z
navazujících skenů (scan-to-map). Zatím **bez IMU, odometrie a loop closure**;
pohyb musí být pomalý, může se kumulovat drift. Pro zmapování dalších místností
je nutné přesouvat **celý** LiDAR; se stojícím snímačem mapa nebude růst mimo
jeho dosah.

Na **Raspberry Pi 4** zastavit běžící čtečky UART a spustit:

~~~bash
sudo apt install -y python3-numpy python3-serial
sudo git -C /opt/lidartest pull
sudo python3 /opt/lidartest/tools/slam_server.py
~~~

Mapování: **http://IP_RPI4:8766/**, současně původní 360° radar:
**http://IP_RPI4:8766/radar** (jediná společná čtečka CP2102).
Prostřednictvím webu lze uložit mapu PGM a trasu CSV, změnit měřítko
a resetovat pouze odvozenou mapu. Server je bez přihlášení: **pouze
důvěryhodná místní síť**. Kompletní návod:
[docs/SLAM2D.md](docs/SLAM2D.md).

**Ověření:** GitHub Actions offline testy simulovaného přesunu, datového
protokolu a JavaScriptu prošly. **Fyzické mapování při reálném přesunu
zatím neověřeno.**

## MPU-6050 + LiDAR – experimentální gyro fúze

**MPU-6050 již funguje:** reálný test gyro Z a akcelerometru na RPi 4 prošel
(12. 11. 2026). Gyroskop má offset −0,202 °/s a šum 0,032 °/s
během klidové kalibrace. Modul RPishop je napájen **5 V na VCC**
(fyzický pin 2 RPi4), logika SDA/SCL musí zůstat nejvýše 3,3 V.
SDA→fyzický pin 3, SCL→pin 5, GND→pin 6.

Nově je připravená **volitelná fúze gyro Z s 2D SLAM scan matcherem**.
Pro spuštění se senzorem (původní 360° radar zůstává na `/radar`):

~~~bash
sudo apt install -y python3-numpy python3-serial python3-smbus
sudo git -C /opt/lidartest pull
sudo python3 /opt/lidartest/tools/slam_server.py --imu
~~~

Během prvních **2,5 s** nech celou konstrukci nehybnou pro kalibraci
gyroskopu. Aplikace ukazuje `gyroZ`, relativní úhel IMU, stav
fúze a chyby I²C. Při odpojení IMU automaticky pokračuje
LiDAR-only scan matching. Přínos musí být ještě ověřen při pohybu
na skutečné základně; IMU není absolutní měření X/Y ani kompas.

Na RPi4 je navíc **kamera přes CSI plochý kabel**. Zatím se jí nic
nemění a aktuální SLAM ji nepoužívá. V budoucnu může pomoci s
vizuální odometrií a korekcí translačního pohybu.

Dokumentace: [MPU-6050](docs/MPU6050.md) a
[SLAM + IMU](docs/SLAM2D.md). Test IMU: [imu_test.py](tools/imu_test.py).

### Živý obraz: kamera a LiDAR v jednom okně

Nová stránka **/camera** umí zobrazit skutečný CSI obraz s
průhlednou vrstvou úhlově přiřazených **skutečných vzdáleností** LiDARu.
Zobrazení je orientační – bez prostorové kalibrace kamery **nejde
o přesnou AR registraci** a naklánění skeneru stále vyžaduje 3D model.

~~~bash
cd /opt/lidartest && git pull
sudo python3 tools/slam_server.py --imu --camera --camera-preview
~~~

Web: **http://IP_RPI4:8766/camera**; stránka SLAM: `/`,
360° radar: `/radar`. Náhled je **explicitně opt-in**, přibližně
5 JPEG snímků/s v paměti, bez nahrávání na disk.
**Pozor:** HTTP náhled kamery nemá heslo – jen pro důvěryhodnou LAN,
nevystavovat internetové veřejnosti. Kamera nadále nesmí být
současně používaná ProjectionLabem.

## Tři senzory: LiDAR + MPU-6050 + CSI kamera

**Nově připraveno** volitelné propojení CSI kamery do experimentálního
2D SLAMu – optický tok, sledování obrazových bodů, stav dostupnosti
kamery, vizuální změna natočení při pohledu kolmo dolů a volitelná
metrická XY predikce pouze po kalibraci měřítka. Kamera je **standardně
vypnutá**: na RPi4 může být využívaná ProjectionLabem a žádný proces
nebude automaticky ukončen, kamera se nebude násilně přebírat.

**Nejdříve diagnostika kamery bez jejího vlivu na SLAM:**

~~~bash
sudo apt install -y python3-opencv python3-picamera2
cd /opt/lidartest && git pull
sudo python3 tools/slam_server.py --imu --camera
~~~

**Pouze pro kameru namířenou přímo dolů:**

~~~bash
sudo python3 /opt/lidartest/tools/slam_server.py --imu --camera --camera-downward
~~~

Bez znalosti měřítka, výšky a natočení kamery není metrický
posun X/Y z jedné kamery spolehlivý; výchozí stav proto používá
pouze LiDAR a gyro, kamera jen sleduje obraz. Pro fyzicky
kalibrované X/Y existuje parametr `--camera-scale`, ale zatím
jej **nepoužívej bez kalibrace**. Podrobnosti:
[**docs/CAMERA_SLAM.md**](docs/CAMERA_SLAM.md).

**Stav:** offline testy Python/OpenCV, falešné posuny a rotace obrazu.
Reálný současný běh LiDAR + IMU + kamera **dosud neověřen**.

## Další kroky

1. Naměřit napětí na vstupním `+`/GND přímo u LiDARu pod zátěží (ověřit skutečných ~5 V).
2. Ověřit klidová napětí `RADAR RX` a `RADAR TX` proti GND a určit skutečný vysílací směr.
3. **Splněno:** zachycena a CRC ověřena měřicí data při 115200 baud.
4. **Splněno:** živý 360° radar i validní 10sekundový odposlech (1 921 rámců, 0 CRC chyb).
5. **Připraveno:** experimentální 2D SLAM – čeká na fyzické ověření při pohybu.
6. Upřesnit elektrické zapojení a okolnosti automatického roztočení, ideálně ověřit osciloskopem / měřením proudu.

Podrobnosti a průběžné měření: [docs/PRUBEH.md](docs/PRUBEH.md).

**Bezpečnost:** Nehledět do laserového okénka, nepřekračovat ověřené napájení, nepropojovat dvě napájecí 5V větve a neaplikovat 5 V na GPIO nebo UART data.
