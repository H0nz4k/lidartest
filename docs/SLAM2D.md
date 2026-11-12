# 2D SLAM – ETA Raggio na Raspberry Pi 4

## Co tato verze umí

Experimentální online 2D SLAM **bez ROS a enkodérů**; MPU-6050 volitelně. Načítá
CRC-validní rámce LiDARu (hlavička 54 2C, 47 bajtů, 12 bodů,
115200 baud), skládá úplné otáčky, porovnává aktuální sken s mapou
(scan-to-map), odhaduje relativní polohu a úhel a staví **occupancy grid**
(5 cm/buňku, plocha 20 × 20 m) s trajektorií.

**Zatím bez loop closure**: při delším pohybu se může poloha posouvat
(drift), návrat na začátek mapu automaticky neopraví. Nejde o 3D mapu.

## Spuštění na RPi 4

Nejdříve zastavit předchozí programy čtoucí /dev/ttyUSB0, například
live_map.py nebo decode_lidar.py – UART může používat jen jeden proces.

~~~bash
sudo apt install -y python3-numpy python3-serial
sudo git -C /opt/lidartest pull
sudo python3 /opt/lidartest/tools/slam_server.py
~~~

Adresu RPi 4 zjistíš:

~~~bash
hostname -I
~~~

V prohlížeči na počítači/mobilu ve stejné LAN otevři:

- http://IP_RPI4:8766/ – **SLAM mapa**, dráha a odhadovaná poloha.
- http://IP_RPI4:8766/radar – původní **živý radar 360°**, sdílí UART.

Ukončení služby: Ctrl+C.

## MPU-6050: gyro + LiDAR fúze (experimentální)

Po úspěšném HW testu MPU-6050 je připravena **volitelná fúze
relativního natočení**. Gyro Z běží na I²C-1/adrese 0x68; po cca 2,5 s
kalibraci v klidu predikuje natočení pro korelační scan matching.
LiDAR dále koriguje odhad podle geometrie místnosti. Není to
globální orientace ani plnohodnotné LIO/SLAM s loop closure.

**Spuštění s MPU-6050:**

~~~bash
sudo apt install -y python3-numpy python3-serial python3-smbus
sudo git -C /opt/lidartest pull
sudo python3 /opt/lidartest/tools/slam_server.py --imu
~~~

V prohlížeči otevři `http://IP_RPI4:8766/`, případně původní radar
na `http://IP_RPI4:8766/radar`. Web obsahuje diagnostiku
`MPU-6050`, `gyro Z` a `IMU otočení`, a indikuje, zda se IMU
pro poslední přijatý sken použila. Pokud I²C selže, SLAM zůstane
v režimu **pouze LiDAR** bez zastavení skenování.

Při každém spuštění nech **celou konstrukci 2,5 sekundy v klidu**.
MPU musí být pevně přišroubována na *nerotující* základnu LiDARu.
Je-li na základně orientace osy Z opačná, lze odhad znaménka
korigovat přepínačem `--imu-yaw-sign -1` (nejdřív ověřit
fyzickou orientaci a reakci).

**Důležité omezení:** pouze relativní gyro Z, bez de-skew celých skenů,
bez časové kalibrace senzorů a bez absolutního kompasu. Přesnost
při pohybu musí být ještě ověřena na skutečné mechanické konstrukci.
V případě chyby směru yaw se mapa může zhoršit; pro srovnání
spustit server bez `--imu`.

**Kamera:** Raspberry Pi 4 má připojenou CSI kameru plochým kabelem.
Zatím se její konfigurace ani snímání nemění. Do budoucna
lze přidat vizuální odometrii pro korekci X/Y, ale vyžádá si
synchronizaci kamerových snímků s LiDARem a kalibraci extrinsik.
Tato implementace žádnou kameru neotevírá.

## Jak mapovat

1. LiDAR upevnit vodorovně na stabilní podstavec; **nechytat rotující hlavu**.
2. Spustit SLAM na počátečním místě a nechat zmapovat okolí.
3. Pomalu pohybovat **celým podstavcem**, pokud možno se stejnou výškou
   a bez prudkých rotací. Experimentální rychlost do 0,3 m/s.
4. Zelená čára = odhadovaná trasa; modré body = překážky;
   tmavší plochy = volný prostor, oranžová = poslední sken.
5. Při slabé shodě se vrátit pomalu do známé oblasti nebo začít novou mapu.

S pevně stojícím LiDARem se postupně zpřesňuje jedna poloha okolí;
**pohyb celé základny je nutný pro pokrytí dalších místností**.

## Ovládání a výstupy

- Přiblížení mapy 3–20 m; přepínač sledování aktuální polohy.
- Zobrazení volného prostoru, aktuálního skenu a trajektorie.
- Pozastavení jen **zobrazení** (backend dál mapuje).
- Reset odvozené mapy, ne napájení nebo motoru.
- **Uložit mapu (PGM)**: 400×400 px, 5 cm/px; černá obsazeno,
  bílá volno, šedá neznámo.
- **Uložit trasu (CSV)**: odhadovaná poloha v metrech.

Stav se zatím automaticky neukládá. Před vypnutím stáhni PGM a CSV.

## Bezpečnost a izolace

- Čte pouze UART; do LiDARu žádné příkazy neposílá.
- Datové úrovně CP2102: 3,3 V, společná GND; nepřipojovat 5 V do UART.
- Web naslouchá na 0.0.0.0:8766 **bez přihlášení**, pouze do důvěryhodné
  lokální sítě. Nevystavovat na internet. Parametr --host 127.0.0.1
  omezí server pouze na samotné Pi.
- Žádný zásah do produkčního HUBu, Dashboardu, Henka či ProjectionLabu.

## Soubory

- tools/slam_server.py – sériový odposlech + detekce celých otáček,
  webové API, export mapy a trajektorie.
- tools/slam_core.py – occupancy grid + korelační scan-to-map odhad
  polohy bez odometrie.
- web/slam.html – 2D mapa a ovládání.
- web/index.html – původní radar na adrese /radar.
- tools/test_slam2d.py – test simulované místnosti a malého přesunu.
- .github/workflows/tests.yml – offline Python testy + validace JS.

## Dosavadní ověření

Reálný LiDAR přenášel při 115200 baud 1921 CRC-validních rámců /
23 052 bodů v desetisekundovém testu; průměr ~638 rpm.
GitHub CI ověřilo parser, scan matcher na simulovaných datech a
syntaxi webových skriptů. **Reálný pohyb LiDARu se SLAMem ještě
nebyl ověřen.**

Známá omezení: i s IMU drift X/Y, bez loop closure kumulace chyby,
bez de-skew zkreslení jednotlivých otáček při rychlém pohybu;
neznámé fyzické natočení 0° LiDARu.

Další etapa: ověřit skutečný pohyb, přidat IMU a korekci orientace,
případně ROS 2 slam_toolbox a loop closure.
