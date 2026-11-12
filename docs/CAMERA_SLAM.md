# Kamera + MPU-6050 + LiDAR: experimentální tří-senzorové mapování

## Stav 12. 11. 2026

Na **Raspberry Pi 4** přibyla volitelná CSI kamera do stávajícího
2D SLAMu ETA Raggio. Kód je připraven a prochází offline testy
se syntetickými kamerovými snímky. **Reálný společný běh všech
tří senzorů ještě není ověřený.** Produkční HUB ani jiné služby
se nasazováním nemění.

Systém používá jedinou UART čtečku pro LiDAR, samostatné vlákno pro
MPU-6050 (I²C) a **volitelné** vlákno kamery (libcamera/Picamera2).
Snímky se **neukládají na disk**. Do webu se obraz přenáší **pouze při výslovném zapnutí** `--camera-preview`; jinak se počítají pouze optické body, jejich pohyb a geometrická shoda.


## Živá CSI kamera + orientační LiDAR překryv (novinka)

Ve webu je nová záložka **Kamera + LiDAR** na adrese
`http://IP_RPI4:8766/camera`. Ve výchozím stavu kamera zůstává skrytá:
**živé JPEG snímky** jsou přístupné pouze po explicitním přepínači
`--camera-preview` současně s `--camera`.

~~~bash
sudo apt install -y python3-opencv python3-picamera2
cd /opt/lidartest && git pull
sudo python3 tools/slam_server.py --imu --camera --camera-preview
~~~

Obraz je z CSI, program ho drží pouze v RAM (cca 5 fps, 640×480).
Překryv používá **skutečně naměřené vzdálenosti** 360° LiDARu,
ale projekce měření do obrazu je **pouze orientační úhlový model
jedné vodorovné roviny**, nikoliv prostorově kalibrovaná AR:
- **Směr středu kamery v LiDARu**: orientační nastavení natočení 0–359°.
- **Horizontální zorné pole**: odhad podle objektivu, 25–110°.
- **Průhlednost vrstvy** a volitelné propojování blízkých paprsků.
- Dotykem LiDAR bodu zobrazit skutečnou vzdálenost a úhel.

Při naklápění celé konstrukce je současná **2D mapa zkreslená**:
body se stále projekují do horizontální roviny. Skutečná 3D registrace
potřebuje kalibraci kamery (intrinsiky/distortion), vzájemné polohy a
natočení kamery vůči LiDARu (extrinsiky), časovou synchronizaci a
6DoF lokalizaci. Neprezentovat tento překryv jako přesné 3D mapování.

**Bezpečnost soukromí:** JPEG endpoint `/api/camera.jpg` nemá
autentizaci. Používat **výhradně v důvěryhodné LAN**, neotvírat port
8766 do internetu a neprovozovat bez `--camera-preview`, pokud
obraz nechceš zpřístupnit. Snímky nejsou zapisovány na disk.
Kamera se nepřebírá, pokud ji používá ProjectionLab.

## Respektování ProjectionLabu

RPi4 používá CSI kameru také pro **ProjectionLab**.
Proto je kamera v SLAMu **ve výchozím stavu vypnutá**.
Volba `--camera` provede jediný pokus o otevření CSI kamery.

Pokud ji již používá ProjectionLab, aplikace zaznamená nedostupnost
a pokračuje s **LiDAR + IMU**. Neukončuje jinou službu,
nepřepisuje konfiguraci, nemění systémový camera overlay,
neodpojuje procesy a nesnaží se zařízení násilně převzít.
Pro reálné testy kamery je potřeba kamera **dobrovolně volná**;
nepřepínat ani nerestartovat ProjectionLab automaticky.

## První spuštění – pouze diagnostika kamery

Nejdřív ukončit jiný spuštěný SLAM/radar proces používající
`/dev/ttyUSB0`. Na RPi 4 nainstalovat balíky (jednorázově):

~~~bash
sudo apt install -y python3-numpy python3-serial python3-smbus python3-opencv python3-picamera2
sudo git -C /opt/lidartest pull
sudo python3 /opt/lidartest/tools/slam_server.py --imu --camera
~~~

Otevřít web:

- `http://IP_RPI4:8766/` — 2D mapa + IMU + stav CSI kamery.
- `http://IP_RPI4:8766/radar` — tradiční živý 360° radar.

Příkaz `--camera` pouze **sleduje optický tok** a diagnostické
veličiny: počet bodů, RANSAC shod, pohyb v obraze. Bez změny
algoritmu odhadu polohy. Pokud kamera není dostupná, SLAM
dál funguje jen s LiDARem a MPU-6050.

V prvních 2,5 s neotáčet konstrukcí (kalibrace gyroskopu Z).

## Zapojení kamery do odhadu pohybu

**Režim downward je vhodný jen pro kameru pevně namířenou kolmo
na podlahu** s dostatkem obrazové textury a zhruba konstantní
výškou nad podlahou. NENÍ bezpečný geometrický model pro kameru
mířící dopředu či do strany.

V tomto specifickém uspořádání lze nejprve zapnout **pouze
relativní vizuální změnu yaw** (kamera + gyro Z + LiDAR):

~~~bash
sudo python3 /opt/lidartest/tools/slam_server.py --imu --camera --camera-downward
~~~

Algoritmus:

1. OpenCV detekuje až 240 rohů v obraze a Lucas–Kanade optickým
   tokem je sleduje do dalšího snímku.
2. RANSAC přizpůsobí přibližnou 2D transformaci, odmítá nekvalitní
   nebo příliš velké změny.
3. Z rotace obrazu při **pohledu kolmo dolů** vzniká relativní
   vizuální yaw.
4. Pokud gyro Z a vizuální yaw souhlasí do 10°, použije se slabě
   vážený vizuální příspěvek (20 %) k predikci.
   Pokud IMU chybí, může vizuální yaw dodat predikci samostatně.
   **Konečnou korekci geometrie určuje LiDAR scan matching.**
5. Při ztrátě obrazu / malé shodě funkce pokračuje jen s IMU
   a LiDARem. Stav je vidět ve webové diagnostice.

Znaménko yaw závisí na fyzické orientaci obrazového senzoru
vůči konstrukci; pokud reakce neodpovídá známému směru rotace,
lze zkusit `--camera-yaw-sign -1`. Není vhodné měnit znaménko
bez pozorování skutečného známého směru pohybu.

### Metrická poloha X/Y z kamery: až po kalibraci

**Z jedné obyčejné kamery nedostaneme obecně metrické X/Y
bez kalibrace a předpokladů o scéně.** Proto je kamerový
XY prior **výchozím nastavením vypnutý**.

Pokud je kamera pevně namířená dolů, má stejnoměrnou výšku
a podlaha je přibližně rovná, lze kalibrovat měřítko takto:

1. Spustit downward režim bez `--camera-scale`.
2. Ve webu odečíst hodnotu **Pohyb v obraze** (px).
3. Pomalu bez otáčení přesunout celou základnu o známou délku,
   například 0,50 m.
4. Spočítat `scale = 0.50 / rozdíl_px` v metrech na pixel;
   ověřit měřítko dalším známým přesunem.
5. Až po ověření použít volbu `--camera-scale <m_na_pixel>`
   a případnou orientaci `--camera-yaw-offset <stupne>`.

Příklad syntaxe (čísla jen ilustrativní, **nepoužívat bez kalibrace**):

~~~bash
sudo python3 /opt/lidartest/tools/slam_server.py --imu --camera --camera-downward --camera-scale 0.002 --camera-yaw-offset 0
~~~

Z pohybu kamery se pak vytváří **slabá XY predikce** s limitem
0,4 m mezi přijatými skeny. LiDAR ji musí potvrdit scan matchingem.
Při změně výšky či náklonu je měřítko nepřesné a je vhodné
se vrátit do režimu bez `--camera-scale`.

## Výkon a omezení

- Požadavky: `python3-picamera2`, `python3-opencv`, NumPy; kamera
  se snímá přibližně 10 fps při 640×480, CPU RPi4 je omezené.
- Web zobrazuje stav kamery, počet sledovaných/inlier bodů,
  relativní otočení a informaci, zda bylo využito ve fúzi.
- Čas je aproximován monotónními hodinami při dokončení frame /
  LiDAR otáčky. **Není hardwarově synchronizovaný**; rychlý pohyb
  může být zkreslený.
- Monokulární optický tok může selhat na prázdné/lesklé podlaze,
  při pohybujících se objektech, rozmazání a při změně osvětlení.
- U kamery namířené dopředu funguje zatím jen diagnostika optického
  toku; robustní metrická visual odometry vyžaduje další kalibraci,
  perspektivní/3D model a nastavení extrinsik.
- Bez loop closure a de-skew se při delším pohybu může mapa
  výrazně rozcházet. Zde nejde o plnohodnotný Fast-LIO2.

## Soubory a ověření

- `tools/camera_vo.py` – optický tok, detekce/shoda rohů,
  pozorování relativního pohybu a bezpečné připojení kamery.
- `tools/slam_core.py` – triangulace relativního yaw z gyra
  a obrazu do predikce pro LiDAR scan matcher; opt-in XY prior.
- `tools/slam_server.py` – oddělená čtecí vlákna a společná
  telemetrie v `/api/slam`.
- `web/slam.html` – grafická diagnostika všech tří senzorů.
- `tools/test_camera_vo.py` – syntetické překlady a rotace obrazu,
  kontrola režimu monitor, nekvalitních snímků a fallbacku.

Základní jednotkové testy na GitHubu CI jsou automatické.
**Bez fyzického společného testu není potvrzené znaménko
kamerového yaw ani skutečný přesun XY**, proto jsou pokročilé
režimy explicitně volitelné.

### Bezpečnost

Nepropojovat napájení LiDARu ani 5V větev MPU se vstupy GPIO.
MPU breakout vyžaduje vůči RPi maximálně 3,3 V na SDA/SCL.
LiDAR čteme pasivně. Webové UI běží bez autentizace – pouze
v důvěryhodné LAN, nepublikovat na internet.
