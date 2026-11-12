# Experimentální 3D skener: pevná základna + naklápěcí LiDAR

## Princip

Tvoje konstrukce řeší hlavní problém ručního 3D skenování:
**spodní část musí být pevná vůči zemi**. Pak můžeme nastavit
posun základny X=Y=Z=0 a měnit pouze úhel naklápění horní části.

- **Základna**: nepohybuje se, nese mechanický pivot a RPi4.
- **Horní naklápěcí část**: celý LiDAR ETA Raggio, pevně připojené
  MPU-6050 a případně CSI kamera; LiDAR dále sám rotuje 360°.
- **Osa**: jedna pevná vodorovná osa v ložiskách, ideálně procházející
  středem měření LiDARu, aby se minimalizoval posun při náklonu.
- MPU-6050 musí být **na pohyblivé sestavě**, ne na stojící základně.
  Gyro měří krátkodobou úhlovou rychlost, akcelerometr stabilizuje
  roll/pitch podle gravitace (komplementární filtr).

## Geometrie

Jeden 2D bod LiDARu je lokálně (d·sin θ, d·cos θ, 0).
Známý náklon α kolem osy Y/X ho převede pomocí rotace do XYZ.
Pokud je střed LiDARu mimo osu naklápění, musí se započíst i
známý vektor od **pivotu do středu senzoru** v metrech. Tato
kompenzace je ve 3D programu volitelná.

**Nejde o obecný 6DoF SLAM.** Pevný pivot je zásadní předpoklad
a pohyby X/Y/Z celé základny se nesmějí objevit. Ani samotná IMU
nedokáže trvale přesně odhadovat metrický posun.

## První spuštění na Raspberry Pi 4

Nejprve ukonči běžný 2D SLAM pomocí Ctrl+C, protože oba procesy
nemohou současně používat stejný /dev/ttyUSB0. Kamera a ProjectionLab
zůstávají bez zásahu.

~~~bash
sudo apt install -y python3-numpy python3-serial python3-smbus
sudo git -C /opt/lidartest pull
sudo python3 /opt/lidartest/tools/scan3d_server.py --tilt-axis y
~~~

Otevři **http://IP_RPI4:8767/**. Nová stránka ukáže rotovatelné
3D mračno bodů, stav gyroskopu, celkový počet bodů a možnost
**Stáhnout 3D mračno (PLY)**. PLY lze otevřít v CloudCompare či
MeshLabu. Uloží XYZ v metrech a intenzitu, zatím ne RGB z kamery.

## Jak skenovat

1. Základnu skutečně nepohybovat. LiDAR nastavit vodorovně.
2. Spustit server a prvních **2,5 sekundy nehybat** senzorem:
   MPU kalibruje bias gyra a výchozí orientaci vůči gravitaci.
3. Po prvním 360° skenu se objeví první vodorovný prstenec.
4. Pohnout horní částí o **2–5°** kolem pevné osy, na každé
   poloze vydržet několik desetin sekundy a opakovat, například
   od −30° do +30°. Omezit rozsah podle mechaniky.
5. Pozorovat postupně vznikající mračno a uložit PLY.
   Reset vymaže pouze lokální mračno a nastaví nový počátek
   úhlu u dalšího platného skenu.

**Zvlášť důležité:** Během jedné otáčky LiDARu mohou jednotlivé
paprsky měřit při různém náklonu. První verze používá orientaci
přibližně **ve středu otáčky**, nikoli individuální de-skew.
Proto jsou lepší krátké zastávky v každém úhlu než rychlý
plynulý pohyb.

## Nastavení osy

Předozadní naklánění zpravidla odpovídá MPU pitch (Y).
Výchozí volba je osa Y. Pokud se při tvém mechanickém
uložení mění naopak roll, přepni osu X a podle potřeby znaménko:

~~~bash
sudo python3 /opt/lidartest/tools/scan3d_server.py --tilt-axis x --tilt-sign -1 --tilt-step-deg 3
~~~

Při začátku drž LiDAR skutečně vodorovně. Výchozí úhel se
nastaví na první přijatý platný sken.

Pro kompenzaci **měřeného** posunu osy otáčení vůči středu
LiDARu lze zadat metrický pivot offset; dokud není změřený,
ponechej nulu:

~~~bash
sudo python3 /opt/lidartest/tools/scan3d_server.py --tilt-axis y --pivot-offset-x 0 --pivot-offset-y 0 --pivot-offset-z 0
~~~

## Ochranné limity a přesnost

- Přidávají se jen skeny při změně úhlu nejméně **2°**
  (přepínač tilt-step-deg).
- Při náklonu druhé než zvolené osy přes **12°** se sken odmítá.
- Výkyv přes **85°** se ignoruje.
- Při opětovné kalibraci IMU po I²C výpadku se skeny nového
  časového období nepřidají ke starým: nejprve ulož PLY a resetuj.
- Výchozí paměťový limit je **160 000 bodů**.
- MPU dává orientační roll/pitch, ne geodeticky přesnou polohu.
  Osy MPU vůči osám LiDARu a skutečný pivot musejí být
  mechanicky sladěny; kamera není pro první fázi nutná.
- Web na portu 8767 nemá přihlášení: používat pouze v LAN.

## Soubory

- tools/imu_fusion.py: gyro XYZ + akcelerometr, kalibrovaný roll/pitch.
- tools/cloud3d.py: přepočet paprsků do XYZ, model pivotu, PLY.
- tools/scan3d_server.py: izolovaný UART+IMU webový server, port 8767.
- web/cloud3d.html: interaktivní 3D Canvas, export a reset.
- tools/test_cloud3d.py: testy geometrie, omezení a PLY.

**Stav:** offline automatické testy probíhají na GitHubu; funkčnost
skutečné konstrukce, orientace os a přesnost 3D mračna zatím
nebyla ověřena v reálném pohybu.
