# Proč aktuální 2D SLAM netvoří 3D a jak z ETA Raggio získat 3D

**Stav:** návrh další experimentální etapy, nikoli hotové 3D mapování.

## Současná situace

- ETA Raggio rotuje a měří vzdálenosti v **jedné rovině**; každý paprsek má úhel v této rovině a vzdálenost.
- Aktuální `slam_core.py` používá odhad polohy pouze **X, Y a yaw**. Všechna měření vkládá do 2D occupancy grid (mapa se souřadnicemi X/Y).
- MPU-6050 dodává úhlovou rychlost a zrychlení, ale sama neurčí spolehlivě absolutní polohu X/Y/Z (dvojitá integrace zrychlení výrazně driftuje). Pitch/roll lze korigovat gravitačním vektorem, zatímco yaw bez externí reference driftuje.
- CSI kamera ve stávajícím režimu optického toku sama neposkytuje kompletní metrickou 6DoF polohu. Monokulární odometrie vyžaduje kalibraci měřítka a orientace vůči LiDARu.

## Jak vznikne 3D bod

Lokální paprsek senzoru je `p_local = (d*sin(theta), d*cos(theta), 0)`. Chceme-li 3D mapu, pro **každé měření a jeho čas** musíme znát orientaci senzoru `R(roll,pitch,yaw)` a polohu `t=(x,y,z)`:

`p_world = R * p_local + t`

Jakmile senzorem nakloníme, paprsky už neleží v horizontální rovině. Při přesunu skeneru do jiné výšky vznikají další řezy. Skeny je pak možné ukládat jako 3D mračno bodů (PLY) – **pouze pokud jsou polohy a orientace dostatečně přesně známé**.

## Doporučené etapy

1. **Stacionární 3D skener (nejméně rizikový):** umístit celý LiDAR na mechaniku s řízeným naklápěním kolem vodorovné osy. Potřebujeme *známý úhel* naklopení (servo s kalibrací nebo krokový motor + referenční spínač) a kalibrované umístění osy vůči měřicímu středu LiDARu. Pro každou polohu zaznamenat 360° sken a převést ho do 3D mračna bodů; výstup PLY.
2. **Ruční skenovací režim:** kombinovat gyroskop, akcelerometr, kameru a geometrii LiDARu v 6DoF odometrii. Vyznačit časovou synchronizaci jednotlivých senzorů, intrinsic/extrinsic kalibraci kamery a metrické měřítko. Teprve potom smysluplně skládat 3D mračno při chůzi.
3. **Korekce trajektorie:** doplnit loop closure / globální optimalizaci a odstraňování zkreslení skenu během pohybu.

## Omezení

- Samotné libovolné otáčení skenerem **nedokazuje** správnou 3D polohu; při chybě v pose vzniknou zdvojené stěny a deformované body.
- Jen posun ve vodorovné rovině bez náklonu/výškové změny vytváří stále měření ve stejné rovině.
- Stacionární naklápění s enkodérem/známým úhlem je dobrý první 3D experiment; pro handheld 3D je třeba podstatně robustnější lokalizace.
- Produkční HUB se pro tuto etapu nepoužívá; vše testovat na RPi 4.

## První praktický experiment

Vyrobit pevnou stabilní konstrukci a mechaniku naklápění, která bezpečně udrží *celý* LiDAR s rotující hlavou; potom přidat záznam `(timestamp, theta, distance, tilt_angle)`, transformaci do XYZ a export PLY. Připojenou CSI kameru neovlivňovat, pokud je obsazená ProjectionLabem.
