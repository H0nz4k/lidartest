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

**Před zapojením:** s odpojeným napájením propojit GND LiDARu a CP2102. Ověřit, který vodič je **výstup LiDARu**, a že pro RXD CP2102 nepřekročí bezpečnou napěťovou úroveň 3,3 V (ideálně osciloskopem / logickým analyzátorem s odpovídajícím vstupem). Označení RX/TX na původní desce vysavače je z její perspektivy a není samo o sobě měřením směru. Na první test připojit jen ověřený výstup LiDARu na **RXD CP2102**; **TXD CP2102 nechat odpojený**. Pin +5V na CP2102 nespojovat s externě napájeným LiDARem.

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
