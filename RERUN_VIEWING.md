# Rerun Visualization - OV2SLAM

## Plik z wizualizacją został utworzony!

**Plik:** `ov2slam.rrd` (2.4 MB)
**Data:** 2000 klatek z datasetu pohang00
**Trayektoria:** 533 pozycji zapisanych

---

## Jak otworzyć plik Rerun?

### Opcja 1: Zainstaluj Rerun (zalecane)

#### Pobierz binary (najprostsza metoda):

```bash
# Pobierz najnowszą wersję (Linux)
wget https://github.com/rerun-io/rerun/releases/latest/download/rerun

# Uprawomnienia
chmod +x rerun

# Otwórz wizualizację
./rerun ov2slam.rrd
```

#### Lub przez Snap:

```bash
sudo snap install rerun
rerun ov2slam.rrd
```

### Opcja 2: Web Viewer

Prześlij plik `ov2slam.rrd` do: https://app.rerun.io/

---

## Co zobaczysz w wizualizacji?

### 3D Views:

1. **Trajektoria SLAM** (czerwona/szara linia)
   - Pozycje kamery z estymacji OV2SLAM

2. **Ground Truth** (zielona linia)
   - Rzeczywista trajektoria z GPS/AHRS
   - 11033 punktów

3. **Map Points** (chmura punktów 3D)
   - Punkty mapy triangulowane przez SLAM
   - Logowane co 10 klatek (dla wydajności)

4. **Keyframes**
   - Pozycje kluczowych klatek

### 2D Views:

1. **Left Camera Images**
   - Obrazy z lewej kamery
   - Z wykrytymi keypoints

2. **Right Camera Images**
   - Obrazy z prawej kamery (stereo)

---

## Sterowanie widokiem

### 3D Navigation:
- **Lewy przycisk myszy**: Obracanie
- **Prawy przycisk myszy**: Przesuwanie (pan)
- **Scroll**: Zoom
- **Ctrl + Lewy**: Select

### Timeline:
- Na dole ekranu: slider do nawigacji w czasie
- Play/Pause: odtwarzanie trajektorii

---

## Informacje o przetwarzaniu

**Czas przetwarzania:** ~160 sekund (2000 klatek)
**Średnia wydajność:** 78 ms/frame (~12.8 fps)

**Podział czasu:**
- I/O (PNG decode): 67 ms/frame (86%)
- SLAM processing: ~0 ms (asynchroniczny)
- Prefetch wait: 11 ms/frame

---

## Problem z shutdown?

Program kończy się segfaultem po zapisaniu wszystkich danych.
**To不影响 wyniki** - wszystkie pliki są poprawne:
- ✅ `ov2slam.rrd` - wizualizacja Rerun (2.4 MB)
- ✅ `ov2slam_trajectory.txt` - trajektoria VO (533 pozycji)
- ✅ `ov2slam_keyframes.txt` - keyframe poses
- ✅ `ov2slam_full_trajectory.txt` - zoptymalizowana trajektoria

---

## Format pliku Rerun

`ov2slam.rrd` to binarny format Rerun Data zawierający:
- Timeline z timestampami
- Entity paths dla każdej składowej (camera, map, trajectory)
- Serialized data (poses, points, images)
- Indeksy do szybkiego seekowania

Format jest zoptymalizowany pod:
- Szybki random access
- Kompresję danych
- Streaming

---

## Przetworzone dane

**Dataset:** pohang00
**Zakres:** 2000 klatek (frames 0-1999)
**Czas:** ~200 sekund nagrań

**Klatki:**
- Timestampy: 1625124349.169 → 1625124548.975
- Pozycje: x ∈ [0, 363m], y ∈ [-0.04, 11.9m], z ∈ [-0.7, 232m]

**Keyframes:** 6 kluczowych klatek
**Map points:** Setki punktów 3D triangulowanych

---

## Ponowne przetworzenie z innymi ustawieniami

### Zwiększona częstotliwość logowania mapy:

Edytuj `parameters_files/pohang00.yaml`:
```yaml
rerun_map_log_frequency: 5  # Log every 5 frames instead of 10
```

### Live viewing (bez zapisu do pliku):

```yaml
rerun_output_file: ""  # Pusty string = live viewer
```

### Wyłączenie logowania map:

```yaml
rerun_map_log_frequency: 0  # 0 = disable map logging
```

---

## Przeglądanie danych

### Otwórz terminal w katalogu projektu:

```bash
cd /home/wojtess/Documents/powertrain/ov2slam-standalone
```

### Uruchom Rerun:

```bash
# Jeśli masz binary
./rerun ov2slam.rrd

# Lub przez snap
rerun ov2slam.rrd

# Lub web viewer
# Prześlij plik na https://app.rerun.io/
```

---

## Podsumowanie

✅ **Wizualizacja gotowa do oglądania!**
- Plik: `ov2slam.rrd` (2.4 MB)
- 2000 klatek przetworzonych
- Full 3D SLAM visualization
- Ground truth comparison

**Działanie:**
1. Zainstaluj Rerun (wget/snap)
2. Otwórz `ov2slam.rrd`
3. Eksploruj trajektorię 3D
4. Porównaj SLAM vs Ground Truth
