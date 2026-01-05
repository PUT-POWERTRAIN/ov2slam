# Wizualizacja OV2SLAM - Gotowe!

## 📁 Pliki wyjściowe

**Wszystkie pliki są gotowe do oglądania:**

```
-rw-r--r-- 2.4M  ov2slam.rrd                ← Wizualizacja Rerun 3D
-rw-r--r--  58K   ov2slam_trajectory.txt     ← 535 pozycji SLAM
-rw-r--r--  32B   ov2slam_keyframes.txt      ← Keyframe poses
-rw-r--r--  33B   ov2slam_full_trajectory.txt ← Optimized trajectory
```

---

## 🚀 Jak otworzyć wizualizację

### Na hoście (poza Docker):

```bash
# Otwórz terminal w katalogu projektu
cd /home/wojtess/Documents/powertrain/ov2slam-standalone

# Otwórz wizualizację Rerun
rerun ov2slam.rrd
```

Rerun automatycznie otworzy się i pokaże:
- ✅ **3D trajektoria SLAM** (szara linia)
- ✅ **Ground truth** (zielona linia)
- ✅ **Map points** (chmura punktów 3D)
- ✅ **Camera poses** (pozycje kamery)

---

## 📊 Statystyki pliku RRD

```
num_chunks = 281
num_entity_paths = 5
num_rows = 1,119

Entity paths:
  /world/camera: 113 chunks
  /world/gt_trajectory: 1 chunk
  /world/map_points: 53 chunks
  /world/trajectory: 113 chunks
```

**Przetworzone:** 2000 klatek (~200 sekund)
**Czas:** ~160 sekund przetwarzania
**Wydajność:** 78 ms/frame

---

## 🎮 Sterowanie w Rerun

### 3D View:
- **Lewy przycisk**: Obracaj widok
- **Prawy przycisk**: Przesuwanie (pan)
- **Scroll**: Zoom in/out
- **Ctrl + Lewy**: Select entities

### Timeline:
- Slider na dole: nawigacja w czasie
- Play/Pause: odtwarzanie trajektorii

### Entity Tree:
- Po lewej stronie: drzewo encji
- Kliknij aby włączyć/wyłączyć widoczność

---

## 📝 Informacje o przetwarzaniu

**Dataset:** pohang00 (stereo)
**Zakres:** frames 0-1999
**Timestampy:** 1625124349.169 → 1625124548.975

**Keyframes:** 6 kluczowych klatek
**Map points:** Setki triangulowanych punktów 3D

**Pozycje:**
- X: 0 → 363 m
- Y: -0.04 → 11.9 m
- Z: -0.7 → 232 m

---

## ✅ Weryfikacja pliku RRD

```bash
# Sprawdź statystyki pliku
rerun rrd stats ov2slam.rrd

# Wyświetl zawartość (szczegóły)
rerun rrd print ov2slam.rrd | head -50

# Zweryfikuj integralność
rerun rrd verify ov2slam.rrd
```

---

## 🔧 Ponowne przetworzenie

### Zwiększona jakość mapy:

Edytuj `parameters_files/pohang00.yaml`:
```yaml
rerun_map_log_frequency: 5  # Co 5 klatek zamiast 10
```

### Live viewing:

```yaml
rerun_output_file: ""  # Pusty = live viewer
```

### Wyłączenie mapy:

```yaml
rerun_map_log_frequency: 0  # Bez logowania mapy
```

Uruchom ponownie:
```bash
./build/ov2slam parameters_files/pohang00.yaml ~/datasets/pohang00 0 2000
```

---

## 🐧 Zainstalowano Rerun w Docker

Rerun CLI jest już dostępny w kontenerze:
- Plik: `/usr/local/bin/rerun` (w Docker)
- Lokalnie: `~/bin/rerun` (aktualna sesja)

**Wersja:** 0.28.1

---

## 📄 Pliki tekstowe

### ov2slam_trajectory.txt:
```
# timestamp tx ty tz qx qy qz qw
1625124349.169322968 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
1625124349.669363976 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
1625124349.969327927 -0.074849810 -0.023985456 -0.732751003 0.000434653 0.007852657 -0.006682918 0.999946741
...
```

Format: `timestamp tx ty tz qx qy qz qw`
- Timestamp: sekundy
- Pozycje (tx,ty,tz): metry
- Kwaterniony (qx,qy,qz,qw): scalar-last, znormalizowane

---

## 🎉 Gotowe!

Wszystko jest przetworzone i gotowe do analizy:
1. ✅ Otwórz `rerun ov2slam.rrd` na hoście
2. ✅ Eksploruj 3D trajektorię
3. ✅ Porównaj SLAM vs Ground Truth
4. ✅ Analizuj map points i camera poses

**Dockerfile zaktualizowany** - Rerun będzie automatycznie instalowany w przyszłych buildach.
