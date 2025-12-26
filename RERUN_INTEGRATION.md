# Rerun Visualization Integration

OV2SLAM standalone teraz wspiera wizualizację 3D w czasie rzeczywistym używając [Rerun](https://rerun.io/).

## Co zostało zaimplementowane

### Wizualizowane elementy:
1. **Camera trajectory** - 3D ścieżka ruchu kamery
2. **Current camera pose** - aktualna pozycja jako czerwona kropka
3. **Map points** - chmura punktów 3D z kolorami
4. **Keyframes** - frusta kamery w pozycjach kluczowych klatek
5. **Camera images** - obrazy z kamery do odtwarzania
6. **Timeline scrubbing** - analiza frame-po-frame

## Instalacja Rerun SDK

Rerun C++ SDK musi być zainstalowane w systemie żeby używać wizualizacji.

### Metoda 1: Z pakietu (zalecana)

```bash
# Pobierz i zainstaluj Rerun C++ SDK
wget https://github.com/rerun-io/rerun/releases/download/v0.5.1/rerun_cpp_sdk.zip
unzip rerun_cpp_sdk.zip
cd rerun_cpp_sdk
./install.sh
```

### Metoda 2: Zbuduj ze źródeł

```bash
git clone https://github.com/rerun-io/rerun.git
cd rerun
cargo install rerun-cpp
```

Więcej informacji: https://rerun.io/docs/getting-started/quick-start/cpp

## Budowanie z Rerun

```bash
# Konfiguruj z Rerun
cmake -B build_standalone -DENABLE_PROFILING=ON -DENABLE_RERUN=ON

# Zbuduj
cmake --build build_standalone -j4
```

Jeśli Rerun nie jest zainstalowane, build się powiedzie (wyłączy Rerun automatycznie).

## Konfiguracja

Dodaj do pliku YAML (np. `parameters_files/pohang00.yaml`):

```yaml
# Rerun visualization parameters
rerun_map_log_frequency: 10  # Log map points every N frames (0 = disable)
```

## Uruchamianie

```bash
./build_standalone/ov2slam_standalone parameters_files/pohang00.yaml ~/datasets/pohang00 100
```

Rerun viewer otworzy się automatycznie i pokaże:
- Trajektorię w czasie rzeczywistym
- Chmurę punktów 3D
- Frustum kamer w pozycjach keyframes
- Obrazy z kamer

## Wyłączanie wizualizacji

Żeby kompilować bez Rerun:

```bash
cmake -B build_standalone -DENABLE_PROFILING=ON -DENABLE_RERUN=OFF
```

Lub po prostu nie instaluj Rerun SDK - CMake sam wyłączy wizualizację.

## Performance

- ** Bez Rerun**: ~57 FPS (pełna wydajność)
- **Z Rerun**: ~55-57 FPS (minimalny spadek, zależy od hardware)
- **Map logging frequency**: 10 = co 10 klatek (zmniejsza overhead)

## Implementacja

### Timeline Synchronization
Wszystkie elementy wizualizacji są teraz zsynchronizowane na timeline:
- `logPose()`, `logMapPoints()`, `logKeyframe()` - wszystkie używają `set_time_seconds("frame", time)`
- Pozwala to na scrubbing timeline w Rerun viewer

### Image Logging API
Używamy natywnego API Rerun dla logowania obrazów:
- `rerun::Image::from_rgb24()` dla obrazów RGB
- Zero-copy z `rerun::borrow()` dla optymalnej wydajności

## Pliki zmodyfikowane

### Nowe pliki:
- `include/rerun_visualizer.hpp` - Klasa RerunVisualizer
- `src/rerun_visualizer.cpp` - Implementacja (~130 linii)

### Zmodyfikowane pliki:
- `CMakeLists_standalone.txt` - Dodane find_package(rerun)
- `src/main_standalone.cpp` - Tworzenie RerunVisualizer
- `include/ov2slam.hpp` - Dodany prviz_ member
- `src/ov2slam.cpp` - Wywołania logowania
- `include/slam_params.hpp` - Parametr rerun_map_log_frequency_
- `src/slam_params.cpp` - Wczytywanie parametru z YAML
- `parameters_files/pohang00.yaml` - Dodany rerun_map_log_frequency

## Rozwiązywanie problemów

### Błąd: "Rerun SDK not found"

**Rozwiązanie**: Zainstaluj Rerun SDK (patrz wyżej).

### Błąd 404 podczas cmake

**Rozwiązanie**: CMake automatycznie wyłączy Rerun jeśli go nie znajdzie.

### Brak wizualizacji

**Sprawdź**:
1. Czy Rerun SDK jest zainstalowane?
2. Czy build był z `-DENABLE_RERUN=ON`?
3. Czy w konsoli widzisz "[Rerun] Visualization enabled"?
