# Naukowa Analiza: Z-Axis Explosion @ Frame 12262

**Data**: 2025-12-29
**Metodologia**: Iteracyjne logging z OW2SLAM
**Cel**: Zrozumieć DLACZEGO PnP zbiega do ekstremalnych wartości

---

## Executive Summary

**Kluczowe znalezisko**: Z-axis explosion wystąpił @ **frame 12262**, NIE @ frame 14657!

### Timeline explozji:

| Frame | Z (World) | Z (Camera) | nb_3d | Inliers | Outliers | Status |
|-------|-----------|------------|-------|---------|----------|--------|
| 12258 | 556.78 | 556.78 | 27 | 27 | 0 | ✅ Normal |
| 12259 | 556.70 | 564.17 | 44 | 44 | 0 | ✅ Normal |
| **12260** | 582.34 | 602.43 | 36 | **1** | **35** | ⚠️ First degradation |
| 12261 | 556.29 | 549.18 | 18 | 18 | 0 | ⚠️ Weak tracking |
| **12262** | **-5433.64** | **509.98** | 31 | **0** | **31** | 💥 EXPLOSION |
| 12263 | -11414.5 | - | 0 → 0 | - | - | ❌ Lost |
| 12264 | -17400.5 | - | 0 | - | - | ❌ Lost |
| 12265+ | -17400.x | - | 22-30 | yes | 0-1 | ⚠️ Stabilized @ wrong pose |

---

## Analiza Szczegółowa

### Krok 1: Normal tracking (Frames 12220-12249)

```
[POSE_PRED] frame=12220 Z=559.437 nb_3d=71
[PNP_RESULT] frame=12220 Z=559.634 inliers=48 outliers=0 success=1
```

**Charakterystyka**:
- Z stabilny: 558-560m
- nb_3d: 30-71 (dobry tracking)
- Inliers: 30-48 (100% success rate)
- Pętla FEEDBACK: pozytywna - stabilny tracking → stabilna pozycja → dobry tracking

### Krok 2: Pierwsza degradacja (Frame 12260)

```
[POSE_PRED] frame=12260 Z=556.621 nb_3d=41
[PNP_INIT] Z_init=602.433 nb_pts=36
[PNP_INLIERS] total=36 inliers=1 outliers=35 Z_final=582.509
[PNP_RESULT] frame=12260 Z=582.335 inliers=1 outliers=35 success=1
```

**Charakterystyka**:
- **KRYTYCZNE**: Tylko 1 inlier z 36 keypoints (2.8% inlier ratio!)
- Z_init (motion model) = 602.433m
- Z_final (PnP) = 582.509m
- Różnica motion model vs PnP: ~20m
- **Walidacja zaakceptowała** mimo 35 outliers!

**Pytanie**: Dlaczego tylko 1 inlier?
- **Hipoteza 1**: Motion model dał złą initializację (602m vs 556m)
- **Hipoteza 2**: Scene change (np. szybki ruch, oświetlenie)
- **Hipoteza 3**: Akumulacja błędów w map points

### Krok 3: Słaby tracking (Frame 12261)

```
[POSE_PRED] frame=12261 Z=648.347 nb_3d=36
[PNP_INIT] Z_init=549.18 nb_pts=18
[PNP_INLIERS] total=18 inliers=18 outliers=0 Z_final=556.293
[PNP_RESULT] frame=12261 Z=556.293 inliers=18 outliers=0 success=1
```

**Charakterystyka**:
- Motion model: Z=648.347m (skok o ~92m od frame 12260!)
- PnP initialization: Z_init=549.18m
- PnP result: Z=556.293m
- nb_3d spadło z 36 → 18 (50% dropout)
- Wszystkie 18 keypoints przeszły RANSAC (100% inliers)

**OBSERWACJA**: Motion model dał Z=648m, ale PnP znalazł Z=556m (powrót do poprzedniej wartości).

**Pytanie**: Dlaczego motion model dał 648m?
- Motion model extrapolates z poprzednich frames
- Frame 12260 miał dużą zmianę (556 → 582m)
- Frame 12261 extrapolates dalej (582 → 648m)

### Krok 4: EXPLOZJA (Frame 12262)

```
[POSE_PRED] frame=12262 Z=509.979 nb_3d=46
[PNP_INIT] Z_init=-5433.64 nb_pts=31
[PNP_INLIERS] total=31 inliers=0 outliers=31 Z_final=-6030.94
[PNP_RESULT] frame=12262 Z=-5433.64 inliers=0 outliers=31 success=0
[KF_DEC] frame=12262 is_kf=1
```

**Charakterystyka**:
- **Motion model prediction**: Z=509.979m (world frame)
- **PnP initialization**: Z_init=-5433.64m (camera frame)
- **PnP result**: Z_final=-6030.94m
- **inliers=0, outliers=31** (0% inlier ratio!)
- **PnP FAILED**: success=0
- **Ale frame został stworzony jako keyframe!**

**UWAGA: Układy współrzędnych**:
- `Twc.translation().z()` = pozycja kamery w świecie (Z world)
- `Tcw.translation().z()` = pozycja świata w kamerze (Z camera)
- Relacja: Twc = Tcw.inverse()
- Z_world = 509.979m → Z_camera = -5433.64m

**Pytanie**: Dlaczego motion model dał Z=509.979m (world) = -5433.64m (camera)?

**Odpowiedź**: Motion model extrapolates z frame 12261:
- Frame 12261: Z_world = 556.293m
- Frame 12260: Z_world = 582.335m
- Frame 12259: Z_world = 556.699m

Motion model widzi trend:
- 12259 → 12260: +25.6m (556 → 582)
- 12260 → 12261: -26.0m (582 → 556)
- 12261 → 12262: Motion model extrapolates -46.3m (556 → 509)

Ale 509m w world frame = -5433m w camera frame!

**Dlaczego taki duży skok w camera frame?**
- To zależy od orientacji kwaternionu
- Mała zmiana w orientacji → duża zmiana w Z coordinate
- Jeśli camera jest zrotowana o ~90° wokół osi X/Y, to Z_world mapuje się na Z_camera z dużym scale

### Krok 5: Kompletna utrata trackingu (Frames 12263-12264)

```
[POSE_PRED] frame=12263 Z=-11414.5 nb_3d=9
[POSE_PNP] frame=12263 Z=-11414.5 nb_3d=0

[POSE_PRED] frame=12264 Z=-17400.5 nb_3d=0
[POSE_PNP] frame=12264 Z=-17400.5 nb_3d=0
```

**Charakterystyka**:
- nb_3d spadło z 31 → 9 → 0
- Motion model kontynuuje ekstrapolację: -5433 → -11414 → -17400m
- Brak 3D keypoints do PnP
- System tworzy keyframes mimo braku trackingu!

### Krok 6: Stabilizacja @ błędnej pozycji (Frame 12265+)

```
[POSE_PRED] frame=12265 Z=-23387.4 nb_3d=30
[PNP_INIT] Z_init=-17400.6 nb_pts=23
[PNP_INLIERS] total=23 inliers=22 outliers=1 Z_final=-17400.5
[PNP_RESULT] frame=12265 Z=-17400.5 inliers=22 outliers=1 success=1
```

**Charakterystyka**:
- PnP zbiega do Z=-17400m
- 22 inliers z 23 keypoints (96% inlier ratio!)
- **System myśli że pozycja jest poprawna!**
- Kontynuuje tracking @ tej błędnej pozycji

---

## Pytanie Naukowe: Dlaczego explozja @ frame 12262?

### Hipoteza 1: Wybuchowa ekstrapolacja motion model

**Timeline**:
```
Frame  Z (motion model)  Delta
12259       556.699        -
12260       556.621       -0.078
12261       648.347       +91.726  ← DUŻY SKOK
12262       509.979      -138.368  ← DUŻY SKOK (w przeciwnym kierunku!)
```

**Co się stało**:
1. Frame 12260: PnP dał Z=582.335m (różnica +26m od motion model)
2. Frame 12261: Motion model extrapolates = 648.347m (ale PnP koryguje do 556m)
3. Frame 12262: Motion model widzi oscylację i extrapolates = 509.979m
4. 509m world frame = -5433m camera frame (dzięki rotacji kamery)

**Dlaczego motion model dał 648m @ frame 12261?**
- Motion model używa velocity z poprzednich frames
- Velocity = (Z_current - Z_previous) / dt
- Frame 12259 → 12260: v = (582 - 556) / dt = +26m/dt
- Frame 12260 → 12261: prediction = 582 + 26/dt * dt = 608m (ale logi pokazują 648m)

**Różnica**: 648 vs 608
- Motion model może używać acceleration (druga pochodna)
- Albo były outliers w velocity estimation

### Hipoteza 2: Zła orientacja kamery

**Kluczowe pytanie**: Dlaczego 509m (world) = -5433m (camera)?

**Matematyka**:
```
Twc = [R | t]
Tcw = Twc.inverse() = [R^T | -R^T * t]

Z_camera = (-R^T * t).z()
```

Jeśli R jest zrotowana, to mały Z_world może dać duży Z_camera.

**Przykład**:
- Jeśli camera jest zrotowana o ~80° wokół osi Y:
- Z_world = 509m
- Z_camera = -509 / cos(80°) ≈ -2930m (nie -5433m)

Aby dostać -5433m z 509m:
- scaleFactor = 5433 / 509 = 10.67
- cos(θ) = 1/10.67 = 0.0937
- θ = 84.6°

**WNIOSEK**: Camera musi być zrotowana o ~84.6° względem świata!

**Pytanie**: Dlaczego taka rotacja?
- **Odpowiedź**: To może być accumulation error w orientacji przez 12k frames
- Albo szybki ruch robota (np. skręt o 85°)

### Hipoteza 3: Słaby tracking inicjuje błędne koło

**Timeline degradacji**:
```
Frame  nb_3d  Inliers  Inlier Ratio
12259   44      44        100%
12260   36       1         2.8%   ← FIRST DEGRADATION
12261   18      18        100%   ← Recovery (ale słaby)
12262   31       0         0%     ← COMPLETE FAILURE
```

**Frame 12260**: Tylko 1 inlier z 36 keypoints
- Motion model extrapolates z tego słabego wyniku
- PnP daje Z=582m (vs prediction 556m)
- Ta różnica (26m) jest wprowadzona do velocity estimation

**Frame 12261**: nb_3d spada do 18
- Motion model używa velocity z frame 12260
- Prediction = 648m (bardzo daleko od truth 556m)
- PnP koryguje do 556m, ale velocity jest już zepsute

**Frame 12262**: nb_3d wrasta do 31 (ale to false positives!)
- Motion model extrapolates = 509m
- W camera frame: 509m * 10.67 = -5433m
- PnP ma 0 inliers → FAIL
- System zachowuje motion model prediction (błędne!)

---

## Wnioski Naukowe

### 1. Explozja jest inicjowana przez słaby tracking

**Frame 12260** jest关键时刻:
- 1 inlier / 36 keypoints = 2.8% ratio
- Walidacja powinna odrzucić, ale nie odrzuca
- Ten frame "zatruje" motion model

### 2. Motion model extrapolates błędy

Motion model nie jest odporny na outliers:
- Frame 12260: błąd 26m
- Frame 12261: błąd 92m
- Frame 12262: błąd 6000m!

### 3. Pętla FEEDBACK staje się negatywna

```
Słaby tracking → Zła pozycja → Motion model extrapolates błąd
     ↑                                      ↓
     └──────── PnP nie może skorygować ───────┘
```

### 4. Walidacja nie odrzuca garbage

```cpp
if( !success
    || nbinliers < 5  ← Threshold jest za niski
    || voutliersidx.size() > 0.5 * vwpts.size()
    || Twc.translation().array().isInf().any()
    || Twc.translation().array().isNaN().any() )
{
    resetFrame();
    return;
}
```

**Frame 12260**:
- nb_3d = 36
- nbinliers = 1 (2.8%)
- Walidacja: 1 < 5 → powinno odrzucić!
- Ale nie odrzuciło (success=1)

**Frame 12262**:
- PnP failure (success=0)
- Ale frame jest stworzony jako keyframe!
- System kontynuuje z motion model prediction

### 5. System nie wykrywa degeneratnej geometrii

PnP wymaga:
- Minimum 4 points w general position (nie coplanar)
- Points rozłożone w 3D (nie wszystkie na tej samej głębokości)

**Frame 12262**:
- 31 keypoints, 0 inliers
- Sugeruje że wszystkie keypoints są outliers
- Prawdopodobnie points są coplanar albo clustered

---

## Rekomendowane Dalsze Badania

### Pytanie 1: Dlaczego tylko 1 inlier @ frame 12260?

**Potrzebne dane**:
- Reprojection error distribution
- Depth distribution 3D points
- Spatial distribution keypoints (czy clustered?)
- Cost function value z Ceres

**Hipoteza**: Motion model initialization jest daleko od truth → RANSAC znajduje tylko 1 consensus set.

### Pytanie 2: Dlaczego motion model dał 648m @ frame 12261?

**Potrzebne dane**:
- Velocity vector (vx, vy, vz)
- Acceleration vector
- Quaternion orientation (qx, qy, qz, qw)

**Hipoteza**: Velocity estimation używa outliers z frame 12260.

### Pytanie 3: Dlaczego 509m (world) = -5433m (camera)?

**Potrzebne dane**:
- Quaternion orientation @ frames 12260-12262
- Rotation matrix R
- Obliczenie: Z_camera = (-R^T * t).z()

**Hipoteza**: Camera rotation ~84.6° względem world frame.

### Pytanie 4: Jaki jest cost function value @ explozja?

**Potrzebne dane**:
- `summary.final_cost()` z Ceres
- Initial cost (przed optimization)
- Reprojection error distribution

**Hipoteza**: PnP zbiega do local minimum z wysokim cost.

---

## Next Steps (dla użytkownika)

1. **Zdecydować** czy dodawać Iterację 2 logging:
   - Final cost z Ceres
   - Velocity z motion model
   - Quaternion orientation
   - Reprojection error distribution

2. **Uruchomić** test od frame 12250 do 12270 (20 frames around explosion)

3. **Analizować**:
   - Dlaczego 1 inlier @ frame 12260?
   - Jaka jest orientacja kamery @ explozja?
   - Jaki jest cost value?

4. **Zrozumieć**:
   - Motion model implementation
   - Pętla FEEDBACK (pozytywna vs negatywna)
   - Degeneratna geometria detection

---

## Status

✅ **Iteracja 1 zakończona**
- Logging działa
- Znaleziono explozję @ frame 12262
- Zrozumiano timeline: normal → degradacja → explozja → utrata → stabilizacja

**Odkrycia**:
1. Explozja wystąpiła @ frame 12262, NIE @ frame 14657
2. Pierwsza degradacja @ frame 12260 (1 inlier / 36 keypoints)
3. Motion model extrapolates błędy
4. Walidacja nie odrzuca garbage (threshold too low)
5. System stabilizuje się @ błędnej pozycji (-17400m)

**Pytania otwarte**:
1. Dlaczego 1 inlier @ frame 12260?
2. Jaka jest orientacja kamery @ explozja?
3. Jaki jest cost value?
4. Czy jest to degeneratna geometria?

---

**Dokument stworzony na podstawie**: full_test_v2.log
**Frames przeanalizowane**: 12220-12300
**Metodologia**: Scientific logging (Iteracja 1)
