# Z-Axis Explosions: Complete Timeline

**Data**: 2025-12-29
**Dataset**: pohang00 (frames 0-22183)
**Metodologia**: Scientific logging z OV2SLAM

---

## Summary

**W całym datasetcie wystąpiły 3 explozje Z-axis**:

1. **Frame 12262**: Z = -5433m (first explosion)
2. **Frame 14565**: Z = -139438m (second explosion)
3. **Frame 14590+**: Z = 3.4 million meters (third explosion)

---

## Explosion 1: Frame 12262

### Timeline:

```
Frame  Z (World)    nb_3d  Inliers  Outliers  Status
----- ------------  ------  -------  --------  ------
12258    556.78        27       27         0   ✅ Normal
12259    556.70        44       44         0   ✅ Normal
12260    582.34        36        1        35   ⚠️  First degradation (2.8% inliers!)
12261    556.29        18       18         0   ⚠️  Recovery (weak)
12262  -5433.64        31        0        31   💥  EXPLOSION (0% inliers)
12263 -11414.5         0        -         -   ❌  Lost
12264 -17400.5         0        -         -   ❌  Lost
12265 -17400.x        22       22         0   ⚠️  Stabilized @ wrong pose
```

### Krytyczne momenty:

**Frame 12260**: Pierwsza degradacja
- Tylko 1 inlier z 36 keypoints (2.8% ratio)
- Walidacja powinna odrzucić, ale nie odrzuca
- Motion model otrzymuje zanieczyszczony velocity

**Frame 12262**: Explozja
- Motion model prediction: Z=509.979m (world)
- PnP initialization: Z=-5433.64m (camera)
- 0 inliers / 31 keypoints
- PnP FAILED (success=0)
- Ale frame stworzony jako keyframe!

### Root cause:
- Słaby tracking @ frame 12260 (1 inlier)
- Motion model extrapolates błędy
- Walidacja nie odrzuca garbage

---

## Explosion 2: Frames 12265-14565 (Drift)

### Timeline:

```
Frame  Z (World)     nb_3d  Status
----- --------------  ------  ------
12265   -17400         22    ⚠️  Stabilized @ -17km
...      ...          ...    Tracking kontynuuje
14000   -17400         30    ⚠️  Still @ -17km
14500  -139356         41    ⚠️  Drifting to -139km
14565  -139438         38    ⚠️  Accelerating drift
```

### Charakterystyka:

**System kontynuuje tracking** @ błędnej pozycji:
- Z spokojnie dryftuje: -17400m → -139438m
- Tracking "działa": 30-40 keypoints, 100% inliers
- System myśli że pozycja jest poprawna!
- Motion model extrapolates trend: -17km → -139km

### Pytanie:
- Dlaczego Z dryftuje z -17400m do -139438m?
- Czy jest to continuous trend czy krokowa zmiana?
- Dlaczego PnP nie wykrywa błędu (100% inliers)?

---

## Explosion 3: Frames 14577-14590 (Second catastrophe)

### Timeline:

```
Frame  Z (World)      nb_3d  Inliers  Outliers  Status
----- ---------------  ------  -------  --------  ------
14577  -173018           36       0        36   💥  PnP FAIL (0% inliers)
14578  -172999           44       0        44   💥  PnP FAIL
14579  -172956           45       0        45   💥  PnP FAIL
14580  -172907           42       0        42   💥  PnP FAIL
14581  -172851           46       0        46   💥  PnP FAIL
14582  -172788           41       0        41   💥  PnP FAIL
14583  -172718           40       0        40   💥  PnP FAIL
14584  -172640           42       0        42   💥  PnP FAIL
14585  -172554           45       0        45   💥  PnP FAIL
14586  -172458           46       0        46   💥  PnP FAIL
14587  -172354           47       0        47   💥  PnP FAIL
14588  -172240           46       0        46   💥  PnP FAIL
14589  -172117           44       0        44   💥  PnP FAIL
14590 3.42933e+06         0       -         -   💥  TOTAL EXPLOSION (3.4 million m!)
```

### Charakterystyka:

**Frames 14577-14589**: Continuous PnP failures
- 13 consecutive frames z 0 inliers
- Z linearnie dryftuje: -173018 → -172117m
- System tworzy keyframes mimo PnP failures!

**Frame 14590**: Total explosion
- Z skok z -172117m → 3,429,330m (20x increase!)
- nb_3d = 0 (brak 3D keypoints)
- System kontynuuje tracking

---

## Wnioski Naukowe

### 1. Pierwsza explozja (frame 12262) jest inicjowana przez słaby tracking

**Frame 12260** to关键时刻:
- 1 inlier / 36 keypoints = 2.8% ratio
- Ten frame "zatruwa" motion model
- Następne frame ekstrapolują błąd

### 2. System nie odrzuca garbage

Walidacja jest za słaba:
- Frame 12260: 2.8% inliers → zaakceptowane
- Frame 12262: 0% inliers → PnP FAIL, ale keyframe stworzony!
- Frames 14577-14589: 13 consecutive PnP FAILURES → keyframes tworzone

### 3. Pętla FEEDBACK staje się negatywna

```
Słaby tracking → Zła pozycja → Motion model extrapolates błąd
     ↑                                      ↓
     └──── PnP nie może skorygować ──────────┘
```

### 4. Drift akumuluje się

- Frame 12262: -5433m
- Frames 12265-14500: Dryft do -139438m (8x increase)
- Frames 14577-14589: PnP连续失败
- Frame 14590+: Skok do 3.4 million m (24x increase)

### 5. System "myśli" że tracking działa

Dla Z = -139438m:
- nb_3d = 38
- inliers = 38
- outliers = 0
- System: "Everything is fine!"

Ale w rzeczywistości:
- Z powinno być ~500-600m
- Error = 140,000m (280x错误!)

---

## Pytania Otwarte

### Pytanie 1: Dlaczego PnP ma 100% inliers @ Z=-139438m?

**Hipoteza 1**: Map points są również błędne
- Jeśli wszystkie map points mają Z ≈ -139000m
- To PnP znajdzie "spójną" pozycję @ Z=-139438m
- Ale to jest spójne z błędną mapą!

**Hipoteza 2**: Degeneratna geometria
- Wszystkie keypoints są na tej samej głębokości
- PnP ma infinite solutions
- Solver wybiera jedną (błędną)

**Potrzebne dane**:
- Depth distribution map points
- Cost function value
- Reprojection error distribution

### Pytanie 2: Dlaczego 13 consecutive PnP failures @ frames 14577-14589?

**Hipoteza**: Map jest całkowicie błędna
- Map points mają Z ≈ -170000m
- Rzeczywistość: Z ≈ 500m
- Różnica: 340x error!
- PnP nie może zbiec do poprawnej pozycji

**Potrzebne dane**:
- Kiedy zostały stworzone te map points?
- Czy były triangulacje przed frame 12262?
- Czy BA (bundle adjustment) był aktywny?

### Pytanie 3: Dlaczego skok z -172117m → 3.4 million m @ frame 14590?

**Hipoteza**: Motion model extrapolation
- Trend @ frames 14577-14589: ~90m/frame dryft
- Motion model extrapolates: -172117m + 90m = -172027m
- Ale somehow Z = 3.4 million m

**Możliwe wyjaśnienie**:
- Quaternion rotation change
- Mała zmiana w orientacji → duża zmiana w Z coordinate

### Pytanie 4: Czy można odzyskać poprawną pozycję?

**Current behavior**:
- System kontynuuje tracking @ błędnej pozycji
- Tworzy keyframes z błędnych pozycji
- Nie ma "relocalization" lub "reset"

**Poprawne zachowanie**:
- Wykryć że PnP failed (success=0, 0 inliers)
- Reset tracking
- Rozpocząć od nowa (relocalization)

---

## Rekomendowane Next Steps

### 1. Dodaj Iterację 2 logging:

**Motion model**:
- Velocity vector (vx, vy, vz)
- Quaternion orientation (qx, qy, qz, qw)
- Rotation matrix R

**PnP solver**:
- Final cost z Ceres
- Initial cost (przed optimization)
- Reprojection error distribution
- Number of iterations

**Map quality**:
- Depth distribution map points
- Uncertainties map points
- Kiedy zostały stworzone (before/after frame 12262?)

### 2. Test od frame 14550 do 14600:

**Cel**: Zrozumieć drugą explozję (3.4 million m)

**Logi do zebrania**:
- Pętla PnP failures @ frames 14577-14589
- Moment skoku do 3.4 million m @ frame 14590
- Quaternion orientation change

### 3. Analiza mapy:

**Pytania**:
- Ile map points ma Z > 1000m?
- Kiedy zostały stworzone?
- Czy BA był aktywny?
- Czy można usunąć outliers?

### 4. Implementacja relocalization:

**Current problem**:
- System kontynuuje tracking @ błędnej pozycji
- Nie ma mechanizmu resetu

**Propozycja**:
- Jeśli PnP failed przez N consecutive frames → reset
- Jeśli nb_3d < 5 przez N frames → reset
- Jeśli Z diverges (zmiana > 1000m/frame) → reset

---

## Status

✅ **Analiza zakończona**
- Znaleziono 3 explozje
- Zrozumiano timeline
- Zidentyfikowano root causes

**Odkrycia**:
1. Pierwsza explozja @ frame 12262 (initiated by weak tracking)
2. Continuous drift @ frames 12265-14565
3. Druga explozja @ frame 14590 (3.4 million m)

**Pytania otwarte**:
1. Dlaczego PnP ma 100% inliers @ błędnej pozycji?
2. Dlaczego motion model nie jest resetowany?
3. Czy jest to degeneratna geometria?

---

**Dokument stworzony na podstawie**: full_test_v2.log
**Frames przeanalizowane**: 0-22183
**Metodologia**: Scientific logging (Iteracja 1)
