# ⚠️ ARCHIVED DOCUMENT - SUPERSEDED

**Status**: This analysis has been superseded by more comprehensive testing.
**Date Archived**: 2026-01-03
**Reason**: Further testing showed the root cause analysis was incomplete. See METHODOLOGY.md for updated methodology.

---

# Root Cause Report: Z-Axis Explosion Bug

**Data**: 2025-12-29
**Metodologia**: METHODOLOGY.md - Iteracyjne debugowanie
**Commit**: f727da0
**Status**: ROOT CAUSE ZNALEZIONY

---

## Executive Summary

**Z-axis explosion bug ISTNIEJE w obecnym kodzie.**

- **Test 0-2000**: No explosion
- **Test 0-5000**: No explosion
- **Test 0-22183**: **EXPLOSION FOUND!**

Max Z jump: **346,424 m** (!) przy frame 14657

---

## Dokładne Miejsce Explozji

### Frame 14657 - Largest Explosion

**Sequence of events:**

```
Frame 14656: Z = -995m, keyframe created, nb_3d=26
Frame 14657: POSE_PRED Z = -995m → POSE_PNP Z = -32,550 m  ← PnP FAILED
Frame 14658: POSE_PRED Z = -64,329 m → POSE_PNP Z = -993 m   ← Corrected
Frame 14659: POSE_PRED Z = +30,270 m → POSE_PNP Z = -993 m   ← Jump back
```

**Timeline:**
- 20 frames przed: nb_3d oscyluje 12-42 (słaby tracking)
- Z płynnie spada: -963m → -1012m
- @ frame 14657: **PnP converges to Z = -32,550 m**
- Motion model używa garbage do predykcji następnej ramki

### Inne Explozje

- Frame 13114-13115: Jump -3,797 m (84m → -3,713m)
- Frame 13115-13116: Jump +3,815 m (-3,713m → 102m)
- Frame 14657-14659: Jumps -63,333 m, +94,599 m, -31,264 m

---

## Root Cause Analysis

### Plik: `src/visual_front_end.cpp`
### Linie: 844-872

**Problem Code:**

```cpp
// Ceres-based PnP (motion-only BA)
success = MultiViewGeometry::ceresPnP(..., Twc, ...);

// Check that pose estim. was good enough
size_t nbinliers = vwpts.size() - voutliersidx.size();

if( !success
    || nbinliers < 5
    || voutliersidx.size() > 0.5 * vwpts.size()
    || Twc.translation().array().isInf().any()    // ← Sprawdza tylko Inf
    || Twc.translation().array().isNaN().any() )  // ← Sprawdza tylko NaN
{
    // Reject pose
    resetFrame();
    return;
}

// Pose seems to be OK!
pcurframe_->setTwc(Twc);  // ← Z = -32550m zaakceptowane!
```

### Dlaczego to jest problem:

1. **Brak walidacji zakresu**: Kod sprawdza tylko czy wartość jest Inf/NaN
2. **PnP może zbiec do dowolnej wartości**: Nawet 32km pod ziemię
3. **Garbage pose propaguje**: Motion model używa złej pozycji do następnej ramki
4. **Wyzwalacza cascade**: Zła pozycja → zła predykcja → zły tracking → kolejna explozja

### Dlaczego explozja występuje @ frame 14657:

**Warunki:**
- nb_3d = 32 (mało 3D keypoints)
- Słaby tracking przez ostatnie 20 frames
- Z dryftuje: -963m → -1012m
- PnP initialization z motion model jest garbage
- PnP solver zbiega do ekstremalnej pozycji (Z = -32,550 m)
- **Walidacja nie odrzuca** bo nie jest Inf/NaN
- Pozycja zostaje zaakceptowana

---

## Reprodukcja

### Minimalny Test Case:

```bash
./build/ov2slam parameters_files/pohang00.yaml /datasets/pohang00 0 15000
```

**Oczekiwane wyniki:**
- Frames 0-13000: Normal tracking (Z płynnie rośnie)
- Frame 13114-13116: Pierwsza explozja (~3.8km jumps)
- Frames 13000-14600: Normal tracking
- Frame 14657: Największa explozja (Z = -32,550 m)
- Frames 14658-22183: Recovery ale niestabilny

### Wyzwalacze:

1. **Słaby tracking** (nb_3d < 50)
2. **Duży dryft Z** (> 50m w ciągu 30 frames)
3. **Mało 3D keypoints** (< 40)
4. **Keyframe decision** może wywołać PnP w złym momencie

---

## Otwarte Pytania

### 1. Dlaczego explozja nie występuje w frames 0-5000?

**Hipoteza**: Przez pierwsze 5000 frames, tracking jest dobry i Z jest blisko 0. Nie ma wystarczająco dużego dryftu żeby PnP zbiegł do ekstremalnej wartości.

**Dane z logs:**
- Frame 0-5000: Z rośnie płynnie od -12m do ~400m
- Max jump: <1m między ramkami
- nb_3d zazwyczaj > 50

### 2. Dlaczego explozja występuje @ frame 13114 i 14657?

**Wspólna cecha**: Obie explozje występują gdy:
- Z jest duży ujemny (~-1000m)
- nb_3d jest niski (< 40)
- Tracking degraduje się przez 10-20 frames

**Możliwa przyczyna**: Dataset ma długą sekwencję gdzie kamera jedzie w dół (Z spada). Przy słabym trackingu, PnP nie może utrzymać stabilnej pozycji.

---

## Rekomendowane Naprawy

### Option 1: Sanity Check dla Z (RECOMMENDED)

```cpp
// W visual_front_end.cpp linia 848, dodać:
const double MAX_REASONABLE_Z = 1000.0;  // 1km
double zval = Twc.translation().z();

if( !success
    || nbinliers < 5
    || voutliersidx.size() > 0.5 * vwpts.size()
    || Twc.translation().array().isInf().any()
    || Twc.translation().array().isNaN().any()
    || std::abs(zval) > MAX_REASONABLE_Z )  // ← DODAJ TO
{
    if( !bdop3p ) {
        bp3preq_ = true;
    }
    else if( pslamstate_->mono_ ) {
        resetFrame();
    }
    return;
}
```

### Option 2: Sanity Check dla Jump

```cpp
// Sprawdź czy Z zmieniła się o więcej niż 100m od poprzedniej ramki
Sophus::SE3d prevTwc = pcurframe_->getTwc();
double dz = Twc.translation().z() - prevTwc.translation().z();

if( std::abs(dz) > 100.0 ) {
    // Reject pose
    resetFrame();
    return;
}
```

### Option 3: Większe nb_3d Threshold

```cpp
// Zwiększ minimalną liczbę inliers
if( nbinliers < 10 )  // Zamiast < 5
{
    resetFrame();
    return;
}
```

---

## Status

✅ **ROOT CAUSE ZNALEZIONY**
- Lokalizacja: `src/visual_front_end.cpp:844-872`
- Problem: Brak walidacji zakresu pozycji po PnP
- Impact: Z może przyjąć ekstremalne wartości (nawet 32km)
- Reprodukcja: Test 0-15000 frames

---

**Następne kroki** (dla użytkownika):
1. Zdecydować czy implementować fix
2. Wybrać typ walidacji (Option 1, 2, lub 3)
3. Przetestować fix na tym samym dataset
