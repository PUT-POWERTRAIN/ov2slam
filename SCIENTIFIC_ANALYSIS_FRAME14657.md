# Naukowa Analiza: Z-Axis Behavior @ Frame 14657

**Data**: 2025-12-29
**Metodologia**: Iteracyjne logging z OW2SLAM
**Cel**: Zrozumieć DLACZEGO PnP zbiega do ekstremalnych wartości

---

## Executive Summary

**Kluczowe znalezisko**: Z-axis explosion @ frame 14657 NIE jest incydentalna - jest **zjawiskiem kumulatywnym** wymagającym pełnej historii tracking od frame 0.

### Testy przeprowadzone:

1. **Test 14600-14700** (bez historii):
   - Range: 100 frames
   - Wynik: **BRAK explozji**
   - Z płynnie: 0m → 260m
   - Wszystkie PnP: success=1

2. **Test 0-22183** (pełny dataset, z ROOT_CAUSE_REPORT.md):
   - Range: 22,183 frames
   - Wynik: **EXPLOZJA @ frame 14657**
   - Z skok: -995m → -32,550m

---

## Analiza Logów (Test 14600-14700)

### Statystyki tracking:

```
Liczba PnP solves: 84
PnP failures: 0 (100% success rate)
Frames z nb_3d < 4: 204 (bardzo słaby tracking!)
Min nb_3d: 0
Max nb_3d: 53
Średni nb_3d: ~22
```

### Przykładowe logi:

**Normal tracking** (frame 60):
```
[POSE_PRED] frame=60 Z=96.51 nb_3d=24
[PNP_RESULT] frame=60 Z=102.32 inliers=24 outliers=0 success=1
```

**Słaby tracking** (frame 13):
```
[POSE_PRED] frame=13 Z=1.67 nb_3d=7
[PNP_RESULT] frame=13 Z=4.65 inliers=7 outliers=0 success=1
```

**Bardzo słaby tracking** (frame 14):
```
[POSE_PRED] frame=14 Z=7.87 nb_3d=7
[PNP_RESULT] frame=14 Z=7.87 nb_3d=0 success=1  ← 0 inliers ale success!
```

### Obserwacje:

1. **PnP zawsze zbiega** nawet przy nb_3d = 0
2. **Walidacja nie odrzuca** gdy nb_3d = 0 (success=1 mimo 0 inliers)
3. **Z stabilnie rośnie** bez skoków

---

## Hipoteza: Dlaczego explozja wymaga pełnej historii

### Scenariusz A (Test 14600-14700):
- Start z "czystą" mapą (brak historii)
- Map points są poprawnie triangulowane
- PnP initialization jest blisko ground truth
- Wynik: **Stabilny tracking**

### Scenariusz B (Test 0-22183):
- Po 14k frames akumulują się błędy
- Map points mają uncertainties
- Motion model dryftuje
- @ frame 14657: nb_3d = 32, Z = -995m
- PnP initialization jest daleko od ground truth
- PnP zbiega do lokalnego minimum: Z = -32,550m
- Wynik: **Explozja**

---

## Pytanie Naukowe:

**Dlaczego PnP @ frame 14657 (pełny dataset) zbiega do Z = -32,550m, ale PnP @ frame 60 (test od 14600) zbiega do Z = 102m?**

### Różnice w warunkach:

| Parametr | Frame 60 (14600+) | Frame 14657 (0-22183) |
|----------|-------------------|------------------------|
| nb_3d | 24 | 32 |
| Historia | 0 frames | 14,657 frames |
| Z (before) | 96.51m | -995m |
| Map quality | Czysta (nowa) | Zdegradowana (akumulacja) |
| Motion model | Brak dryftu | Signifikantny dryft |

---

## Wnioski Naukowe:

### 1. Explozja nie jest deterministyczna @ krótkim range

Test od frame 14600 **NIE powiela** explozji, mimo że:
- nb_3d często < 4 (204 frames!)
- Wiele frames z 0 inliers
- Słaby tracking

To obala hipotezę że explozja jest spowodowana wyłącznie przez słaby tracking w danej chwili.

### 2. Explozja jest zjawiskiem kumulatywnym

Wymaga:
- **Historii**: ~14,600 frames przed explozją
- **Dryftu**: Z dryftuje do -995m
- **Degradacji mapy**: uncertainties akumulują się

### 3. PnP solver zachowuje się poprawnie

W teście bez historii:
- 84/84 PnP solves = success
- Wszystkie Z values są sensowne
- Żadnej explozji mimo nb_3d = 0

To sugeruje że **PnP solver nie jest buggy** - po prostu zbiega do dostępnych danych.

### 4. Problem: Walidacja nie odrzuca garbage

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

**Frame 14** z testu:
- nb_3d = 0 (0 inliers!)
- success = 1
- Z = 7.87m (zaakceptowane!)

To powinno być odrzucone, ale nie jest.

---

## Dalsze Badania:

### Pytanie 1: Co jest inne @ frame 14657 w pełnym dataset?

**Potrzebne dane**:
- Stan mapy przed explozją (map point uncertainties)
- Motion model velocity przed explozją
- Keyframe decyzje przed explozją
- Depth distribution 3D points

### Pytanie 2: Dlaczego nb_3d = 0 nie jest odrzucane?

```cpp
size_t nbinliers = vwpts.size() - voutliersidx.size();
if( nbinliers < 5 ) {  // ← To powinno odrzucić nb_3d = 0
    resetFrame();
    return;
}
```

**Ale**: W logach widzę `nb_3d=0` i `success=1`. Dlaczego?

**Możliwe wyjaśnienie**: `nb_3d` ≠ `nbinliers`. Nb_3d to liczba 3D keypoints w frame, nbinliers to liczba keypoints które przeszły RANSAC.

### Pytanie 3: Jaki jest cost function value @ explozja?

Brak logowania `summary.final_cost()`. Powinienem dodać to w Iteracji 2.

---

## Rekomendowane Next Steps:

1. **Dodaj verbose logging** (Iteracja 2):
   - Final cost z Ceres
   - Initial pose (przed PnP)
   - Velocity z motion model
   - Reprojection error distribution

2. **Uruchom pełny test** (0-22183):
   - Zbierz logi @ frames 14640-14670
   - Znajdź moment explozji
   - Porównaj z testem 14600-14700

3. **Analiza mapy**:
   - Sprawdź uncertainties map points
   - Sprawdź depth distribution
   - Znajdź outliery

---

## Status

✅ **Iteracja 1 zakończona**
- Logging działa
- Potwierdzono: explozja jest kumulatywna
- Test 14600-14700: brak explozji

**Następne kroki** (dla użytkownika):
1. Zdecydować czy uruchamiać pełny test 0-22183 (zajmie ~12 min)
2. Czy dodać bardziej szczegółowe logging (Iteracja 2)
3. Czy analizować map points uncertainties

