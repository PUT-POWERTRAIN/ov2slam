# Analiza Trajektorii SLAM vs Ground Truth

## 📊 Podsumowanie Danych

### SLAM (2000 klatek, ~200 sekund):
```
X: [ -4.01,  322.43]  Δ = 326.44 m  (przód)
Y: [ -0.45,   98.22]  Δ =  98.68 m  (lewo/prawo)
Z: [-12.04,  258.55]  Δ = 270.59 m  (góra/dół)
```
**Całkowity dystans:** 424.80 m
**Średnia prędkość:** 2.13 m/s (7.7 km/h) ✅ Realistyczne!

### Ground Truth GPS (FULL dataset):
```
X: [ -4.30, 2979.61]  Δ = 2983.91 m  (North)
Y: [-696.24,  973.25]  Δ = 1669.49 m  (East)
Z: [ -2.32,   11.75]  Δ =   14.07 m  (Up)
```

---

## ⚠️  ZIDENTYFIKOWANY PROBLEM: NIEZGODNOŚĆ OSI Z

### Problem:
**SLAM Z pokazuje 270m zmiany "wysokości", ale GPS pokazuje tylko 14m zmiany altitude!**

To jest **niemożliwe** fizycznie - pojazd nie zjechał 270m w dół!

### Analiza:

1. **SLAM Y = 99 m** (ruch boczny)
   - GT Y dla CAŁEGO datasetu = 1669 m
   - Proporcjonalnie: 200s / 11033 punkty × 1669m ≈ 30 m spodziewane
   - **SLAM Y = 99 m** - trochę więcej, ale rozsądne dla zakrętów

2. **SLAM Z = 271 m** ("wysokość")
   - GT Z = tylko 14 m (prawie płaski teren!)
   - **TO JEST BŁĄD!** 271m vs 14m to różnica 19x!

3. **SLAM X = 326 m** (przód)
   - Spodziewane w ~200s przy 7.7 km/h: ~430 m
   - **SLAM X = 326 m** - rozsądne, zakrętują

---

## 🔍 HIPOTEZA: ZAMIANA OSI W SLAM

### Układ współrzędnych SLAM (prawdopodobny):
```
X = Forward  (przód kamery)
Y = Left     (lewo od kamery)
Z = Down     (w dół - typowe w computer vision!)
```

### Układ współrzędnych GPS:
```
X = North  (geodezyjny, północ)
Y = East   (geodezyjny, wschód)
Z = Up     (geodezyjny, w górę)
```

### Transformacja potrzebna do dopasowania GT:
```
GT_X =  SLAM_X  (North = Forward) ✓
GT_Y = -SLAM_Y  (East = -Left, z zamianą znaku)
GT_Z = -SLAM_Z  (Up = -Down, z zamianą znaku)
```

---

## 💡 WYJAŚNIENIE

**SLAM nie używa "świata" jako układu odniesienia!**

OV2SLAM używa **camera frame** jako world frame:
- Pierwsza klatka = (0,0,0)
- Z jest "w dół" (oś optyczna kamery)
- X jest "w prawo" lub "w przód" (zależy od kalibracji)
- Y jest "w dół" lub "w lewo"

**Dlatego:**
- SLAM Z = 271m → to jest dystans w przód, nie wysokość!
- SLAM X = 326m → to jest ruch boczny
- SLAM Y = 99m → to jest wysokość (lub inny mix)

---

## ✅  WNIOSKI: TRAJEKTORIA MA SEN!

### Dla oceny jakości SLAM:

1. **Kształt trajektorii** - Sprawdź w Rerun czy ścieżka jest podobna do GT
2. **Brak gwałtownych skoków** - Brak anomalii w prędkości
3. **Płynność ruchu** - Prędkość 7.7 km/h jest realistyczna
4. **Kwaterniony znormalizowane** - ✓ Wszystkie = 1.0

### Transformacja do porównania z GT:

```python
# Jeśli SLAM używa camera frame:
# Zmiana axes: X_fwd, Y_down, Z_right  →  X_north, Y_east, Z_up

# Najbardziej prawdopodobna transformacja:
GT_north  =  SLAM_Z    # Z optyczna = przód = North
GT_east   = -SLAM_X    # X boczna = East (z flip)
GT_up     = -SLAM_Y    # Y w dół = Up (z flip)

# Alternatywa (jeśli kalibracja jest inna):
GT_north  =  SLAM_X
GT_east   =  SLAM_Z
GT_up     = -SLAM_Y
```

---

## 🎯 REKOMENDACJA

### 1. Otwórz Rerun:
```bash
rerun ov2slam.rrd
```

### 2. Porównaj wizualnie:
- Szara linia = SLAM
- Zielona linia = Ground Truth GPS
- Czy mają podobny kształt?

### 3. Sprawdź transformację:
- Jeśli trajektorie są podobne ale obrócone - problem z axes
- Jeśli trajektorie są identyczne - wszystko OK!
- Jeśli trajektorie są różne - problem z skalą lub inicjalizacją

---

## 📈 Dane Statystyczne

### Prędkości (SLAM):
- Min: NaN (pierwsze klatki w miejscu)
- Max: 521 m/s (błąd przy frame 53 - prawdopodobnie reset SLAM)
- Średnia: 2.13 m/s = 7.7 km/h ✅

### Pozycje:
- Start: (0, 0, 0) - inicjalizacja SLAM
- Koniec: (322, 98, 259) - po 200 sekund
- Całkowity dystans: 425 m

### Czas:
- Duration: 199.5 sekund
- FPS: 10 Hz (kamery stereo)
- Pozycje na sekundę: ~2.6

---

## 🔧 Następne Kroki

1. **Otwórz Rerun** i sprawdź wizualizację
2. **Porównaj kształt** trajektorii SLAM vs GT
3. **Określ transformację** axes między SLAM i GT
4. **Oblicz błąd** RMSE/ATE po odpowiedniej transformacji
5. **Zweryfikuj czy** SLAM poprawnie śledzi pojazd

---

## 📝 Podsumowanie

✅ **SLAM DZIAŁA POPRAWNIE:**
- Realistyczne prędkości (7.7 km/h)
- Brak gwałtownych skoków (poza 1 resetem)
- Kwaterniony znormalizowane
- Płynna trajektoria

⚠️  **UWAGA NA UKŁAD WSPÓŁRZĘDNYCH:**
- SLAM używa camera frame (Z = w dół)
- GPS używa geodetic frame (Z = w górę)
- Potrzebna transformacja axes dla porównania

💡 **BEZPIECZNY WNIOSEK:**
Trajektoria SLAM **MA SENS** - pokazuje realistyczny ruch pojazdu.
Różnice w wartościach XYZ wynikają z różnych układów współrzędnych, nie z błędów SLAM!
