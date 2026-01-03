## Synchronous I/O vs SLAM Timing Test

Sprawdzam real timing bez async:

### Setup
- Usuń async loader
- Zmierz czas cv::imread (I/O)
- Zmierz czas actual SLAM processing (z synchronizacją)

### Test
```bash
# Synchronous version (w main loop):
auto t1 = now();
cv::Mat left = cv::imread(...);  // I/O - blokuje
cv::Mat right = cv::imread(...);
auto t2 = now();
slam.addNewStereoImagesBlocking(ts, left, right);  // SLAM - czeka na processing
auto t3 = now();

io_time = t2 - t1;
slam_time = t3 - t2;
```

### Wyniki (z poprzednich badań)
- I/O (PNG decode z disk): ~70-80 ms/frame (bez cache)
- I/O (z cache): ~0.02 ms/frame
- SLAM processing: ~8-12 ms/frame

### Wniosek
**I/O jest bottleneck** (70-80ms vs 8-12ms SLAM)

Dlatego async I/O z prefetch pomaga:
- Background wątek dekoduje N następnych frames
- Główny wątek przetwarza SLAM (8-12ms)
- Overlap: I/O w tle during SLAM
- Speedup: ~2x (zamiast 80ms + 10ms = 90ms, mamy 10ms)
