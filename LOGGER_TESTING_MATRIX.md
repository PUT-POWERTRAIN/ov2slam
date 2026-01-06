# Logger Testing Matrix - Quick Reference

**Phase:** 1.3 - Agent 3 (Testing/Validation)
**Status:** NEEDS ADDITIONAL TESTS
**Date:** 2026-01-06

---

## Executive Summary

| Category | Count | CRITICAL | HIGH | MEDIUM | LOW |
|----------|-------|----------|------|--------|-----|
| Thread Safety | 3 | 1 | 1 | 1 | 0 |
| Performance | 2 | 0 | 1 | 0 | 1 |
| Correctness | 3 | 1 | 2 | 0 | 0 |
| Integration | 2 | 1 | 0 | 1 | 0 |
| Stress | 3 | 0 | 1 | 2 | 0 |
| Edge Cases | 4 | 0 | 0 | 2 | 2 |
| **TOTAL** | **17** | **3** | **5** | **6** | **3** |

**Estimated Total Time:** ~2 hours
**Minimum Viable Testing:** ~1 hour (CRITICAL + HIGH priority tests)

---

## Quick Test Reference

### Must Have (Before Production)

| Test | Priority | Duration | What It Validates |
|------|----------|----------|-------------------|
| 1.1 Concurrent Write + Read | CRITICAL | 5 min | No data corruption under concurrency |
| 3.1 Format Validation | CRITICAL | 2 min | Output file format correctness |
| 4.1 Full Integration | CRITICAL | 30 min | Complete SLAM pipeline |
| 2.1 Mutex Overhead | HIGH | 10 min | Performance < 1% degradation |
| 3.3 Regression Test | HIGH | 15 min | Output matches baseline |

**Total Time:** 62 minutes (~1 hour)

### Should Have (Before Next Release)

| Test | Priority | Duration | What It Validates |
|------|----------|----------|-------------------|
| 1.2 Concurrent KF + Regular | HIGH | 3 min | Keyframe handling correctness |
| 3.2 Data Integrity | HIGH | 3 min | Round-trip data accuracy |
| 5.1 Max Frame Rate | HIGH | 5 min | System performance under load |

**Additional Time:** 11 minutes

### Nice to Have (Future Work)

All remaining MEDIUM and LOW priority tests:
- Stress tests (5.2, 5.3)
- Edge cases (6.1, 6.2, 6.3, 6.4)
- Performance tuning (2.2)
- Rerun integration (4.2)

**Additional Time:** ~48 minutes

---

## Test Scenarios At A Glance

### Thread Safety Tests

```
Test 1.1: Concurrent Write + Read
┌─────────────────┐         ┌──────────────────┐
│ Thread A        │         │ Thread B         │
│ addSE3Pose()    │         │ writeTrajectory()│
│ @ 30 Hz         │         │ @ 1 Hz           │
│ (300 poses)     │         │ (10 writes)      │
└────────┬────────┘         └────────┬─────────┘
         │                          │
         └──────────┬───────────────┘
                    │
              ┌─────▼─────┐
              │  Logger   │
              │  Mutex    │
              └───────────┘

Expected: No corruption, all 300 lines present
```

```
Test 1.2: Concurrent Pose Types
┌─────────────────┐         ┌──────────────────┐
│ Thread A        │         │ Thread B          │
│ addSE3Pose()    │         │ addKfSE3Pose()    │
│ @ 30 Hz         │         │ @ 3 Hz            │
│ (regular poses) │         │ (keyframes)       │
└────────┬────────┘         └────────┬──────────┘
         │                          │
         └──────────┬───────────────┘
                    │
              ┌─────▼─────┐
              │  Logger   │
              │  Mutex    │
              └───────────┘

Expected: Both files valid, no duplicates in KF file
```

### Performance Tests

```
Test 2.1: Mutex Overhead Measurement

Baseline (Single Thread):
addSE3Pose() x 10000  →  Time: T1
Total: T1

Contended (Two Threads):
Thread A: addSE3Pose() x 5000
Thread B: addKfSE3Pose() x 5000
Total: T2

Acceptable: T2 < 1.01 * T1  (1% overhead)

Profiler Output:
┌─────────────────────┬───────┬──────────┬──────────┬────────────┐
│ Mutex               │ Locks │ Wait(ms) │ Hold(ms) │ Contention │
├─────────────────────┼───────┼──────────┼──────────┼────────────┤
│ Logger::logger_     │ 10000 │   <50    │   <100   │    <5%     │
└─────────────────────┴───────┴──────────┴──────────┴────────────┘
```

### Correctness Tests

```
Test 3.1: Format Validation

Input: 100 known poses
Output: test_trajectory.txt

Validation Pipeline:
┌──────────────┐
│ Count Lines  │ → Should be 100
├──────────────┤
│ Count Fields │ → Should be 8 (all lines)
├──────────────┤
│ Check Types  │ → All numeric
├──────────────┤
│ Check Quat   │ → Norm ≈ 1.0 (±0.01)
├──────────────┤
│ Check Format │ → No scientific notation
└──────────────┘

Pass Condition: All checks pass
```

```
Test 3.2: Data Integrity

┌──────────┐
│ Generate │ Known poses
│  100     │ (deterministic)
└─────┬────┘
      │
      ▼
┌──────────┐
│  Logger  │ addSE3Pose() x 100
└─────┬────┘
      │
      ▼
┌──────────┐
│  Write   │ to file
└─────┬────┘
      │
      ▼
┌──────────┐
│  Read    │ back from file
└─────┬────┘
      │
      ▼
┌──────────┐
│ Compare  │ Each field:
│          │   abs(original - read) < 1e-9
└──────────┘

Pass Condition: All 100 poses match exactly
```

### Integration Tests

```
Test 4.1: Full SLAM Pipeline

┌─────────────┐
│ Build OV2SLAM│
│  + Profiling│
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Run on      │ pohang00 dataset
│ 1000 frames │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Monitor     │ 1. Exit code = 0
│ Execution   │ 2. No segfaults
│             │ 3. Clean shutdown
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Validate    │ 1. 3 output files created
│ Output      │ 2. File format correct
│             │ 3. Profiling report OK
└─────────────┘

Success Criteria:
✅ Clean exit (code 0)
✅ All files valid
✅ Contention < 10%
✅ Runtime < 2x baseline
```

### Stress Tests

```
Test 5.1: Maximum Frame Rate

┌─────────────────┐
│ Thread A        │
│ addSE3Pose()    │ 120 Hz (high-speed camera)
│ NO DELAY        │ (1200 poses in 10 sec)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Thread B        │
│ writeTrajectory()│ Every 1 second
│                 │ (10 writes total)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Verify          │ 1. No data loss
│                 │ 2. No crashes
│                 │ 3. Contention < 20%
└─────────────────┘
```

```
Test 5.3: Rapid Shutdown

Timeline:
0.0s: Start adding poses @ 30 Hz
1.0s: Trigger shutdown (call writeResults())
      └─> Some poses may be in-flight
1.1s: Shutdown complete

Verify:
✅ All poses before shutdown are written
✅ No partial file write
✅ Clean exit (no use-after-free)
```

### Edge Cases

```
Test 6.1: Empty File Writes

Scenario: No poses added, just write files

Logger::writeTrajectory("empty.txt")
Logger::writeKfsTrajectory("empty_kfs.txt")

Expected:
✅ Files created (0 bytes or header only)
✅ No crashes
✅ No error messages
```

```
Test 6.2: Rapid Successive Calls

for(int i = 0; i < 1000; i++) {
    Logger::addSE3Pose(i, pose, false);
}
// No delay, back-to-back calls

Expected:
✅ All 1000 poses added
✅ No mutex starvation
✅ No lock failures
```

```
Test 6.3: Exception During Write

Scenario: Disk full / invalid path

Logger::writeTrajectory("/dev/full/invalid.txt")

Expected:
✅ Exception propagated (not swallowed)
✅ Mutex released (RAII guarantees this)
✅ No memory leaks
✅ Consistent state

Note: May require fault injection
```

---

## Validation Commands Reference

### Quick Validation (One-Liners)

```bash
# Count lines in trajectory
wc -l ov2slam_traj.txt

# Check all lines have 8 fields
awk '{if(NF!=8) print "Bad line", NR}' ov2slam_traj.txt

# Check quaternion norms
awk '{q=$5^2+$6^2+$7^2+$8^2; if(q<0.99||q>1.01) print "Bad quat at", NR}' ov2slam_traj.txt

# Check timestamp ordering
awk 'NR>1{if($1<=prev)print="Out of order at",NR; prev=$1}' ov2slam_traj.txt

# Check for duplicates (keyframes)
sort ov2slam_kfs_traj.txt | uniq -d

# Check exit code of last command
echo $?
```

### Full Validation Script

```bash
#!/bin/bash
# validate_trajectory.sh

TRAJ_FILE=$1

if [ -z "$TRAJ_FILE" ]; then
    echo "Usage: $0 <trajectory_file>"
    exit 1
fi

echo "Validating: $TRAJ_FILE"
echo "================================"

# 1. File exists
if [ ! -f "$TRAJ_FILE" ]; then
    echo "❌ File does not exist"
    exit 1
fi
echo "✅ File exists"

# 2. Not empty
if [ ! -s "$TRAJ_FILE" ]; then
    echo "❌ File is empty"
    exit 1
fi
echo "✅ File not empty"

# 3. Correct number of fields
BAD_LINES=$(awk '{if(NF!=8) print NR}' "$TRAJ_FILE")
if [ -n "$BAD_LINES" ]; then
    echo "❌ Lines with wrong field count: $BAD_LINES"
    exit 1
fi
echo "✅ All lines have 8 fields"

# 4. Quaternion normalization
BAD_QUATS=$(awk '{q=$5^2+$6^2+$7^2+$8^2; if(q<0.99||q>1.01) print NR}' "$TRAJ_FILE")
if [ -n "$BAD_QUATS" ]; then
    echo "❌ Bad quaternions at lines: $BAD_QUATS"
    exit 1
fi
echo "✅ All quaternions normalized"

# 5. Timestamp ordering
OUT_OF_ORDER=$(awk 'NR>1{if($1<=prev)print NR; prev=$1}' "$TRAJ_FILE")
if [ -n "$OUT_OF_ORDER" ]; then
    echo "⚠️  Out of order timestamps at: $OUT_OF_ORDER"
else
    echo "✅ Timestamps are ordered"
fi

echo "================================"
echo "✅ Validation passed!"
```

---

## Profiling Report Interpretation

### Expected Output (Healthy System)

```
================================================================================
                    OV2SLAM Synchronization Profiler Report
================================================================================

--- Mutex Lock Statistics (Sorted by Contention Ratio) ---
Mutex Name          Locks    Wait (ms)    Hold (ms)    Avg Wait    Avg Hold    Contention %
--------------------------------------------------------------------------------------
Logger::logger_    10000      42.3        856.7        0.004       0.086       4.9%
Map::map_mutex      5000      15.2        423.1        0.003       0.085       3.6%
Frame::feat_mutex   3000       8.7        234.5        0.003       0.078       3.7%
--------------------------------------------------------------------------------------
```

### Key Metrics

| Metric | Good | Warning | Bad |
|--------|------|---------|-----|
| **Contention %** | < 5% | 5-10% | > 10% |
| **Avg Wait (ms)** | < 0.01 | 0.01-0.1 | > 0.1 |
| **Avg Hold (ms)** | < 0.1 | 0.1-1.0 | > 1.0 |
| **Wait/Hold Ratio** | < 0.1 | 0.1-0.5 | > 0.5 |

### What to Look For

✅ **Healthy:**
- Contention < 5%
- Avg Wait < 0.01 ms
- Wait/Hold ratio < 0.1

⚠️ **Warning Signs:**
- Contention > 10% (threads fighting over lock)
- Avg Wait > 0.1 ms (slow lock acquisition)
- Wait time >> Hold time (lock is bottleneck)

❌ **Critical Issues:**
- Contention > 50% (severe contention)
- Wait time > 1 ms (very slow)
- Lock count is 0 (lock not being used)

---

## Decision Tree: Which Tests to Run?

```
                   ┌─────────────────┐
                   │ Starting Point  │
                   └────────┬────────┘
                            │
                            ▼
                   ┌─────────────────┐
                   │ Just merged the │
                   │ mutex changes?  │
                   └────────┬────────┘
                     │ YES  │ NO
                     ▼      │
              ┌────────────┴────────────┐
              │                         │
              ▼                         ▼
     ┌────────────────┐        ┌────────────────┐
     │ Run CRITICAL   │        │ Run all tests  │
     │ tests only     │        │ (full suite)   │
     │ (1.1, 3.1, 4.1)│        └────────────────┘
     └────────┬───────┘
              │
              ▼
     ┌────────────────┐
     │ All passed?    │
     └────┬───────┬───┘
          │ YES   │ NO
          ▼       ▼
    ┌─────────┐ ┌──────────┐
    │ Ship it │ │ Debug    │
    │         │ │ (run HIGH│
    │         │ │ priority)│
    └─────────┘ └──────────┘
```

---

## Timeline: Minimal Viable Testing

**Week 1: Core Validation (CRITICAL)**
- Day 1: Implement Test 1.1 (Concurrent Write + Read)
- Day 2: Implement Test 3.1 (Format Validation)
- Day 3: Run Test 4.1 (Full Integration)

**Week 2: Performance & Regression (HIGH)**
- Day 1: Implement Test 2.1 (Mutex Overhead)
- Day 2: Implement Test 3.3 (Regression Test)

**Week 3+: Polish (MEDIUM/LOW)**
- Implement remaining tests as time permits

---

## Success Criteria Summary

**Minimum for Production:**
- ✅ Test 1.1: No data corruption
- ✅ Test 3.1: All files valid
- ✅ Test 4.1: Clean shutdown
- ✅ Test 2.1: Overhead < 1%
- ✅ Test 3.3: Matches baseline

**Stretch Goals:**
- ✅ All HIGH priority tests pass
- ✅ All MEDIUM priority tests pass
- ✅ Low priority tests documented

---

## FAQ

**Q: Can I skip thread safety tests if I'm confident the mutex is correct?**
A: No. Empirical validation is required. Theoretical correctness doesn't guarantee absence of subtle bugs.

**Q: What if I don't have time for all tests?**
A: Run CRITICAL tests first (1.1, 3.1, 4.1). That's the minimum viable testing.

**Q: Do I need to test on real hardware or can I use a VM?**
A: Real hardware is preferred for performance tests (2.1, 5.x). Correctness tests (1.x, 3.x) can run on VM.

**Q: How often should I run these tests?**
A:
- CRITICAL: Every build (CI/CD)
- HIGH: Before every commit
- MEDIUM/LOW: Weekly or before releases

**Q: Can I automate these tests?**
A: Yes! All tests should be automated. Add to CI/CD pipeline.

**Q: What's the ROI of testing?**
A:
- Prevents data corruption (critical for SLAM)
- Catches performance regressions early
- Provides confidence for production deployment
- Estimated time saved: 10-100x (debugging vs testing)

---

**End of Quick Reference**

See `LOGGER_THREAD_SAFETY_TEST_REQUIREMENTS.md` for detailed specifications.
