# OV2SLAM Standalone - Master Documentation Index

**Last Updated**: 2026-01-03
**Project**: OV2SLAM Visual SLAM System (ROS-independent fork)
**Status**: Active development

---

## Quick Start

### What is OV2SLAM?

OV2SLAM is a real-time Visual SLAM system for stereo/monocular cameras with a multi-threaded architecture (Tracking, Mapping, Bundle Adjustment, Loop Closing). This version operates without ROS dependencies and has been enhanced with:

- Async parallel image loading (thread-safe, 2-6x speedup)
- GPS/AHRS initialization support
- Rerun 3D visualization integration
- Comprehensive z-axis bug fixes

**Core Documentation**: Start with `README.md` and `CLAUDE.md`

---

## 1. Core Documentation

### Essential Reading (✅ Current)

| File | Size | Summary | Priority |
|------|------|---------|----------|
| **[README.md](README.md)** | 13.9 KB | Official project documentation with build/run instructions, architecture overview, and usage examples | ⭐⭐⭐ |
| **[CLAUDE.md](CLAUDE.md)** | 8.1 KB | AI assistant instructions + project overview (build system, threading model, datasets) | ⭐⭐⭐ |

### Reference Documentation

| File | Size | Summary |
|------|------|---------|
| **[ARCHITECTURE_DIAGRAM.md](ARCHITECTURE_DIAGRAM.md)** | 22.0 KB | Visual architecture of the async image loader (thread pools, work queues, synchronization) |

---

## 2. Implementation Work

### Async Image Loader (✅ Production Ready)

| File | Size | Summary | Status |
|------|------|---------|--------|
| **[async_image_loader_README.md](async_image_loader_README.md)** | 9.5 KB | Thread-safe parallel PNG decoder implementation, performance benchmarks (2-6x speedup), sanitizer results | ✅ Current |
| **[ASYNC_LOADER_THREAD_SAFETY_ANALYSIS.md](ASYNC_LOADER_THREAD_SAFETY_ANALYSIS.md)** | 9.6 KB | Thread safety analysis of parallel decode implementation, data race fixes, atomic operations | ✅ Current |
| **[INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md)** | 8.7 KB | Step-by-step integration guide for async loader into main.cpp | ✅ Current |
| **[IMPLEMENTATION_VERIFICATION.md](IMPLEMENTATION_VERIFICATION.md)** | 9.1 KB | Verification checklist for async loader implementation (shared pointers, mutexes, cleanup) | ✅ Current |
| **[ARCHITECTURE_DIAGRAM.md](ARCHITECTURE_DIAGRAM.md)** | 22.0 KB | Visual architecture of thread pools, work queues, and synchronization | ✅ Current |

### GPS/AHRS Integration (✅ Current)

| File | Size | Summary | Status |
|------|------|---------|--------|
| **[RERUN_INTEGRATION.md](RERUN_INTEGRATION.md)** | 3.7 KB | Rerun 3D visualization integration (optional build feature) | ✅ Current |

---

## 3. Z-Axis Bug Investigation

### Definitive Reports (✅ Current - Read These First)

| File | Size | Summary | Status |
|------|------|---------|--------|
| **[FINAL_SCIENTIFIC_REPORT.md](FINAL_SCIENTIFIC_REPORT.md)** | 12.0 KB | **READ THIS FIRST** - Root cause identified: PnP validation bug for stereo mode (`resetFrame()` only called in monocular) | ✅ Definitive |
| **[EXPLOSIONS_SUMMARY.md](EXPLOSIONS_SUMMARY.md)** | 8.5 KB | Summary of all 3 z-axis explosions found in pohang00 dataset (frames 12262, 14565, 14590) | ✅ Current |
| **[BUG_ANALYSIS_Z_AXIS_EXPLOSION.md](BUG_ANALYSIS_Z_AXIS_EXPLOSION.md)** | 11.2 KB | Narrative debugging story explaining the cascade failure mechanism | ✅ Current |

### Frame-Level Analysis (✅ Current - Detailed)

| File | Size | Summary | Status |
|------|------|---------|--------|
| **[SCIENTIFIC_ANALYSIS_FRAME12262.md](SCIENTIFIC_ANALYSIS_FRAME12262.md)** | 12.1 KB | Deep dive into first explosion @ frame 12262 (PnP outlier ratio 2.8% accepted) | ✅ Current |
| **[SCIENTIFIC_ANALYSIS_FRAME14657.md](SCIENTIFIC_ANALYSIS_FRAME14657.md)** | 5.5 KB | Analysis of second explosion pattern | ✅ Current |

### Superseded Reports (⚠️ Historical Context Only)

| File | Size | Summary | Status | Why Superseded |
|------|------|---------|--------|----------------|
| **[ROOT_CAUSE_REPORT.md](ROOT_CAUSE_REPORT.md)** | 5.4 KB | Initial root cause analysis findings | ⚠️ Superseded | Replaced by FINAL_SCIENTIFIC_REPORT.md (more comprehensive) |
| **[HANDOFF.md](HANDOFF.md)** | 13.1 KB | Handoff document during investigation phase | ⚠️ Superseded | Investigation complete, see FINAL_SCIENTIFIC_REPORT.md |

### Invalidated Reports (❌ Do Not Use)

| File | Size | Summary | Status | Why Invalidated |
|------|------|---------|--------|-----------------|
| **[PERFORMANCE_COMPARISON_REPORT.md](PERFORMANCE_COMPARISON_REPORT.md)** | 8.5 KB | Performance comparison of parallel decode | ❌ Invalidated | **Flawed methodology** - compared different frame ranges, not same frames |
| **[PARALLEL_DECODE_IMPLEMENTATION.md](PARALLEL_DECODE_IMPLEMENTATION.md)** | 9.1 KB | Initial parallel decode implementation notes | ⚠️ Superseded | Replaced by async_image_loader_README.md (production version) |

---

## 4. Performance & Testing

| File | Size | Summary | Status |
|------|------|---------|--------|
| **[SYNC_IO_TEST.md](SYNC_IO_TEST.md)** | 0.9 KB | Synchronous I/O performance test results | ✅ Current |

---

## 5. Project Management

### Methodology (✅ Current - Manager Mode)

| File | Size | Summary | Status |
|------|------|---------|--------|
| **[METHODOLOGY.md](METHODOLOGY.md)** | 16.0 KB | **Manager Mode (Level 0.5 Abstraction)** - How to coordinate subagents, verification strategies, quality control | ⭐⭐⭐ Essential |
| **[CODE_REVIEW_SUMMARY.md](CODE_REVIEW_SUMMARY.md)** | 12.0 KB | Code review process and findings summary | ✅ Current |
| **[FINAL_REVIEW_SUMMARY.md](FINAL_REVIEW_SUMMARY.md)** | 10.9 KB | Final review of all implemented changes | ✅ Current |

### Handoff & Planning (⚠️ Superseded)

| File | Size | Summary | Status | Note |
|------|------|---------|--------|------|
| **[HANDOFF.md](HANDOFF.md)** | 13.1 KB | Investigation handoff document | ⚠️ Superseded | Investigation complete, see FINAL_SCIENTIFIC_REPORT.md |
| **[SUMMARY.md](SUMMARY.md)** | 9.8 KB | Project summary during investigation | ⚠️ Superseded | Historical context only |

---

## 6. Archived / Historical

| File | Size | Summary | Status |
|------|------|---------|--------|
| **[PHASE3_SUMMARY.md](PHASE3_SUMMARY.md)** | 9.1 KB | Phase 3 development summary | ⚠️ Historical |

---

## Quick Reference Guide

### For New Developers

1. **Start Here**: `README.md` - Understand the system
2. **Then Read**: `CLAUDE.md` - Build/run instructions
3. **For Changes**: `METHODOLOGY.md` - How to work with AI assistants

### For Understanding Bug Fixes

1. **Read First**: `FINAL_SCIENTIFIC_REPORT.md` - The definitive root cause analysis
2. **Then**: `EXPLOSIONS_SUMMARY.md` - Summary of all explosions
3. **Details**: `BUG_ANALYSIS_Z_AXIS_EXPLOSION.md` - The narrative story

### For Async Loader Implementation

1. **Overview**: `async_image_loader_README.md` - Performance, features, usage
2. **Architecture**: `ARCHITECTURE_DIAGRAM.md` - Visual design
3. **Integration**: `INTEGRATION_GUIDE.md` - Step-by-step integration
4. **Safety**: `ASYNC_LOADER_THREAD_SAFETY_ANALYSIS.md` - Thread safety guarantees

### For Performance Analysis

1. **Current Results**: `async_image_loader_README.md` (Section 2)
2. **Tests**: `SYNC_IO_TEST.md`

---

## Warnings & Important Notes

### ⚠️ Invalidated Documentation

**DO NOT USE** `PERFORMANCE_COMPARISON_REPORT.md` - It contains flawed methodology comparing different frame ranges rather than same frames. Use `async_image_loader_README.md` Section 2 for accurate performance data.

### ⚠️ Superseded Documentation

- `ROOT_CAUSE_REPORT.md` - Superseded by `FINAL_SCIENTIFIC_REPORT.md`
- `PARALLEL_DECODE_IMPLEMENTATION.md` - Superseded by `async_image_loader_README.md`
- `HANDOFF.md` - Investigation complete, use `FINAL_SCIENTIFIC_REPORT.md`

### ✅ Current Definitive Sources

- **Z-Axis Bug**: `FINAL_SCIENTIFIC_REPORT.md` (12.0 KB)
- **Async Loader**: `async_image_loader_README.md` (9.5 KB)
- **Architecture**: `README.md` (13.9 KB) + `ARCHITECTURE_DIAGRAM.md` (22.0 KB)
- **Methodology**: `METHODOLOGY.md` (16.0 KB)

---

## Documentation Statistics

**Total Documentation Files**: 26
- Core: 2 files (23.1 KB)
- Implementation: 6 files (62.6 KB)
- Bug Investigation: 8 files (69.4 KB)
- Performance: 1 file (0.9 KB)
- Project Management: 5 files (61.8 KB)
- Archived: 4 files (36.2 KB)

**Active (Current)**: 19 files
**Superseded**: 5 files
**Invalidated**: 2 files

---

## Change Log

**2026-01-03**: Initial documentation index created
- Categorized all 26 documentation files
- Marked superseded and invalidated reports
- Added quick reference guides
- Added warnings for flawed methodology

---

**Note**: This index should be updated when new documentation is created or existing documentation is superseded/invalidated.
