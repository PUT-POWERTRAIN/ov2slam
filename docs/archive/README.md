# Archive Directory

## ⚠️ WARNING: Archived Content

This directory contains archived files and documents that are **NO LONGER CURRENT** or have been **INVALIDATED**. Most of this content represents work in progress, debugging efforts, or experimental approaches that have been superseded by the final implementation.

---

## 📁 Archive Structure

### `/docs/archive/` - Documentation Archive

Contains historical investigation documents and reports:

- **`HANDOFF.md`** - ⚠️ **ARCHIVED**
  - Status: Investigation completed
  - Content: Z-axis bug investigation that could not reproduce the original issue
  - Warning: Bug could not be reproduced in current codebase

- **`INVALIDATED_flawed_methodology.md`** - ⚠️ **INVALIDATED**
  - Status: Contains FLAWED METHODOLOGY
  - Content: Performance comparison with fundamentally flawed methodology
  - Warning: Results cannot be trusted

- **`PARALLEL_DECODE_IMPLEMENTATION.md`** - ⚠️ **SUPERSEDED**
  - Status: Implementation approaches have been superseded
  - Content: Parallel PNG decode implementation details
  - Warning: Implementation complete and documented in main codebase

- **`ROOT_CAUSE_REPORT.md`** - ⚠️ **SUPERSEeded**
  - Status: Analysis has been superseded
  - Content: Incomplete root cause analysis of Z-axis explosion
  - Warning: Further testing showed the analysis was incomplete

### `/archive/` - Code Archive

Contains historical implementation files:

- **`async_image_loader.hpp`** - Baseline async image loader
- **`async_image_loader_4thread.hpp`** - 4-thread parallel decode variant
- **`async_image_loader_6thread.hpp`** - 6-thread parallel decode variant
- **`async_image_loader_fixed.hpp`** - Fixed version with load balancing corrections

---

## 🚨 Important Warnings

### **DO NOT USE** these files for:
- Current development or production code
- Performance benchmarks or comparisons
- Bug reference or reproduction
- Understanding the current system architecture

### **VALIDATION STATUS**: ❌ INVALIDATED
- Most documents represent intermediate debugging steps
- Methodologies have been proven flawed or incomplete
- Root cause analyses were incorrect or superseded
- Implementation variants have been integrated into main codebase

### **Current Reference**
For accurate information, refer to:
- `METHODOLOGY.md` - Current development methodology and conclusions
- Main source code in `src/` directory
- Current documentation in `docs/` directory

---

## 📋 Archive Rationale

These files were archived to:
1. **Preserve historical context** of the debugging and development process
2. **Document failed approaches** to prevent repeating the same mistakes
3. **Maintain investigation trail** for future reference
4. **Separate current from obsolete** implementations

---

## 🔍 For Researchers

If you're studying this codebase for academic or research purposes:
- These archived files show the **evolution of the debugging process**
- They demonstrate **common pitfalls** in SLAM system debugging
- They illustrate **how thorough methodology** is essential for reliable results
- Use them as examples of **what not to do** in performance testing and bug analysis

---

**Last Updated**: 2026-01-03
**Archive Status**: All content invalidated or superseded