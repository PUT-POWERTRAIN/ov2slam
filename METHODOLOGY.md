# Sposób Pracy - Metodologia Debugowania OV2SLAM

**Data**: 2025-12-29
**Zadanie**: Znalezienie i naprawa błędu eksplozji osi Z w systemie SLAM
**Narzędzie**: Claude Code (Sonnet 4.5) z agentami

---

## Filozofia: Manager Mode (Level 0.5 Abstraction)

### Moja Rola

Nie byłem pasywnym wykonawcą ("zrób X, zrób Y"), ale **koordynatorem systemu**:

1. **Zarządzanie wiedzą**: Musiałem wiedzieć DLACZEGO każda linijka została zmieniona
2. **Kontrola jakości**: Nie ufałem subagentom w 100% - weryfikowałem ich raporty
3. **Zarządzanie błędami**: Kiedy coś się nie udało, diagnozowałem i korygowałem
4. **Delegacja**: Zadania które mogły być zrobione równolegle -> subagenci

**Kluczowa zasada**: Diabeł tkwi w szczegółach implementacji. Nie pozwalałem subagentom na przemycanie ogólników.

---

## Use Case: Kiedy Używać Subagentów

### ✅ UŻWAJ (Wymagane lub bardzo pomocne):

1. **Research tasks** (eksploracja codebase)
   - Znajdowanie plików po patternach (np. "znajdź gdzie jest epipolar filtering")
   - Analiza dużych fragmentów kodu
   - Zrozumienie architektury systemu

2. **Implementation > 3 plików**
   - Zmiany które dotyczą wielu plików
   - Subagent może czytać i edytować równolegle

3. **Code review** (zawsze inny agent niż implementor)
   - Weryfikacja zmian
   - Znalezienie błędów

4. **Poprawa błędów po review**
   - Zapytaj użytkownika o kierunek
   - Użyj subagenta do naprawy

5. **Long-running tasks** (budowanie, testowanie)
   - `./build.sh` - subagent obsłuży błędy kompilacji
   - Testy na dużych datasetach

6. **Complex multi-step tasks**
   - Zadania wymagające >10 kroków
   - Zadania które wymagają iteracji

7. **Analiza logów i danych**
   - Przeszukiwanie dużych plików
   - Eksportowanie metryk

### ❌ NIE UŻYWAJ (Proste zadania):

1. **Proste edycje (1-2 pliki)**
   - Dodanie jednego loga
   - Zmiana jednej linii

2. **Pojedyncze komendy bash**
   - `ls -la`
   - `ps aux | grep ov2slam`

3. **Pytania o konkretne linie kodu**
   - "Co robi ta funkcja?"
   - Read tool wystarczy

---

## Model Choice Strategy

### Sonnet (Dokładność)
- **Zadania wymagające precyzji**: Implementation, code review, research
- **Kiedy błędy są kosztowne**
- **Kiedy potrzebujesz głębokiej analizy**

### Haiku (Szybkość)
- **Proste taski**: Lista plików, sprawdzenie statusu
- **Quick checks**: "Czy plik istnieje?"
- **Niski koszt**: Mało ważne zadania

### Opus (Maksymalna jakość)
- **Krytyczne zadania**: Bardzo skomplikowane problemy
- **Rzadko używany**: Tylko gdy Sonnet nie radzi sobie

---

## Iteracyjne Testowanie: Case Study

### Problem: Z-eksplozja @ frame 253

**Iteracja 0: Co wiemy?**
- User mówi: "Jest bug przy frame 253, Z eksploduje"
- Nothing more - nie wiemy DLACZEGO, GDZIE, JAK

**Iteracja 1: Coarse Logging (szukanie needle w haystack)**
```
Cel: Znajdź GDZIE Z zmienia się radykalnie
Metoda: Dodaj 4 log points w pipeline
  - POSE_PRED (przed PnP)
  - POSE_PNP (po PnP) - KLUCZOWE
  - KF_DEC (keyframe decision)
  - BA_UPDATE (po bundle adjustment)
```

**Wynik**: Znaleźliśmy że przy frame 253 Z = +1328m (eksplozja)

**Iteracja 2: Trace Backward (co prowadzi do eksplozji?)**
```
Cel: Znajdź trigger eksplozji
Metoda: Przeanalizuj 10 frames przed eksplozją
Discovery:
  - Frame 249: nb_3d drops 65→45 (pierwszy sygnał!)
  - Frame 251: Brak POSE_PNP log = PnP failed
  - Frame 252: resetFrame() called
  - Frame 253: Eksplozja
```

**Iteracja 3: Refined Logging (dlaczego nb_3d drop?)**
```
Cel: Zrozum dlaczego spadek 3D points
Metoda: Dodaj epipolar logging
  - EPIPOLAR_PRE: przed filtrowaniem
  - EPIPOLAR_POST1: po stage 1
  - EPIPOLAR_POST2: po stage 2
```

**Wynik**: Epipolar filtering odrzuca features, ale nie wyjaśnia to czemu

**Iteracja 4: Root Cause Analysis**
```
Pytanie: Dlaczego PnP failuje przy frame 251?
Discovery:
  - Kod ma: if( pcurframe_->id_ == 0 ) // first frame?
  - Ale frame 249 ma ID=249, nie 0!
  - Pierwszy frame nie jest keyframe!
  - Brak inicjalizacji = cascade failure
```

**Iteracja 5: Fix Implementation**
```
Solution: Change id_ == 0 → nbkfs_ == 0
Test: Run z --start-frame=240
Result: Works!
```

**Iteracja 6: Validation**
```
Test: Run OD FRAME 0 (bez --start-frame)
Result: No explosion?!?
Mystery: Fix nie jest potrzebny?
```

### Kluczowa Lekcja

**Iteracyjne podejście**:
1. Start coarse (4 log points)
2. Znajdź moment awarii
3. Doprecyzuj logging wokół awarii
4. Znajdź trigger
5. Zrozum przyczynę
6. Napraw
7. **ZWERYFIKUJ** (tu nasze zadanie padło)

---

## Parallel Processing Strategy

### Równoległe Execute Independent Tasks

**Przykład**: Chcę sprawdzić różne aspekty kodu:

```python
# Launch 3 agents in parallel (single message!)
Task(agent_id="a1", task="Znajdź gdzie jest PnP code")
Task(agent_id="a2", task="Znajdź gdzie jest keyframe creation")
Task(agent_id="a3", task="Znajdź gdzie jest epipolar filtering")

# Czekam na wszystkie - 3x faster niż sequential
```

**Ważne**: Single message z wieloma Task tool calls

### Sequential Processing

**Kiedy**: Taski zależą od siebie

```python
# Step 1: Build
result = Task(task="Build project")

# Step 2: Test (zależy od build)
if result.success:
    Task(task="Run test")
```

---

## Workflow: Complete Debugging Session

### Phase 1: Exploration (1-2h)

**Goal**: Zrozumienie systemu

**Subagent 1** (Explore agent):
- Znajdź główne klasy i relacje
- Zmapuj pipeline tracking → BA → keyframes
- Zidentyfikuj gdzie są potencjalne failure points

**Output**: Architectural overview, lista podejrzanych miejsc

### Phase 2: Instrumentation (30min)

**Goal: Dodaj logging**

**Manager decision**: "Dodamy 4 log points na podstawie exploration"

**Subagent 2** (Implementation):
- Dodaj POSE_PRED, POSE_PNP, KF_DEC, BA_UPDATE logs
- Rebuild
- Raport: gdzie dokładnie dodano logi

**Critical**: Manager REVIEWED co subagent dodał!

### Phase 3: Data Collection (30min)

**Goal: Zbierz dane

**Subagent 3** (Execution):
- Run test na frames 0-100
- Extract log data
- Znajdź gdzie Z eksploduje

**Output**: "Z explosion @ frame 253"

### Phase 4: Deep Dive (1h)

**Goal: Zrozum czemu

**Manager**: "Potrzebuję więcej danych wokół frame 253"

**Subagent 4** (Analysis):
- Przejrzyj frames 240-255 w detailed logs
- Znajdź pattern (nb_3d drop, PnP failures)
- Zidentyfikuj cascade sequence

**Output**: Timeline of failure

### Phase 5: Root Cause (1h)

**Goal: Znajdź przyczynę

**Subagent 5** (Code analysis):
- Read visual_front_end.cpp around first frame check
- Znajdź `if( id_ == 0 )`
- Wyjaśnij dlaczego to jest problem

**Output**: "Pierwszy frame nie jest keyframe przy start != 0"

### Phase 6: Fix (30min)

**Goal: Napraw

**Subagent 6** (Implementation):
- Create .fixed files
- Change id_ == 0 → nbkfs_ == 0
- Document fix

### Phase 7: Verification (2h+)

**Goal: Czy fix działa?

**Subagent 7** (Testing):
- Test z --start-frame=240: ✅ Works
- Test od frame 0: ❌ No explosion without fix?!

**Manager**: "Wait, to jest dziwane. Powtórzmy test z większym range."

**Subagent 8** (Extended testing):
- Add verbose logging (per-frame)
- Test 0-2000 frames
- Result: **NO EXPLOSION**

**Manager**: "Mamy mystery. Original bug nie występuje."

---

## Tool Usage: Best Practices

### Read Tool (Zamiast Bash cat)

**❌ WRONG**:
```python
Bash(command="cat file.txt | grep pattern")
```

**✅ CORRECT**:
```python
Read(file_path="file.txt")
```

**Why**:
- Bash wymaga user approval (blocking!)
- Read jest szybszy
- User nie musi approve każdy file read

### Edit Tool (Zamiast Bash sed)

**❌ WRONG**:
```python
Bash(command="sed -i 's/old/new/g' file.cpp")
```

**✅ CORRECT**:
```python
Edit(file_path="file.cpp", old_string="old", new_string="new")
```

**Why**:
- Edit pokazuje dokładnie co się zmienia
- Może być reviewed przed application
- Nie wymaga approval

### Write Tool (Zamiast Bash echo/heredoc)

**❌ WRONG**:
```python
Bash(command="echo 'line1\nline2' > file.txt")
```

**✅ CORRECT**:
```python
Write(file_path="file.txt", content="line1\nline2\n")
```

### Bash Tool (TYLKO dla system commands)

**✅ ALLOWED**:
```python
Bash(command="./build.sh")  # Build
Bash(command="ls -la")       # List files
Bash(command="mkdir -p dir") # Create directory
Bash(command="./ov2slam params.yaml dataset 0 100")  # Run binary
```

---

## Agent Coordination Patterns

### Pattern 1: Researcher + Implementor

**Research Agent**:
- Znajdź gdzie jest problem
- Zrozum kod
- Raport findings

**Implementor Agent**:
- Na podstawie research findings
- Implement fix
- Test

**Manager**: Review research findings, approve implementation

### Pattern 2: Implementor + Reviewer

**Implementor Agent**:
- Zrób zmiany
- Commit to branch
- Raport co zmieniono

**Reviewer Agent**:
- Review commit
- Znajdź bugs
- Suggest improvements

**Manager**: Decide whether to merge or redo

### Pattern 3: Swarm (Parallel Execution)

**Goal**: Szybko przetworzyć dużo danych

```python
# Launch 5 agents in parallel
Task(agent_id="a1", task="Test frame range 0-1000")
Task(agent_id="a2", task="Test frame range 1000-2000")
Task(agent_id="a3", task="Test frame range 2000-3000")
Task(agent_id="a4", task="Test frame range 3000-4000")
Task(agent_id="a5", task="Test frame range 4000-5000")

# All run in parallel - 5x speedup
```

**Manager**: Collect results, combine into report

---

## Error Handling Strategy

### When Subagent Fails

**Scenario**: Subagent reports build error

**Manager Options**:

1. **Fix subagent's work**:
   ```
   "Subagent made mistake in line 42.
    I'll Edit it myself."
   ```

2. **Ask subagent to fix**:
   ```
   Task(task="Fix the build error in line 42")
   ```

3. **Start over with different approach**:
   ```
   "Subagent's approach was wrong.
    Try different method."
   ```

### When Task Gets Stuck

**Symptoms**:
- Subagent running for >10 minutes
- No progress updates
- Context window filling up

**Manager Actions**:

1. **Check status**: Use TaskOutput with `block=false`
2. **Kill if needed**: Task is hung, terminate it
3. **Simplify**: Break into smaller subtasks
4. **Try different model**: Haiku instead of Sonnet

---

## Context Window Management

### Problem: Subagents fill context

**Solution**: Break tasks into chunks

**Przykład**: Zamiast
```
Task(task="Analyze entire codebase")  # Too broad
```

Użyj:
```
Task(task="Find all classes in src/")
Task(task="Find all classes in include/")
Task(task="Map dependencies between classes")
```

### Problem: Large files

**Solution**: Use offset/limit with Read

```
Read(file_path="large.log", offset=1000, limit=50)
```

---

## Time Management

### Typical Task Durations

**Quick tasks** (Haiku):
- List files: 10-30s
- Check status: 10-30s
- Simple edits: 30-60s

**Medium tasks** (Sonnet):
- Research (1 file): 1-2 min
- Simple implementation: 2-5 min
- Build: 2-3 min
- Test (small dataset): 1-2 min

**Long tasks** (Sonnet background):
- Complex implementation: 5-10 min
- Test (large dataset): 5-15 min
- Code review: 3-5 min

**Very long tasks** (Split up):
- Full analysis: 30+ min (break into pieces!)

### Optimization Strategies

1. **Parallel execution**: Launch multiple agents at once
2. **Background mode**: Long-running tasks don't block manager
3. **Smaller models**: Haiku for simple checks
4. **Caching**: Don't re-read files unnecessarily

---

## Communication Patterns

### Manager → Subagent

**Good prompts**:
```
"Read file X, find pattern Y, report A, B, C"
"Implement feature X with constraints Y, Z"
"Test X and report metrics A, B"
```

**Specificity matters**:
- What to read
- What to look for
- What to return
- Format of output

### Subagent → Manager

**Good reports**:
```
"Found X at line Y in file Z"
"Changed lines A-B in file C"
"Test completed: X frames processed, Y errors"
```

**Manager needs**:
- Exact locations (file:line)
- What was done
- Why (rationale)
- Results (data)

---

## Quality Assurance

### Never Trust Subagent 100%

**Verification checkpoints**:

1. **After research**:
   ```
   "Subagent says PnP is in visual_front_end.cpp.
    Verify with grep."
   ```

2. **After implementation**:
   ```
   "Subagent added logging.
    Review the code to ensure correctness."
   ```

3. **After testing**:
   ```
   "Subagent reports success.
    Check the actual log files."
   ```

### Code Review Process

**Subagent 1** (Implementor):
- Makes changes
- Creates pull request

**Subagent 2** (Reviewer):
- Reviews changes
- Finds bugs
- Suggests improvements

**Manager**:
- Reviews review
- Makes final decision
- Merges or requests changes

---

## Lessons Learned

### What Worked Well

1. **Iterative logging**: Coarse → refined → specific
2. **Parallel research**: Multiple agents exploring different aspects
3. **Verification**: Never trusted single source
4. **Documentation**: Wrote blog post to clarify thinking

### What Didn't Work

1. **Assuming bug exists**: Based on old log file that couldn't be reproduced
2. **Over-engineering fix**: May not be necessary
3. **Sequential testing**: Should have tested multiple scenarios earlier

### Improvements for Next Time

1. **Verify bug first**: Before planning fix, confirm bug is reproducible
2. **Baseline tests**: Establish what "normal" looks like
3. **A/B testing**: Test multiple configurations in parallel
4. **Simpler logging**: Don't add too much at once

---

## Summary: Sztuka Zarządzania Agentami

### Kluczowe Princypy

1. **Bądź Manager, nie Robot**
   - Myśl, nie wykonuj
   - Koordynuj, nie rob wszystko sam

2. **Deleguj mądrze**
   - Small tasks → Haiku (fast)
   - Complex tasks → Sonnet (accurate)
   - Independent tasks → Parallel

3. **Weryfikuj wszystko**
   - Nie ufaj bez sprawdzenia
   - Use tools, nie assumptions

4. **Komunikuj jasno**
   - Specific prompts
   - Clear expectations
   - Defined outputs

5. **Iteruj szybko**
   - Start coarse
   - Refine based on data
   - Don't plan everything upfront

### Manager's Mantra

> "Diabeł tkwi w szczegółach. Muszę wiedzieć DLACZEGO każda linijka została zmieniona. Nie pozwolę subagentom na ogólniki. Weryfikuję, koryguję, poprawiam."

To jest klucz do sukcesu w complex debugging tasks.

---

## Appendix: Quick Reference

### Task Templates

**Research Task**:
```
"Find X in codebase. Report: file locations, line numbers,
 brief description of what each does."
```

**Implementation Task**:
```
"Implement feature X in file Y. Constraints: A, B, C.
 Report: exact lines changed, why, any issues."
```

**Test Task**:
```
"Run test X with parameters Y. Report: frames processed,
 errors found, metrics A, B, C."
```

**Review Task**:
```
"Review changes in files X, Y, Z. Look for: bugs, style issues,
 logic errors. Report: specific problems with line numbers."
```

### Model Selection Guide

| Task Complexity | Model | Time | Quality |
|----------------|-------|------|---------|
| List files | Haiku | 10s | Good |
| Simple grep | Haiku | 10s | Good |
| Read + analyze 1 file | Sonnet | 1-2m | Excellent |
| Multi-file changes | Sonnet | 5-10m | Excellent |
| Complex research | Sonnet | 5-15m | Excellent |
| Critical tasks | Opus | 10-30m | Best |

### Tool Selection Guide

| Need | Tool | Why |
|------|------|-----|
| Read file | Read | No approval needed |
| Edit file | Edit | Shows diff, reviewable |
| Create file | Write | Clear content |
| List files | Bash ls | System command |
| Build | Bash ./build.sh | System command |
| Run binary | Bash ./prog | System command |
| Search files | Grep (tool) | No approval needed |
| Find files | Glob (tool) | No approval needed |

---

**End of Methodology Document**

Ta metodologia pozwoliła na:
- 4-godzinne badanie complex systemu
- Znalezienie potencjalnego root cause
- Stworzenie fixu
- Odkrycie że fix może nie być potrzebny

Klucz do sukcesu: Manager mode + iteracyjne testowanie + weryfikacja wszystkiego.
