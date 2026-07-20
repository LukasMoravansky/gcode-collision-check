# CLAUDE.md — gcode-collision-check

## Co to je

CLI nástroj pro offline detekci kolizí CNC nástrojové sestavy (fréza + stopka + držák +
upínač) se statickou scénou (svěrák, polotovar, stůl) na základě G-kódu.

Vstup: G-kód + STL scény + definice nástroje.
Výstup: safe / collision s řádkem G-kódu, pozicí XYZ, kolizním párem, hloubkou průniku.

Žádný existující open-source nástroj tohle nedělá. CAMotics a FreeCAD Path dělají jen
material removal — nekontrolují kolize držáku, upínače ani svěráku.

## Stack

- **Python 3.10+**
- **trimesh + python-fcl** — kolizní engine (dva CollisionManagery: statická scéna + pohyblivý nástroj)
- **numpy** — transformace, interpolace
- **click** — CLI framework
- Žádná GPL závislost. Celý stack je MIT/BSD.

## Architektura

gcode-collision-check/
├── src/gcode_collision_check/
│   ├── init.py
│   ├── cli.py                 # click CLI: gcode-collision-check verify ...
│   ├── parser/
│   │   ├── init.py
│   │   ├── gcode_parser.py    # generic RS-274 parser (G0/G1/G2/G3, modální stav)
│   │   ├── modal_state.py     # G54-G59, G43/G49, G90/G91, G17/G18/G19, G20/G21
│   │   └── arc_linearizer.py  # G2/G3 → lineární segmenty s chord tolerance
│   ├── tool/
│   │   ├── init.py
│   │   ├── assembly.py        # tool assembly builder (fréza + stopka + držák jako trimesh)
│   │   └── profiles.py        # built-in profily (flat, ball, bull endmill + ER holder)
│   ├── collision/
│   │   ├── init.py
│   │   ├── scene.py           # dva CollisionManagery, add_object jednou, set_transform per sample
│   │   ├── sampler.py         # interpolace segmentů, step ≤ 0.4 × r_tool, Z-prefilter
│   │   └── checker.py         # hlavní loop: parser → sampler → collision query → events
│   ├── types.py               # CollisionEvent, ToolConfig, SceneConfig — dataclasses
│   └── report.py              # JSON report, stdout summary, optional GLB vizualizace
├── tests/
│   ├── test_parser.py         # modální stav, arc linearizace, WCS, TLC
│   ├── test_tool.py           # tool assembly geometrie
│   ├── test_collision.py      # known-collision a known-safe scénáře
│   └── fixtures/              # testovací G-kódy a STL
├── examples/
│   ├── vise.stl               # generic svěrák (parametrický, ne SCHUNK)
│   ├── crash.nc               # G-kód který narazí do svěráku
│   ├── safe.nc                # G-kód který projde čistě
│   └── README.md              # "try it now" návod
├── pyproject.toml
├── README.md
├── LICENSE                    # MIT
└── .github/workflows/ci.yml


## Klíčová pravidla

### Dva CollisionManagery
Statická scéna (svěrák, stock, stůl) = `obstacles`. Pohyblivý nástroj (fréza, stopka, držák) =
`tool_group`. Objekty přidat jednou při inicializaci. Pak jen `set_transform()` per sample —
nikdy re-add (boří BVH broadphase, řádově pomalejší).

### Celá nástrojová sestava
Většina reálných kolizí = držák nebo upínač vs. svěrák, ne břit. Každá část registrovaná
pod vlastním jménem → report atribuje pár ("holder", "vise_jaw_left").

### Modální stav
Parser musí trackovat: G54–G59 (WCS offsets), G43 Hn / G49 (tool-length compensation),
G90/G91 (abs/inc), G17/G18/G19 (arc plane), G20/G21 (inch/mm). Bez správných offsetů
je kolizní check bezcenný — nástroj je na špatném místě.

### Sampling
- Lineární segmenty (G0/G1): step ≤ 0.4 × tool_radius
- Oblouky (G2/G3): chord tolerance ε → seg_angle = 2·acos(1 − ε/R)
- Canned cycles (G81–G89, G73): expandovat na G0/G1 sekvence

### Z-prefilter
Pokud Z_tip > Z_max(obstacles) + margin pro oba konce segmentu → přeskočit.
Eliminuje 60–90 % dotazů na reálných programech.

## G-code parser scope (MVP)

**Podporovat:**
- G0, G1 (lineární pohyb)
- G2, G3 (kruhová interpolace, IJK i R formát)
- G17/G18/G19 (rovina oblouku)
- G20/G21 (palce/mm)
- G28 (home)
- G40/G41/G42 (cutter comp — parsovat a ignorovat, neaplikovat)
- G43 Hn / G49 (tool-length compensation — aplikovat na Z)
- G54–G59 (work coordinate systems)
- G80–G89 + G73 (canned cycles — expandovat)
- G90/G91 (absolute/incremental)
- M3/M4/M5 (spindle — parsovat, ignorovat)
- M6 Tn (tool change — trigger re-build tool assembly)
- M30/M2 (program end)
- Komentáře: (...) a ;...
- N-čísla řádků (ignorovat)
- % (program delimiters)

**Nepodporovat v MVP:**
- Makro proměnné (#100...)
- Subprogramy (M98/M99, O-call)
- Siemens-specifické: CYCLE, CR=, WORKPIECE, R-parametry
- Parametrické programování
- 4./5. osa (A, B, C)

**Dialekt:** Fanuc-style (de facto standard). Pokrývá Fanuc, Haas, Mazak, většinu
generic postprocesorů. Siemens a Heidenhain mají vlastní syntax — mimo MVP scope.

## Jak pracovat

1. Inkrementálně. Nejdřív parser + typy + testy. Pak tool assembly. Pak collision.
   Pak CLI. Pak README. Neskládat vše najednou.
2. Testy first pro parser — modální stav je nejkritičtější a nejsnáze testovatelný
   komponent. Každý G-kód feature = test.
3. Nepsat kód spekulativně. Ověřit chování trimesh/fcl API minimálním experimentem
   pokud není jasné (transform konvence, collision query API).
4. Všechny interní jednotky v mm. Pokud vstup je G20 (palce), konvertovat při parsování.