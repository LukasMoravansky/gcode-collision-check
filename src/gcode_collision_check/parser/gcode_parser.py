"""Generic RS-274 (Fanuc-style) G-code parser: text -> list[GCodeSegment].

MVP limitations:
- Arc centers are resolved from I/J/K words only (no R-format arcs).
- G28 (home) is treated as a plain rapid to the given coordinates -- there is
  no modeling of a separate machine-home position.
- Canned cycles always retract to the R-plane after the cycle (G99-style),
  regardless of G98/G99.
"""

from __future__ import annotations

import re

from gcode_collision_check.parser.arc_linearizer import GCodeSegment, expand_canned_cycle
from gcode_collision_check.parser.modal_state import GCodeWord, ModalState, Vector3

_WORD_RE = re.compile(r"([A-Za-z])\s*(-?\d+\.\d+|-?\.\d+|-?\d+)")

_LINEAR_CODES = {0, 1}
_ARC_CODES = {2, 3}
_CANNED_CYCLE_CODES = {73, 81, 82, 83, 84, 85, 86, 87, 88, 89}
_MOTION_CODES = _LINEAR_CODES | _ARC_CODES | _CANNED_CYCLE_CODES | {28}

_ARC_OFFSET_LETTERS = {
    "G17": (("I", "x"), ("J", "y")),
    "G18": (("I", "x"), ("K", "z")),
    "G19": (("J", "y"), ("K", "z")),
}


def strip_comment(line: str) -> str:
    line = re.sub(r"\([^)]*\)", "", line)
    line = line.split(";", 1)[0]
    return line.strip()


def tokenize_line(line: str) -> list[GCodeWord]:
    text = strip_comment(line)
    if not text or text == "%":
        return []
    words = []
    for letter, value in _WORD_RE.findall(text):
        letter = letter.upper()
        if letter == "N":
            continue  # line numbers, not motion-relevant
        words.append(GCodeWord(letter, float(value)))
    return words


def _find(words: list[GCodeWord], letter: str) -> float | None:
    for word in words:
        if word.letter.upper() == letter:
            return word.value
    return None


def _arc_center(start: Vector3, plane: str, words: list[GCodeWord]) -> Vector3:
    (letter_a, axis_a), (letter_b, axis_b) = _ARC_OFFSET_LETTERS[plane]
    offset_a = _find(words, letter_a)
    offset_b = _find(words, letter_b)
    center = Vector3(start.x, start.y, start.z)
    setattr(center, axis_a, getattr(start, axis_a) + (offset_a if offset_a is not None else 0.0))
    setattr(center, axis_b, getattr(start, axis_b) + (offset_b if offset_b is not None else 0.0))
    return center


def parse_program(text: str, tool_table: dict[int, float] | None = None) -> list[GCodeSegment]:
    """Parse G-code text into a flat list of motion segments.

    Canned cycles are expanded into their constituent rapid/feed moves here;
    arcs are returned as single G2/G3 segments (linearize with
    ``arc_linearizer.linearize_arc`` downstream).
    """
    state = ModalState()
    segments: list[GCodeSegment] = []

    for line_no, raw_line in enumerate(text.splitlines(), start=1):
        words = tokenize_line(raw_line)
        if not words:
            continue

        motion_word = next(
            (w for w in words if w.letter == "G" and int(w.value) in _MOTION_CODES), None
        )
        r_word = _find(words, "R")
        z_word = _find(words, "Z")
        q_word = _find(words, "Q")
        f_word = _find(words, "F")

        start = Vector3(state.position.x, state.position.y, state.position.z)
        plane, wcs = state.plane, state.coord_system
        state.update(words, tool_table=tool_table)
        end = Vector3(state.position.x, state.position.y, state.position.z)
        tlc_offset = state.tlc_offset
        raw_text = raw_line.strip()

        if motion_word is None:
            continue

        code = int(motion_word.value)

        if code in _LINEAR_CODES or code == 28:
            motion_mode = "G1" if code == 1 else "G0"
            segments.append(
                GCodeSegment(
                    line_no, raw_text, motion_mode, start, end,
                    plane=plane, feed=f_word, wcs=wcs, tlc_offset=tlc_offset,
                )
            )
        elif code in _ARC_CODES:
            center = _arc_center(start, plane, words)
            segments.append(
                GCodeSegment(
                    line_no, raw_text, f"G{code}", start, end,
                    plane=plane, arc_center=center, feed=f_word, wcs=wcs, tlc_offset=tlc_offset,
                )
            )
        elif code in _CANNED_CYCLE_CODES:
            r_plane = r_word if r_word is not None else start.z
            z_depth = z_word if z_word is not None else start.z
            cycle_start = Vector3(end.x, end.y, start.z)
            cycle_end = Vector3(end.x, end.y, r_plane)
            cycle = GCodeSegment(
                line_no, raw_text, f"G{code}", cycle_start, cycle_end,
                plane=plane, r_plane=r_plane, z_depth=z_depth, q_peck=q_word,
                feed=f_word, wcs=wcs, tlc_offset=tlc_offset,
            )
            segments.extend(expand_canned_cycle(cycle))
            state.position.z = r_plane  # simplified retract (see module docstring)

    return segments
