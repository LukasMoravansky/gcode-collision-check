"""G-code modal state: values that persist across lines until overridden."""

from __future__ import annotations

from dataclasses import dataclass, field

MM_PER_INCH = 25.4

_MOTION_CODES = {0, 1, 2, 3, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89}
_COORD_SYSTEM_CODES = {54, 55, 56, 57, 58, 59}
_PLANE_CODES = {17, 18, 19}
_AXIS_LETTERS = ("x", "y", "z")


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class GCodeWord:
    """A single letter+number word parsed from a line, e.g. G90, X10.5, H1."""

    letter: str
    value: float


@dataclass
class ModalState:
    """Modal state of a G-code interpreter.

    Every field here is "sticky": it keeps its value from line to line until
    a word in the current line's ``update()`` call overrides it.
    """

    motion_mode: str | None = None  # G0/G1/G2/G3/G80-G89
    coord_system: str = "G54"  # G54-G59
    distance_mode: str = "G90"  # G90 (absolute) / G91 (incremental)
    plane: str = "G17"  # G17/G18/G19
    units: str = "G21"  # G20 (inch) / G21 (mm)
    tlc_offset: float = 0.0
    position: Vector3 = field(default_factory=Vector3)

    def update(self, words: list[GCodeWord], tool_table: dict[int, float] | None = None) -> None:
        """Apply one line's words to the state.

        ``tool_table`` maps H-number -> tool length offset (mm), used to
        resolve ``G43 Hn``. Axis words (X/Y/Z) are interpreted in the units
        and distance mode active *after* all G-words on this line have been
        processed, then converted to and stored internally as mm.
        """
        axis_values = {}

        for word in words:
            letter = word.letter.upper()
            if letter == "G":
                self._apply_g_code(int(word.value), words, tool_table)
            elif letter.lower() in _AXIS_LETTERS:
                axis_values[letter.lower()] = word.value

        if axis_values:
            self._apply_axis_values(axis_values)

    def _apply_g_code(
        self, code: int, words: list[GCodeWord], tool_table: dict[int, float] | None
    ) -> None:
        if code in _MOTION_CODES:
            self.motion_mode = f"G{code}"
        elif code in _COORD_SYSTEM_CODES:
            self.coord_system = f"G{code}"
        elif code == 90:
            self.distance_mode = "G90"
        elif code == 91:
            self.distance_mode = "G91"
        elif code in _PLANE_CODES:
            self.plane = f"G{code}"
        elif code == 20:
            self.units = "G20"
        elif code == 21:
            self.units = "G21"
        elif code == 43:
            self._apply_tlc(words, tool_table)
        elif code == 49:
            self.tlc_offset = 0.0

    def _apply_tlc(self, words: list[GCodeWord], tool_table: dict[int, float] | None) -> None:
        h_number = next((int(w.value) for w in words if w.letter.upper() == "H"), None)
        if h_number is not None and tool_table is not None and h_number in tool_table:
            self.tlc_offset = tool_table[h_number]

    def _apply_axis_values(self, axis_values: dict[str, float]) -> None:
        scale = MM_PER_INCH if self.units == "G20" else 1.0
        incremental = self.distance_mode == "G91"
        for axis, value in axis_values.items():
            mm_value = value * scale
            if incremental:
                setattr(self.position, axis, getattr(self.position, axis) + mm_value)
            else:
                setattr(self.position, axis, mm_value)
