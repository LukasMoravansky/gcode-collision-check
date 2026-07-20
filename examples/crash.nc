G21 G90 G54
G0 X0 Y0 Z50    (safe height)
G0 Z5            (approach)
G1 Z-10 F200     (cut -- under the jaw top, but inside the opening = safe)
G0 Z5
G0 X-50          (rapid into the vise jaw = CRASH)
G1 Z-10 F200
G0 Z50
M30
