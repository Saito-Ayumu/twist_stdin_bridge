# SPDX-FileCopyrightText: 2025 Ayumu Saito
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import dataclass


@dataclass(frozen=True)
class Vw:
    vx: float
    wz: float


def parse_vw_line(line: str) -> Vw:
    """Parse one line 'vx wz' or 'vx,wz' into a Vw."""
    s = line.strip()
    if not s:
        raise ValueError('empty')

    parts = s.replace(',', ' ').split()
    if len(parts) != 2:
        raise ValueError('need two values: vx wz')

    return Vw(float(parts[0]), float(parts[1]))


def format_vw_csv(vw: Vw) -> str:
    """Format Vw as CSV 'vx,wz\\n' for STDOUT."""
    return f'{vw.vx:.6f},{vw.wz:.6f}\n'
