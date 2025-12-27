# SPDX-FileCopyrightText: 2025 Ayumu Saito
# SPDX-License-Identifier: BSD-3-Clause

import pytest

from twist_stdin_bridge.parse import format_vw_csv, parse_vw_line, Vw


@pytest.mark.parametrize(
    'line,exp',
    [
        ('0.1 0.2\n', Vw(0.1, 0.2)),
        ('0.1,0.2\n', Vw(0.1, 0.2)),
        (' 1.0 , -2.5 ', Vw(1.0, -2.5)),
    ],
)
def test_parse_ok(line, exp):
    assert parse_vw_line(line) == exp


@pytest.mark.parametrize('line', ['', '\n', '1.0', '1 2 3', 'a b'])
def test_parse_ng(line):
    with pytest.raises(Exception):
        parse_vw_line(line)


def test_format_csv():
    assert format_vw_csv(Vw(1.23456789, -2.0)) == '1.234568,-2.000000\n'
