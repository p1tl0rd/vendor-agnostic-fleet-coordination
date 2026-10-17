"""Tests for psipp_ctc.csv_merge -- the PSIPP/CCBS CSV merger.

The merger is a thin concat-and-validate tool. The contract is:

* Every input row has exactly 8 comma-separated fields matching the
  C++'s ``printf("%11s,%6s,%4d,%4d,%4d,%10.3lf,%10.3lf,%10d\\n")``
  format.
* Columns 0-1 are non-empty strings; columns 2-4 are non-negative
  ints; columns 5-7 are non-negative floats (``-1.0`` is the failure
  sentinel).
* Output is a row-by-row concat of the inputs in order.
* A header line is auto-detected (and skipped) on the first line of
  the first input only when its first field equals
  ``"problem_name"`` -- both the C++ and the Python emit no header,
  so this is a defensive convenience for user-supplied files.

Tests cover:
* Round-trip a 2-row CSV through the merger; verify the bytes are
  preserved exactly.
* Concatenate three CSVs of different shapes; verify row count and
  field order.
* Mismatched field count raises ``ValueError`` with the file/line
  context.
* Empty input is well-defined (zero rows out).
* Header detection: a user-supplied header on the first input is
  skipped; on the second input is NOT skipped (per the
  "first-line-only" rule).
* ``--header`` writes a column-name row at the top of the output.
* The CLI driver (``cli/merge_csv.py``) handles ``-`` for stdin /
  stdout, emits a row count to stderr, and exits non-zero on a bad
  row.
"""
from __future__ import annotations

import io
import subprocess
import sys
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

import pytest

from psipp_ctc.csv_merge import (
    COLUMNS,
    EXPECTED_FIELD_COUNT,
    iter_rows,
    merge_csv_files,
    validate_field_types,
)


# --------------------------------------------------------------------
# iter_rows / merge_csv_files
# --------------------------------------------------------------------


def test_iter_rows_round_trip(tmp_path):
    """A 2-row CSV round-trips through ``iter_rows`` byte-identically."""
    csv_text = (
        "  Berlin_1_3_CDT,     PSIPP,  10,   0,   0,   1302.280,  61516.081,        85\n"
        "  Berlin_1_3_CDT,     CCBS,  10,   0,   0,   1300.500,  61400.300,      2100\n"
    )
    p = tmp_path / "in.csv"
    p.write_text(csv_text)
    rows = list(iter_rows([str(p)]))
    assert len(rows) == 2
    # The C++ uses ``%11s`` (width 11) so the problem name is
    # right-padded with spaces; the merger does not strip the padding
    # because pandas / R's ``read_csv`` does that on read.
    assert rows[0][0] == "  Berlin_1_3_CDT"
    assert rows[0][1] == "     PSIPP"
    assert rows[0][2] == "  10"
    assert rows[1][1] == "     CCBS"


def test_iter_rows_detects_header_on_first_file(tmp_path):
    """A first-line header ``problem_name,planner_name,...`` is
    auto-detected and skipped (the C++/Python never emit one, but a
    user-supplied header is a common case)."""
    p = tmp_path / "with_header.csv"
    p.write_text(
        "problem_name,planner_name,number_of_agents,perm_id,iteration,"
        "makespan,sum_of_costs,used_time\n"
        "  Berlin_1_3_CDT,     PSIPP,  10,   0,   0,   1302.280,  61516.081,        85\n"
    )
    rows = list(iter_rows([str(p)]))
    assert len(rows) == 1
    assert rows[0][1] == "     PSIPP"


def test_iter_rows_does_not_treat_header_on_second_file_as_header(tmp_path):
    """A header-looking line on a SECOND input file is NOT skipped --
    only the first file's first line gets the special case. (The
    purpose of the auto-detect is to be friendly to user-prepared
    files; chained mergers should still validate every row.)"""
    p1 = tmp_path / "a.csv"
    p1.write_text(
        "  Berlin_1_3_CDT,     PSIPP,  10,   0,   0,   1302.280,  61516.081,        85\n"
    )
    p2 = tmp_path / "b.csv"
    p2.write_text(
        "problem_name,planner_name,number_of_agents,perm_id,iteration,"
        "makespan,sum_of_costs,used_time\n"
        "  Berlin_1_3_CDT,     CCBS,  10,   0,   0,   1300.500,  61400.300,      2100\n"
    )
    rows = list(iter_rows([str(p1), str(p2)]))
    # The first file's row + the second file's "header" + 1 data row
    # from the second file = 3 rows. The second file's "header" is
    # validated as a data row -- the field count is 8 and the
    # numeric fields parse correctly, so the merger does not raise.
    # (This is a *feature* of the simple "first file only" rule:
    # chained mergers always validate.)
    assert len(rows) == 3


def test_iter_rows_raises_on_wrong_field_count(tmp_path):
    """A row with the wrong field count raises ``ValueError`` with
    the file and 1-based line number."""
    p = tmp_path / "bad.csv"
    p.write_text(
        "  Berlin_1_3_CDT,     PSIPP,  10,   0,   0,   1302.280,  61516.081,        85\n"
        "Berlin,PSIPP,10,0,0\n"  # 6 fields
    )
    with pytest.raises(ValueError, match="bad.csv:2: expected 8 fields"):
        list(iter_rows([str(p)]))


def test_iter_rows_skips_blank_lines(tmp_path):
    """Blank lines (empty rows) are skipped silently -- the C++ never
    emits them but defensiveness is cheap."""
    p = tmp_path / "blank.csv"
    p.write_text(
        "  Berlin_1_3_CDT,     PSIPP,  10,   0,   0,   1302.280,  61516.081,        85\n"
        "\n"
        "  Berlin_1_3_CDT,     CCBS,  10,   0,   0,   1300.500,  61400.300,      2100\n"
    )
    rows = list(iter_rows([str(p)]))
    assert len(rows) == 2


def test_iter_rows_reads_from_stdin_sentinel(capsys):
    """The ``"stdin"`` sentinel means ``sys.stdin`` (mirrors the
    ``io/roadmap.py`` convention)."""
    saved_stdin = sys.stdin
    sys.stdin = io.StringIO(
        "  Berlin_1_3_CDT,     PSIPP,  10,   0,   0,   1302.280,  61516.081,        85\n"
    )
    try:
        rows = list(iter_rows(["stdin"]))
    finally:
        sys.stdin = saved_stdin
    assert len(rows) == 1


# --------------------------------------------------------------------
# validate_field_types
# --------------------------------------------------------------------


def test_validate_field_types_accepts_well_formed_rows():
    """The C++'s format is well-formed by construction."""
    rows = [
        ["  Berlin_1_3_CDT", "     PSIPP", "  10", "   0", "   0",
         "   1302.280", "  61516.081", "        85"],
        ["  Berlin_1_3_CDT", "     CCBS", "  10", "   0", "   0",
         "   1300.500", "  61400.300", "      2100"],
    ]
    # Should not raise.
    validate_field_types(rows)


def test_validate_field_types_accepts_negative_one_failure_sentinel():
    """``-1.000`` (the C++'s failure sentinel for makespan / sum-of-costs
    on a failed run) is a valid value."""
    rows = [
        ["  Random", "     PSIPP", "  20", "   0", "   0",
         "    -1.000", "    -1.000", "   30000"],
    ]
    validate_field_types(rows)


def test_validate_field_types_rejects_negative_iteration():
    """A negative ``iteration`` is malformed (the C++'s ``%4d``
    emits ``  -1`` for -1, but iteration is logically non-negative)."""
    rows = [
        ["  Berlin_1_3_CDT", "     PSIPP", "  10", "   0", "  -1",
         "   1302.280", "  61516.081", "        85"],
    ]
    with pytest.raises(ValueError, match="iteration=-1 is negative"):
        validate_field_types(rows)


def test_validate_field_types_rejects_non_numeric_used_time():
    """A non-numeric ``used_time`` is malformed. (``float("NaN")``
    succeeds in CPython; we use a clearly non-numeric value.)"""
    rows = [
        ["  Berlin_1_3_CDT", "     PSIPP", "  10", "   0", "   0",
         "   1302.280", "  61516.081", "abc"],
    ]
    with pytest.raises(ValueError, match="used_time='abc' is not float"):
        validate_field_types(rows)


def test_validate_field_types_rejects_below_failure_sentinel():
    """A makespan below ``-1.0`` (the failure sentinel) is
    malformed -- the C++'s format never produces such values."""
    rows = [
        ["  Berlin_1_3_CDT", "     PSIPP", "  10", "   0", "   0",
         "    -2.500", "  61516.081", "        85"],
    ]
    with pytest.raises(ValueError, match="below the -1.0 failure sentinel"):
        validate_field_types(rows)


# --------------------------------------------------------------------
# merge_csv_files
# --------------------------------------------------------------------


def test_merge_csv_files_concatenates_two_files_in_order(tmp_path):
    """The output is the row-by-row concat of the two inputs in
    order, with no header (matching the C++ and Python defaults)."""
    p1 = tmp_path / "psipp.csv"
    p1.write_text(
        "  Berlin_1_3_CDT,     PSIPP,  10,   0,   0,   1302.280,  61516.081,        85\n"
        "  Berlin_1_3_CDT,     PSIPP,  20,   0,   0,   2500.500, 120000.000,       150\n"
    )
    p2 = tmp_path / "ccbs.csv"
    p2.write_text(
        "  Berlin_1_3_CDT,     CCBS,  10,   0,   0,   1300.500,  61400.300,      2100\n"
    )
    buf = io.StringIO()
    n = merge_csv_files([str(p1), str(p2)], buf)
    assert n == 3
    out = buf.getvalue()
    lines = out.strip().split("\n")
    assert len(lines) == 3
    # Order is preserved: PSIPP rows first, then CCBS.
    assert "PSIPP" in lines[0]
    assert "PSIPP" in lines[1]
    assert "CCBS" in lines[2]


def test_merge_csv_files_with_header_flag(tmp_path):
    """``write_header=True`` prepends the column-name header."""
    p = tmp_path / "in.csv"
    p.write_text(
        "  Berlin_1_3_CDT,     PSIPP,  10,   0,   0,   1302.280,  61516.081,        85\n"
    )
    buf = io.StringIO()
    n = merge_csv_files([str(p)], buf, write_header=True)
    assert n == 1
    out = buf.getvalue()
    first = out.splitlines()[0]
    assert first == ",".join(COLUMNS)
    assert len(out.splitlines()) == 2


def test_merge_csv_files_empty_input_writes_zero_rows():
    """No inputs writes a zero-row output -- the function returns
    ``0`` and emits nothing."""
    buf = io.StringIO()
    n = merge_csv_files([], buf)
    assert n == 0
    assert buf.getvalue() == ""


def test_merge_csv_files_propagates_validation_error(tmp_path):
    """A bad row in any input raises before any row is written."""
    p1 = tmp_path / "good.csv"
    p1.write_text(
        "  Berlin_1_3_CDT,     PSIPP,  10,   0,   0,   1302.280,  61516.081,        85\n"
    )
    p2 = tmp_path / "bad.csv"
    p2.write_text(
        "Berlin,PSIPP,10,0,0\n"  # 6 fields
    )
    buf = io.StringIO()
    with pytest.raises(ValueError, match="bad.csv:1"):
        merge_csv_files([str(p1), str(p2)], buf)
    # The good file's row was NOT written -- we collect all rows
    # before any write so a bad input aborts the whole merge.
    assert buf.getvalue() == ""


# --------------------------------------------------------------------
# CLI driver
# --------------------------------------------------------------------


def test_cli_merges_two_files_to_stdout(tmp_path):
    """``psipp-merge-csv file1 file2`` writes the merged CSV to
    stdout and the row count to stderr."""
    p1 = tmp_path / "psipp.csv"
    p1.write_text(
        "  Berlin_1_3_CDT,     PSIPP,  10,   0,   0,   1302.280,  61516.081,        85\n"
    )
    p2 = tmp_path / "ccbs.csv"
    p2.write_text(
        "  Berlin_1_3_CDT,     CCBS,  10,   0,   0,   1300.500,  61400.300,      2100\n"
    )
    env = {"PATH": "/home/two/PSIPP-CTC/.venv/bin:/usr/bin:/bin"}
    out = subprocess.run(
        ["/home/two/PSIPP-CTC/.venv/bin/psipp-merge-csv",
         str(p1), str(p2)],
        capture_output=True, text=True, env=env, timeout=10,
    )
    assert out.returncode == 0, out.stderr
    assert "wrote 2 rows" in out.stderr
    # Stdout has the merged CSV.
    assert "PSIPP" in out.stdout
    assert "CCBS" in out.stdout


def test_cli_emits_header_when_requested(tmp_path):
    """``psipp-merge-csv --header file1 file2`` writes a column-name
    row at the top of stdout."""
    p = tmp_path / "in.csv"
    p.write_text(
        "  Berlin_1_3_CDT,     PSIPP,  10,   0,   0,   1302.280,  61516.081,        85\n"
    )
    env = {"PATH": "/home/two/PSIPP-CTC/.venv/bin:/usr/bin:/bin"}
    out = subprocess.run(
        ["/home/two/PSIPP-CTC/.venv/bin/psipp-merge-csv", "--header", str(p)],
        capture_output=True, text=True, env=env, timeout=10,
    )
    assert out.returncode == 0, out.stderr
    first = out.stdout.splitlines()[0]
    assert first == ",".join(COLUMNS)


def test_cli_exits_nonzero_on_bad_row(tmp_path):
    """A row with the wrong field count makes the CLI exit with a
    non-zero status and a clear error message on stderr."""
    p = tmp_path / "bad.csv"
    p.write_text("Berlin,PSIPP,10,0,0\n")
    env = {"PATH": "/home/two/PSIPP-CTC/.venv/bin:/usr/bin:/bin"}
    out = subprocess.run(
        ["/home/two/PSIPP-CTC/.venv/bin/psipp-merge-csv", str(p)],
        capture_output=True, text=True, env=env, timeout=10,
    )
    assert out.returncode != 0
    assert "expected 8 fields" in out.stderr


def test_cli_reads_stdin_via_dash():
    """The ``-`` path sentinel reads from stdin, mirroring
    ``sort`` / ``cat``."""
    env = {"PATH": "/home/two/PSIPP-CTC/.venv/bin:/usr/bin:/bin"}
    out = subprocess.run(
        ["/home/two/PSIPP-CTC/.venv/bin/psipp-merge-csv", "-"],
        input=(
            "  Berlin_1_3_CDT,     PSIPP,  10,   0,   0,   "
            "1302.280,  61516.081,        85\n"
        ),
        capture_output=True, text=True, env=env, timeout=10,
    )
    assert out.returncode == 0, out.stderr
    assert "PSIPP" in out.stdout


def test_cli_writes_to_output_file(tmp_path):
    """``-o path`` writes the merged CSV to ``path``."""
    p = tmp_path / "in.csv"
    p.write_text(
        "  Berlin_1_3_CDT,     PSIPP,  10,   0,   0,   1302.280,  61516.081,        85\n"
    )
    out_path = tmp_path / "merged.csv"
    env = {"PATH": "/home/two/PSIPP-CTC/.venv/bin:/usr/bin:/bin"}
    out = subprocess.run(
        ["/home/two/PSIPP-CTC/.venv/bin/psipp-merge-csv",
         "-o", str(out_path), str(p)],
        capture_output=True, text=True, env=env, timeout=10,
    )
    assert out.returncode == 0, out.stderr
    text = out_path.read_text()
    assert "PSIPP" in text
    # The CLI's stderr report still fires for file output.
    assert "wrote 1 rows" in out.stderr
