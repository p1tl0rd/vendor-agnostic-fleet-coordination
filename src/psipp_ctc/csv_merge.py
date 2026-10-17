"""Merge PSIPP / CCBS benchmark CSVs.

The C++ reference's ``planning_incrementally`` driver and the
Python's ``Benchmark.run_incrementally_all_pairs`` both emit the
same 8-column CSV:

    <problem_name>,<planner_name>,<number_of_agents>,<perm_id>,<count>,
    <makespan>,<sum_of_costs>,<used_time>

with the ``%11s,%6s,%4d,%4d,%4d,%10.3lf,%10.3lf,%10d`` format. There
is no header line in either output.

This module provides a tiny concat-and-validate merger so the
thesis author can run the C++'s ``planning_incrementally`` driver
to obtain the CCBS CSV and the Python's
``Benchmark.run_incrementally_all_pairs`` to obtain the PSIPP CSV,
then merge them into a single dataset for the figures:

    psipp-merge-csv python_psipp.csv ccbs_results.csv > merged.csv

The merger validates that every row has the expected 8 fields and
that the string fields (columns 0 and 1) and the numeric fields
(columns 2-7) are well-formed. The first mismatch raises a
``ValueError`` with the offending file, line number, and content --
so a bad CSV is caught immediately rather than silently corrupting
the merged dataset.

Not the responsibility of this module
--------------------------------------
* Generating the CSVs themselves. Use the Python's
  ``Benchmark.run_incrementally_all_pairs`` (or the wired CLI driver)
  for the PSIPP side; the C++'s ``planning_incrementally`` driver
  for the CCBS side.
* Statistics / figure plotting. The merged CSV is a tidy long-format
  dataset; ``pandas.read_csv`` + ``matplotlib`` (or R) are downstream.
* Per-instance text normalisation. The Continuous-CBS
  ``main.cpp``'s free-form text output is not a CSV; if the thesis
  author has that, it needs a per-instance converter.
"""
from __future__ import annotations

import csv
import io
import sys
from typing import Iterable, Iterator, List, TextIO, Union

#: The 8-column header the C++ and Python both emit. Listed here as
#: a tuple so a future schema bump (e.g. adding ``sum_of_costs``
#: derived metrics) has an obvious single point of update.
COLUMNS: List[str] = [
    "problem_name",
    "planner_name",
    "number_of_agents",
    "perm_id",
    "iteration",
    "makespan",
    "sum_of_costs",
    "used_time",
]

#: Number of comma-separated fields per row.
EXPECTED_FIELD_COUNT: int = 8

PathLike = Union[str, "object"]


def _open_for_read(path: PathLike) -> TextIO:
    """Open a CSV file for reading, or pass through ``"stdin"``.

    The literal ``"stdin"`` / ``"stdout"`` sentinel convention is used
    by ``io/roadmap.py`` and ``benchmark.py`` -- the literal string
    ``"stdin"`` means ``sys.stdin``. Anything else is opened as a
    file.
    """
    if str(path) == "stdin":
        return sys.stdin
    return open(path, "r", newline="")


def iter_rows(paths: Iterable[PathLike]) -> Iterator[List[str]]:
    """Yield validated rows from each CSV in ``paths`` in order.

    Each yielded row is a list of exactly 8 strings. A row with the
    wrong number of fields raises ``ValueError`` with the file name,
    the 1-based line number, and the offending content.

    The C++ and Python both omit the header. A user-supplied header
    (e.g. ``"problem_name,planner_name,..."``) is detected on the
    first input only and skipped silently: a chained merger should
    still validate every subsequent input as data, so the
    "header-looking" line is a *first-file* affordance for users who
    want to read their own data through this tool. The convention is
    documented in :data:`COLUMNS`.
    """
    for path_index, path in enumerate(paths):
        with _open_for_read(path) as f:
            reader = csv.reader(f)
            for line_no, row in enumerate(reader, start=1):
                if not row:
                    # Skip empty lines (the C++ never emits them;
                    # defensiveness only).
                    continue
                if (
                    path_index == 0
                    and line_no == 1
                    and row
                    and row[0].strip() == COLUMNS[0]
                ):
                    # Detect a header line on the FIRST file and
                    # skip it. Subsequent files are treated as data.
                    continue
                if len(row) != EXPECTED_FIELD_COUNT:
                    raise ValueError(
                        f"{path}:{line_no}: expected "
                        f"{EXPECTED_FIELD_COUNT} fields, got {len(row)}: {row!r}"
                    )
                yield row


def validate_field_types(rows: Iterable[List[str]]) -> None:
    """Verify the field types of every row match the C++ format.

    Columns 0-1 are planner / problem names (any non-empty string);
    columns 2-4 are non-negative integers; columns 5-7 are
    non-negative floats. A row whose makespan is ``-1.0`` is a
    well-formed failure sentinel (mirrors the C++'s
    ``%10.3lf``-formatted ``-1.000``) and is allowed.
    """
    for line_no, row in enumerate(rows, start=1):
        problem, planner, n_agents, perm, count, ms, soc, used = row
        if not problem:
            raise ValueError(f"line {line_no}: empty problem_name")
        if not planner:
            raise ValueError(f"line {line_no}: empty planner_name")
        for col_name, value in (
            ("number_of_agents", n_agents),
            ("perm_id", perm),
            ("iteration", count),
        ):
            try:
                iv = int(value)
            except ValueError as e:
                raise ValueError(
                    f"line {line_no}: {col_name}={value!r} is not int"
                ) from e
            if iv < 0:
                raise ValueError(
                    f"line {line_no}: {col_name}={iv} is negative"
                )
        for col_name, value in (
            ("makespan", ms),
            ("sum_of_costs", soc),
            ("used_time", used),
        ):
            try:
                fv = float(value)
            except ValueError as e:
                raise ValueError(
                    f"line {line_no}: {col_name}={value!r} is not float"
                ) from e
            if fv < -1.0 - 1e-9:
                # ``-1.0`` is the failure sentinel; anything below it
                # is a malformed row.
                raise ValueError(
                    f"line {line_no}: {col_name}={fv} is below the "
                    f"-1.0 failure sentinel"
                )


def merge_csv_files(
    paths: Iterable[PathLike],
    out: TextIO,
    *,
    write_header: bool = False,
) -> int:
    """Concatenate the rows of each input CSV into ``out``.

    Parameters
    ----------
    paths : iterable of path-like or ``"stdin"``
        CSV files to merge. Order is preserved in the output.
    out : text stream
        The destination stream. Typically ``sys.stdout`` or an
        open file handle.
    write_header : bool, default ``False``
        When ``True``, prepend a header line with the column names
        from :data:`COLUMNS`. The C++ and Python both omit the
        header; pass ``True`` only if your downstream tooling
        needs one (e.g. ``pandas.read_csv`` with no ``names=`` arg).

    Returns
    -------
    int
        The number of data rows written. Useful for sanity checks
        in the CLI and in tests.

    The merger validates every row via
    :func:`validate_field_types` before writing. A bad row raises
    ``ValueError`` and ``out`` may be partially written; the CLI
    catches this and exits with a non-zero status.
    """
    rows = list(iter_rows(paths))
    validate_field_types(rows)

    if write_header:
        out.write(",".join(COLUMNS) + "\n")
    writer = csv.writer(out, lineterminator="\n")
    for row in rows:
        writer.writerow(row)
    return len(rows)


__all__ = [
    "COLUMNS",
    "EXPECTED_FIELD_COUNT",
    "iter_rows",
    "merge_csv_files",
    "validate_field_types",
]
