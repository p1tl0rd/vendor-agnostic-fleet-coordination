"""CLI: merge multiple PSIPP / CCBS benchmark CSVs.

    psipp-merge-csv file1.csv [file2.csv ...] > merged.csv
    psipp-merge-csv -o merged.csv file1.csv file2.csv
    psipp-merge-csv --header python_psipp.csv ccbs_results.csv > merged.csv
    psipp-merge-csv - python_psipp.csv ccbs_results.csv < extras.csv > merged.csv

The merger validates every row's 8-column shape and field types
(see :mod:`psipp_ctc.csv_merge`) and exits non-zero on the first
malformed row. The C++ reference and the Python port both emit
the same 8-column format with no header; pass ``--header`` only
if your downstream tooling needs a column-name row.

The literal path ``"-"`` is accepted as a synonym for ``stdin``,
mirrors ``sort`` / ``cat`` conventions, and is the recommended
way to pipe a single extra CSV into the merger:

    cat extras.csv | psipp-merge-csv - main.csv > merged.csv
"""
from __future__ import annotations

import argparse
import sys
from typing import List

from psipp_ctc.csv_merge import merge_csv_files


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "files",
        nargs="+",
        help=(
            "CSV files to merge, in order. Use '-' to read from stdin "
            "(equivalent to 'stdin')."
        ),
    )
    parser.add_argument(
        "-o", "--output",
        default="-",
        help=(
            "Output path or '-' for stdout (default: stdout). The "
            "literal 'stdout' is also accepted for parity with the "
            "I/O sentinel convention used elsewhere in this tree."
        ),
    )
    parser.add_argument(
        "--header",
        action="store_true",
        help=(
            "Prepend a column-name header to the output. Both the "
            "C++ reference and the Python port omit the header; "
            "pass --header only if your downstream tooling needs "
            "one (e.g. pandas.read_csv without names=)."
        ),
    )
    args = parser.parse_args()

    # Normalise sentinels. The C++/Python I/O convention uses the
    # literal "stdin" / "stdout"; the POSIX / GNU convention uses
    # "-". Accept both.
    files: List[str] = []
    for f in args.files:
        if f == "-":
            files.append("stdin")
        else:
            files.append(f)

    out_name = args.output
    if out_name == "-":
        out_name = "stdout"

    if out_name == "stdout":
        out = sys.stdout
        close_after = False
    else:
        out = open(out_name, "w", newline="")
        close_after = True

    try:
        n_rows = merge_csv_files(files, out, write_header=args.header)
    finally:
        if close_after:
            out.close()
    sys.stderr.write(f"psipp-merge-csv: wrote {n_rows} rows\n")


if __name__ == "__main__":
    main()
