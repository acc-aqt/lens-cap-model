"""Write the finger force against squeeze from a squeeze.dat file as CSV.

The quarter model holds half of one finger pad, so the force of one finger is
twice the reaction at the pad's reference node. The squeeze is the reduction
of the cap's outer diameter at the pad, i.e. twice the pad travel after the
initial gap has closed.
"""

import argparse
import re
from pathlib import Path

GAP = 0.05
ROW = re.compile(r"^\s*\d+\s+(\S+)\s+\S+\s+\S+\s*$")


def read_curve(dat: Path) -> list[tuple[float, float]]:
    """Return (squeeze in mm, finger force in N) per converged increment."""
    displacements: list[float] = []
    forces: list[float] = []
    target: list[float] | None = None
    for line in dat.read_text().splitlines():
        header = line.lstrip()
        if header.startswith(("displacements", "forces")):
            if "for set NREF" not in line:
                target = None
            elif header.startswith("displacements"):
                target = displacements
            else:
                target = forces
        elif target is not None and (match := ROW.match(line)):
            target.append(float(match.group(1)))
            target = None
    return [
        (2 * max(-u - GAP, 0.0), -2 * f)
        for u, f in zip(displacements, forces, strict=False)
    ]


def main() -> None:
    """Print the curve as CSV and write it next to the .dat file."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dat", type=Path, help="Path to squeeze.dat")
    dat = parser.parse_args().dat
    rows = ["squeeze_mm,finger_force_N"] + [
        f"{s:.4f},{f:.4f}" for s, f in read_curve(dat)
    ]
    csv = dat.with_name("force_deflection.csv")
    csv.write_text("\n".join(rows) + "\n")
    print("\n".join(rows))


if __name__ == "__main__":
    main()
