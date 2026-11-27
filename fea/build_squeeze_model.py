"""Build a CalculiX model of a slip-on lens cap squeezed between two fingers.

The cap profile matches ``src/lens_cap_model/main.py``. A quarter of the cap
(x >= 0, y >= 0) is meshed with cgx; a rigid flat pad stands in for the
finger at +x and is pushed towards the axis. The planes x = 0 and y = 0 are
symmetry planes, so the second finger at -x is implied.
"""

import argparse
import math
import subprocess
from dataclasses import dataclass
from pathlib import Path

CGX = Path.home() / "src/cgx/CalculiX/cgx_2.24/src/cgx"

# Fillet radii from main.py
R_IN = 0.8
R_OUT = 1.0
R_TOP_OUT = 0.9
R_TOP_IN = 0.9

# TPU 95A, Neo-Hookean: E ~ 26 MPa, nu = 0.48
YOUNGS = 26.0
POISSON = 0.48

TOL = 1e-6

# Grounded z spring at the floor centre: removes the free z translation
# before the fingers grip, without carrying load afterwards.
ANCHOR_STIFFNESS = 0.01

C3D20_LINE_LENGTH = 21  # element id + 20 nodes
# Wall faces within this angle of the finger, facing outward, may touch it
CONTACT_SECTOR_DEG = 60.0
MIN_OUTWARD = 0.5

# C3D8/C3D20 faces by corner node positions (0-based), ccx numbering S1..S6
HEX_FACES = (
    (0, 1, 2, 3),
    (4, 7, 6, 5),
    (0, 4, 5, 1),
    (1, 5, 6, 2),
    (2, 6, 7, 3),
    (3, 7, 4, 0),
)


@dataclass(frozen=True)
class Cap:
    """Cap dimensions in mm."""

    inner_diameter: float
    wall: float
    height: float

    @property
    def r_in(self) -> float:
        """Inner wall radius."""
        return self.inner_diameter / 2

    @property
    def r_out(self) -> float:
        """Outer wall radius."""
        return self.r_in + self.wall


@dataclass(frozen=True)
class Squeeze:
    """Finger pad size, position and travel in mm."""

    travel: float
    pad_height: float
    pad_centre_z: float
    pad_half_width: float
    friction: float
    pad_radius: float = 10.0
    gap: float = 0.05


def cap_fbd(cap: Cap) -> str:
    """Return the cgx script that meshes a quarter of the cap."""
    r_f = cap.r_in - R_IN
    z_w0 = cap.wall + R_IN
    z_g = cap.height - R_TOP_OUT
    z_h = cap.height - R_TOP_IN
    core = 14.0
    core_corner = 11.0
    r_m = r_f / math.sqrt(2)
    o, w, h = cap.r_out, cap.wall, cap.height
    return f"""\
pnt pO 0 0 0
pnt pA {r_f} 0 0
pnt pB {o - R_OUT} 0 0
pnt pBc {o - R_OUT} 0 {R_OUT}
pnt pC {o} 0 {R_OUT}
pnt pD {o} 0 {z_w0}
pnt pE {cap.r_in} 0 {z_w0}
pnt pEc {r_f} 0 {z_w0}
pnt pF {r_f} 0 {w}
pnt pG {o} 0 {z_g}
pnt pGc {o - R_TOP_OUT} 0 {z_g}
pnt pI {o - R_TOP_OUT} 0 {h}
pnt pJ {cap.r_in + R_TOP_IN} 0 {h}
pnt pHc {cap.r_in + R_TOP_IN} 0 {z_h}
pnt pH {cap.r_in} 0 {z_h}
line lAF pA pF 4
line lFE pF pE pEc 6
line lED pE pD 4
line lDC pD pC 2
line lCB pC pB pBc 2
line lBA pB pA 2
lcmb cbot + lDC + lCB + lBA
surf scor blend lAF lFE lED cbot
line lDG pD pG 24
line lGH pG pH 4
line lHE pH pE 24
surf swal blend lED lDG lGH lHE
line lGI pG pI pGc 2
line lIJ pI pJ 4
line lJH pJ pH pHc 2
surf srim blend lGH lGI lIJ lJH
seta prof s scor swal srim
swep prof prof2 rot z 90 64
pnt pP1 {core} 0 0
pnt pCc {core_corner} {core_corner} 0
pnt pP2 0 {core} 0
pnt pRM {r_m} {r_m} 0
pnt pR2 0 {r_f} 0
line lOP1 pO pP1 32
line lP1C pP1 pCc 32
line lCP2 pCc pP2 32
line lP2O pP2 pO 32
line lP1R pP1 pA 16
line lRRM pA pRM pO 32
line lRMC pRM pCc 16
line lRMR2 pRM pR2 pO 32
line lR2P2 pR2 pP2 16
surf sco blend lOP1 lP1C lCP2 lP2O
surf sfa blend lP1R lRRM lRMC lP1C
surf sfb blend lRMC lRMR2 lR2P2 lCP2
seta flr s sco sfa sfb
swep flr flr2 tra 0 0 {w} 4
elty all he20r
mesh all
merg n all 0.001
send all abq
"""


def read_msh(
    path: Path,
) -> tuple[dict[int, tuple[float, ...]], dict[int, list[int]]]:
    """Read nodes and C3D20R elements from a cgx abq mesh."""
    nodes: dict[int, tuple[float, ...]] = {}
    elements: dict[int, list[int]] = {}
    block = ""
    pending: list[int] = []
    for raw in path.read_text().splitlines():
        line = raw.strip().rstrip(",")
        if not line:
            continue
        if line.startswith("*"):
            block = line.split(",")[0].upper()
            continue
        values = [v for v in line.split(",") if v.strip()]
        if block == "*NODE":
            nodes[int(values[0])] = tuple(float(v) for v in values[1:4])
        elif block == "*ELEMENT":
            pending += [int(v) for v in values]
            if len(pending) == C3D20_LINE_LENGTH:
                elements[pending[0]] = pending[1:]
                pending = []
    return nodes, elements


def free_faces(elements: dict[int, list[int]]) -> list[tuple[int, int]]:
    """Return (element, face number) for faces that belong to one element."""
    owners: dict[frozenset[int], list[tuple[int, int]]] = {}
    for eid, conn in elements.items():
        for number, corners in enumerate(HEX_FACES, start=1):
            key = frozenset(conn[i] for i in corners)
            owners.setdefault(key, []).append((eid, number))
    return [faces[0] for faces in owners.values() if len(faces) == 1]


def centroid(points: list[tuple[float, ...]]) -> tuple[float, float, float]:
    """Return the mean of the points."""
    n = len(points)
    return (
        sum(p[0] for p in points) / n,
        sum(p[1] for p in points) / n,
        sum(p[2] for p in points) / n,
    )


def outward_direction(
    nodes: dict[int, tuple[float, ...]], conn: list[int], face: int
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return a face's centroid and unit vector from element to face centre."""
    corners = conn[:8]
    fc = centroid([nodes[corners[i]] for i in HEX_FACES[face - 1]])
    ec = centroid([nodes[n] for n in corners])
    d = tuple(f - e for f, e in zip(fc, ec, strict=True))
    length = math.sqrt(sum(c * c for c in d))
    return fc, (d[0] / length, d[1] / length, d[2] / length)


def contact_faces(
    nodes: dict[int, tuple[float, ...]],
    elements: dict[int, list[int]],
    cap: Cap,
    squeeze: Squeeze,
) -> list[tuple[int, int]]:
    """Return outer-wall faces the finger pad can reach."""
    z_lo = squeeze.pad_centre_z - squeeze.pad_height / 2 - 2.0
    z_hi = squeeze.pad_centre_z + squeeze.pad_height / 2 + 2.0
    selected = []
    for eid, face in free_faces(elements):
        fc, d = outward_direction(nodes, elements[eid], face)
        r = math.hypot(fc[0], fc[1])
        if r < cap.r_out - R_OUT or not z_lo <= fc[2] <= z_hi:
            continue
        theta = math.degrees(math.atan2(fc[1], fc[0]))
        radial = (d[0] * fc[0] + d[1] * fc[1]) / r
        if theta < CONTACT_SECTOR_DEG and radial > MIN_OUTWARD:
            selected.append((eid, face))
    return selected


def pad_mesh(
    cap: Cap, squeeze: Squeeze, first_node: int, first_element: int
) -> tuple[dict[int, tuple[float, ...]], dict[int, list[int]], int, int]:
    """Mesh the rigid finger pad and return nodes, C3D8s, ref and rot node.

    The pad face is curved in z (radius ``pad_radius``) like a fingertip, so
    the cap never meets a sharp pad edge.
    """
    x0 = cap.r_out + squeeze.gap
    ny, nz = 4, 12
    z0 = squeeze.pad_centre_z - squeeze.pad_height / 2
    ys = [squeeze.pad_half_width * j / ny for j in range(ny + 1)]
    zs = [z0 + squeeze.pad_height * k / nz for k in range(nz + 1)]
    ids: dict[tuple[int, int, int], int] = {}
    nodes: dict[int, tuple[float, ...]] = {}
    nid = first_node
    for k, z in enumerate(zs):
        dz = z - squeeze.pad_centre_z
        face = (
            x0 + squeeze.pad_radius - math.sqrt(squeeze.pad_radius**2 - dz**2)
        )
        for i, x in enumerate((face, face + 2.0)):
            for j, y in enumerate(ys):
                ids[i, j, k] = nid
                nodes[nid] = (x, y, z)
                nid += 1
    elements: dict[int, list[int]] = {}
    eid = first_element
    for j in range(ny):
        for k in range(nz):
            elements[eid] = [
                ids[0, j, k],
                ids[0, j + 1, k],
                ids[0, j + 1, k + 1],
                ids[0, j, k + 1],
                ids[1, j, k],
                ids[1, j + 1, k],
                ids[1, j + 1, k + 1],
                ids[1, j, k + 1],
            ]
            eid += 1
    ref, rot = nid, nid + 1
    nodes[ref] = (x0, 0.0, squeeze.pad_centre_z)
    nodes[rot] = (x0, 0.0, squeeze.pad_centre_z)
    return nodes, elements, ref, rot


def pad_contact_faces(
    nodes: dict[int, tuple[float, ...]], elements: dict[int, list[int]]
) -> list[tuple[int, int]]:
    """Return the pad faces that point towards the cap (-x)."""
    selected = []
    for eid, face in free_faces(elements):
        _, d = outward_direction(nodes, elements[eid], face)
        if d[0] < -MIN_OUTWARD:
            selected.append((eid, face))
    return selected


def write_lines(path: Path, header: str, rows: list[str]) -> None:
    """Write a keyword line followed by data lines."""
    path.write_text("\n".join([header, *rows]) + "\n")


def id_rows(ids: list[int]) -> list[str]:
    """Format ids eight per line."""
    return [
        ", ".join(str(i) for i in ids[n : n + 8])
        for n in range(0, len(ids), 8)
    ]


def deck(
    squeeze: Squeeze, ref: int, rot: int, spring: int, anchor: int
) -> str:
    """Return the root CalculiX deck."""
    shear = YOUNGS / (2 * (1 + POISSON))
    bulk = YOUNGS / (3 * (1 - 2 * POISSON))
    c10 = shear / 2
    d1 = 2 / bulk
    return f"""\
** Lens cap squeezed between two fingers: quarter model, rigid pad at +x.
*INCLUDE, INPUT=all.msh
*INCLUDE, INPUT=pad.msh
*INCLUDE, INPUT=sets.nam
*INCLUDE, INPUT=surfaces.sur
*MATERIAL, NAME=TPU95A
*HYPERELASTIC, NEO HOOKE
{c10:.4f}, {d1:.5f}
*SOLID SECTION, ELSET=Eall, MATERIAL=TPU95A
*MATERIAL, NAME=PAD
*ELASTIC
210000., 0.3
*SOLID SECTION, ELSET=Epad, MATERIAL=PAD
*RIGID BODY, NSET=Npad, REF NODE={ref}, ROT NODE={rot}
*ELEMENT, TYPE=SPRING1, ELSET=Eanchor
{spring}, {anchor}
*SPRING, ELSET=Eanchor
3
{ANCHOR_STIFFNESS}
*SURFACE INTERACTION, NAME=FINGER
*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=LINEAR
1000.
*FRICTION
{squeeze.friction}, 100.
*CONTACT PAIR, INTERACTION=FINGER, TYPE=SURFACE TO SURFACE
Swall, Spad
*BOUNDARY
Nsymx, 1, 1
Nsymy, 2, 2
{ref}, 2, 3
{rot}, 1, 3
*STEP, NLGEOM, INC=500
*STATIC
0.02, 1., 1e-6, 0.05
*BOUNDARY
{ref}, 1, 1, {-(squeeze.travel + squeeze.gap)}
*NODE PRINT, NSET=Nref, FREQUENCY=1
U, RF
*NODE PRINT, NSET=Nanchor, FREQUENCY=1
U
*NODE FILE
U, RF
*EL FILE
S, E
*CONTACT FILE
CDIS, CSTR
*END STEP
"""


def build(cap: Cap, squeeze: Squeeze, out: Path) -> None:
    """Mesh the cap, add the pad and sets, and write the deck to ``out``."""
    out.mkdir(parents=True, exist_ok=True)
    (out / "cap.fbd").write_text(cap_fbd(cap))
    subprocess.run(  # noqa: S603
        [str(CGX), "-bg", "cap.fbd"],
        cwd=out,
        check=True,
        capture_output=True,
    )
    nodes, elements = read_msh(out / "all.msh")

    pad_nodes, pad_elements, ref, rot = pad_mesh(
        cap, squeeze, max(nodes) + 1, max(elements) + 1
    )
    node_rows = [
        f"{n}, {x:.6f}, {y:.6f}, {z:.6f}" for n, (x, y, z) in pad_nodes.items()
    ]
    element_rows = [
        f"{e}, " + ", ".join(str(n) for n in conn)
        for e, conn in pad_elements.items()
    ]
    (out / "pad.msh").write_text(
        "\n".join(
            [
                "*NODE, NSET=Npad",
                *node_rows,
                "*ELEMENT, TYPE=C3D8, ELSET=Epad",
                *element_rows,
            ]
        )
        + "\n"
    )

    symx = [n for n, p in nodes.items() if abs(p[0]) < TOL]
    symy = [n for n, p in nodes.items() if abs(p[1]) < TOL]
    anchor = [n for n, p in nodes.items() if max(map(abs, p)) < TOL]
    sets = []
    for name, ids in (
        ("Nsymx", symx),
        ("Nsymy", symy),
        ("Nanchor", anchor),
        ("Nref", [ref]),
    ):
        sets += [f"*NSET, NSET={name}", *id_rows(sorted(ids))]
    (out / "sets.nam").write_text("\n".join(sets) + "\n")

    wall = contact_faces(nodes, elements, cap, squeeze)
    pad = pad_contact_faces(pad_nodes, pad_elements)
    (out / "surfaces.sur").write_text(
        "\n".join(
            [
                "*SURFACE, NAME=Swall, TYPE=ELEMENT",
                *(f"{e}, S{f}" for e, f in sorted(wall)),
                "*SURFACE, NAME=Spad, TYPE=ELEMENT",
                *(f"{e}, S{f}" for e, f in sorted(pad)),
            ]
        )
        + "\n"
    )
    spring = max(pad_elements) + 1
    (out / "squeeze.inp").write_text(
        deck(squeeze, ref, rot, spring, anchor[0])
    )

    print(
        f"{out / 'squeeze.inp'}: {len(nodes)} cap nodes, {len(elements)} "
        f"C3D20R, symx={len(symx)} symy={len(symy)} anchor={len(anchor)} "
        f"wall faces={len(wall)} pad faces={len(pad)}"
    )


def main() -> None:
    """Parse arguments and build the model."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--id", type=float, default=73.0, help="Inner diameter (mm)"
    )
    parser.add_argument(
        "--wall", type=float, default=2.0, help="Wall thickness (mm)"
    )
    parser.add_argument(
        "--height", type=float, default=20.0, help="Height (mm)"
    )
    parser.add_argument(
        "--travel", type=float, default=5.0, help="Pad travel per finger (mm)"
    )
    parser.add_argument("--pad-height", type=float, default=15.0)
    parser.add_argument(
        "--pad-z", type=float, default=11.5, help="Pad centre height (mm)"
    )
    parser.add_argument("--pad-half-width", type=float, default=12.0)
    parser.add_argument(
        "--friction",
        type=float,
        default=0.5,
        help="Finger friction coefficient",
    )
    args = parser.parse_args()

    cap = Cap(args.id, args.wall, args.height)
    squeeze = Squeeze(
        args.travel,
        args.pad_height,
        args.pad_z,
        args.pad_half_width,
        args.friction,
    )
    tag = f"{args.id:g}".replace(".", "_")
    build(cap, squeeze, Path(__file__).parent / f"out-{tag}")


if __name__ == "__main__":
    main()
