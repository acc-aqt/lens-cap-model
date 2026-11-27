# Finger-squeeze FEA (CalculiX)

Simulates squeezing the elastomer lens cap between two fingers.

## Model

- **Geometry:** the cap profile from `src/lens-cap-model/main.py`, including all
  four fillets. One quarter (x ≥ 0, y ≥ 0) is meshed with C3D20R hexahedra in cgx.
  The planes x = 0 and y = 0 are symmetry planes, which also stand in for the
  second finger at −x.
- **Material:** TPU 95A as Neo-Hookean (E = 26 MPa, ν = 0.48, giving C10 = 4.39 MPa
  and D1 = 0.0092 /MPa).
- **Finger:** a rigid pad with its face curved in z like a fingertip (radius 10 mm,
  15 mm tall, centred at z = 11.5 mm). It touches the outer wall through
  surface-to-surface contact with friction μ = 0.5 and moves `--travel` mm towards
  the axis, so the diameter shrinks by twice that.
- **Support:** none apart from symmetry. The fingers hold the cap. A 0.01 N/mm
  spring in z at the floor centre only removes the free z motion before the
  fingers grip.
- Large deformations (`NLGEOM`).

## Run

```sh
../.venv/bin/python build_squeeze_model.py            # writes out-73/
cd out-73 && ccx -i squeeze                           # a few minutes
../../.venv/bin/python ../force_deflection.py squeeze.dat   # force_deflection.csv
cgx squeeze.frd squeeze.inp                           # view the results
```

`build_squeeze_model.py -h` lists the options (cap size, travel, pad position,
friction). It expects cgx at `~/src/cgx/CalculiX/cgx_2.24/src/cgx`. The finger
force in `force_deflection.csv` is for one finger, already doubled from the
half-pad in the quarter model.
