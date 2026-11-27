"""Build a parametric slip-on lens cap and export it as STEP and STL."""

import argparse

import cadquery as cq


def build_part(
    inner_diameter: float, wall: float, height: float
) -> cq.Workplane:
    """Revolve the cap profile (dimensions in mm) into a solid."""
    r_in = 0.8  # inner bottom radius
    r_out = 1.0  # outer bottom radius

    r_top_out = 0.9  # outer rim radius (leading edge)
    r_top_in = 0.9  # inner rim radius (leading edge)

    inner_radius = inner_diameter / 2.0
    outer_radius = inner_radius + wall

    # Guard checks (avoid self-intersections)
    if r_in >= wall:
        raise ValueError("R_in must be smaller than wall thickness.")
    if r_out >= wall:
        raise ValueError("R_out should be smaller than wall thickness.")
    if r_top_out >= wall:
        raise ValueError("R_top_out should be smaller than wall thickness.")
    if r_top_in >= wall:
        raise ValueError("R_top_in should be smaller than wall thickness.")
    if (r_top_out + r_top_in) >= wall:
        raise ValueError(
            "R_top_out + R_top_in must be smaller than wall thickness."
        )

    wp = cq.Workplane("XZ")

    # Build profile with bottom radii + top rim radii
    profile = (
        wp.moveTo(0, 0)
        # bottom to outer corner (leaving room for bottom outer radius)
        .lineTo(outer_radius - r_out, 0)
        .radiusArc((outer_radius, r_out), -r_out)
        # outer wall up, stop short for top outer radius
        .lineTo(outer_radius, height - r_top_out)
        .radiusArc((outer_radius - r_top_out, height), -r_top_out)
        # top face inward, stop short for top inner radius
        .lineTo(inner_radius + r_top_in, height)
        .radiusArc((inner_radius, height - r_top_in), -r_top_in)
        # inner wall down to inner-bottom radius start
        .lineTo(inner_radius, wall + r_in)
        .radiusArc((inner_radius - r_in, wall), r_in)
        # inner floor back to axis
        .lineTo(0, wall)
        .close()
    )

    return profile.revolve(360, (0, 0, 0), (0, 1, 0))


def parse_args() -> tuple[float, float, float]:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Generate a parametric slip-on lens cap (CadQuery)"
    )

    parser.add_argument(
        "--id",
        type=float,
        default=74.0,
        help="Inner diameter in mm (default: 74.0)",
    )

    parser.add_argument(
        "--wall",
        type=float,
        default=2.0,
        help="Wall thickness in mm (default: 2.0)",
    )

    parser.add_argument(
        "--height",
        type=float,
        default=20.0,
        help="Total height in mm (default: 20.0)",
    )

    args = parser.parse_args()

    return args.id, args.wall, args.height


def main() -> None:
    """Build the cap from the command line and export STEP and STL."""
    inner_diameter, wall, height = parse_args()
    part = build_part(inner_diameter, wall, height)

    filename = f"slip-on-cap-{inner_diameter}".replace(".", "_")

    cq.exporters.export(part, f"{filename}.step")
    cq.exporters.export(part, f"{filename}.stl")


if __name__ == "__main__":
    main()
