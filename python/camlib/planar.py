"""Line/arc planar regions. Bulge=tan(signed sweep/4); arcs are at most pi.

Construction returns components/holes and explicit approximation diagnostics.
Stock, motion and reachability are separate downstream operations.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Boundary:
    vertices: tuple

    @classmethod
    def polygon(cls, points):
        return cls(tuple((x, y, 0) for x, y in points))

    @classmethod
    def circle(cls, center, radius):
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError("circle radius must be finite and positive")
        x, y = center
        return cls(((x - radius, y, 1), (x + radius, y, 1)))

    def to_dict(self):
        return {"vertices": self.vertices}


@dataclass(frozen=True)
class Component:
    outer: Boundary
    holes: tuple = ()

    def to_dict(self):
        return {"outer": self.outer.to_dict(), "holes": [hole.to_dict() for hole in self.holes]}


@dataclass(frozen=True)
class PlanarRegion:
    name: str
    components: tuple
    boundary_error_mm: float = 0
    source: object = None

    def to_dict(self):
        return {
            "version": 1,
            "units": "mm",
            "name": self.name,
            "boundary_error_mm": self.boundary_error_mm,
            "components": [component.to_dict() for component in self.components],
            "source": self.source,
        }

    @classmethod
    def from_dict(cls, data):
        if data["version"] != 1 or data["units"] != "mm":
            raise ValueError("planar region requires version 1 and mm")

        def boundary(b):
            return Boundary(tuple(tuple(v) for v in b["vertices"]))

        return cls(
            data["name"],
            tuple(Component(boundary(c["outer"]), tuple(boundary(h) for h in c["holes"])) for c in data["components"]),
            data["boundary_error_mm"],
            data.get("source"),
        )

    @classmethod
    def from_fixture(cls, fixture):
        if fixture["version"] != 1 or fixture["units"] != "mm":
            raise ValueError("fixture requires version 1 and mm")
        return cls(
            fixture["name"],
            (
                Component(
                    Boundary.polygon(fixture["outer"]), tuple(Boundary.polygon(h) for h in fixture.get("holes", []))
                ),
            ),
            fixture["boundary_error_mm"],
            fixture.get("source"),
        )
