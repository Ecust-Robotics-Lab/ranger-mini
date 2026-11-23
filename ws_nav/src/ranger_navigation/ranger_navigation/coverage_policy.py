"""One coverage envelope for guidance, optimization and independent validation."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class CoveragePolicy:
    total_margin: float = .025
    protected_margin: float = .015
    unknown_cell_budget: int = 8
    allow_boundary_unknowns: bool = True
    protected_unknown_cell_budget: int = 0

    def __post_init__(self):
        if (not math.isfinite(self.total_margin) or not math.isfinite(self.protected_margin) or
                not .015<=self.protected_margin<=self.total_margin<=.10 or
                type(self.unknown_cell_budget) is not int or not 0<=self.unknown_cell_budget<=16 or
                type(self.allow_boundary_unknowns) is not bool):
            raise ValueError('invalid coverage margin/edge-cell allowance')
        if type(self.protected_unknown_cell_budget) is not int or not 0 <= self.protected_unknown_cell_budget <= 16:
            raise ValueError('coverage.protected_unknown_cell_budget must be an integer from 0 to 16')
