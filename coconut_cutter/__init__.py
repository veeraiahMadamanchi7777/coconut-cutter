"""Tender-coconut opening machine: parametric design, controller and plant-scale verification."""
from .coconut import Coconut, CoconutPopulation, Dist
from .design import Design, Requirements, optimize, plant_air, size
from .mechanics import MaterialModel, Punch, calibrate, peak_force

__all__ = [
    "Coconut", "CoconutPopulation", "Dist", "Design", "Requirements", "optimize", "plant_air",
    "size", "MaterialModel", "Punch", "calibrate", "peak_force",
]
__version__ = "1.0.0"
