"""Cutting-force model for a serrated tubular crown punch.

The punch is a thin-walled tube with a serrated, inclined crown. As it
advances through the nut, the resisting force is

    F(z) = P · e · q(z) · k  +  P · μ · p_lat · min(z, z_wall)

where
    P       crown perimeter (π·D)                                 [mm]
    e       fraction of the edge engaged at once (shear angle)   [-]
    q(z)    edge cutting resistance of the layer at depth z       [N/mm]
    k       per-nut strength multiplier                           [-]
    μ·p_lat wall friction from husk springing back on the tube    [MPa]

The default resistances are engineering estimates for fresh green husk and
tender (jelly-stage) shell. They MUST be calibrated on a load-cell rig before
release — use :func:`calibrate` with measured force/depth traces.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from statistics import fmean
from typing import Iterable, Sequence, Tuple

from .coconut import Coconut


@dataclass(frozen=True)
class MaterialModel:
    q_husk: float = 6.0       # N/mm of engaged edge, fresh green husk
    q_shell: float = 22.0     # N/mm of engaged edge, tender shell
    mu: float = 0.35          # tube/husk friction coefficient (wet)
    p_lat: float = 0.04       # MPa lateral springback pressure of husk


@dataclass(frozen=True)
class Punch:
    diameter: float = 34.0        # mm, crown outer diameter (= hole size)
    engagement: float = 0.45      # serration engagement fraction (shear-angle crown)
    min_overtravel: float = 8.0   # mm past the shell needed for a clean, burr-free hole

    @property
    def perimeter(self) -> float:
        return math.pi * self.diameter


def force_at_depth(z: float, nut: Coconut, punch: Punch, mat: MaterialModel) -> float:
    """Resisting force [N] at penetration depth ``z`` [mm] below first contact."""
    if z <= 0:
        return 0.0
    P = punch.perimeter
    friction = P * mat.mu * mat.p_lat * min(z, nut.husk_top + nut.shell)
    if z <= nut.husk_top:
        edge = mat.q_husk
    elif z <= nut.husk_top + nut.shell:
        edge = mat.q_shell
    else:
        edge = 0.0  # broken through into the cavity
    return P * punch.engagement * edge * nut.strength + friction


def peak_force(nut: Coconut, punch: Punch, mat: MaterialModel) -> float:
    """Analytic peak: maximum is at the end of the shell layer (friction is monotone)."""
    z_end = nut.husk_top + nut.shell
    return max(
        force_at_depth(nut.husk_top, nut, punch, mat),
        force_at_depth(z_end, nut, punch, mat),
    )


def cut_energy(nut: Coconut, punch: Punch, mat: MaterialModel, depth: float, steps: int = 200) -> float:
    """Work [J] to drive the punch ``depth`` mm into the nut (trapezoidal integration)."""
    dz = depth / steps
    total = 0.0
    prev = force_at_depth(0.0, nut, punch, mat)
    for i in range(1, steps + 1):
        cur = force_at_depth(i * dz, nut, punch, mat)
        total += 0.5 * (prev + cur) * dz
        prev = cur
    return total / 1000.0  # N·mm -> J


def required_depth(nut: Coconut, punch: Punch) -> float:
    """Minimum penetration for a clean hole in this nut."""
    return nut.husk_top + nut.shell + punch.min_overtravel


Trace = Sequence[Tuple[float, float]]  # (depth mm, force N)


def calibrate(
    samples: Iterable[Tuple[Coconut, Trace]],
    punch: Punch,
    base: MaterialModel = MaterialModel(),
) -> MaterialModel:
    """Fit ``q_husk``/``q_shell`` from load-cell traces.

    For each sample, the friction term (from ``base``) is subtracted and the
    remainder divided by ``P·e·k``. Points inside each layer are averaged.
    The nut's ``strength`` should be 1.0 for rig samples (it is unknown a
    priori), so the fitted q values represent the population mean.
    """
    husk_q, shell_q = [], []
    P, e = punch.perimeter, punch.engagement
    for nut, trace in samples:
        for z, f in trace:
            if z <= 0:
                continue
            friction = P * base.mu * base.p_lat * min(z, nut.husk_top + nut.shell)
            q = (f - friction) / (P * e * nut.strength)
            if z <= nut.husk_top:
                husk_q.append(q)
            elif z <= nut.husk_top + nut.shell:
                shell_q.append(q)
    if not husk_q or not shell_q:
        raise ValueError("traces must contain points in both husk and shell layers")
    return replace(base, q_husk=fmean(husk_q), q_shell=fmean(shell_q))
