import numpy as np
import math

def rotation_simple(psi, theta, phi):
    cpsi = math.cos(psi)
    ctheta = math.cos(theta)
    cphi = math.cos(phi)
    spsi = math.sin(psi)
    stheta = math.sin(theta)
    sphi = math.sin(phi)
    rota = np.array([[cpsi*ctheta, (cpsi*stheta*sphi - spsi*cphi), (spsi*sphi+cpsi*stheta*cphi)], [spsi*ctheta,
                    (cpsi*cphi + spsi*stheta*sphi), (spsi*stheta*cphi - cpsi*sphi)], [-stheta, ctheta*sphi, ctheta*cphi]])
    # [cpsi*ctheta, (cpsi*stheta*sphi-spsi*cphi), (spsi*sphi+cpsi*stheta*cphi)]
    # [spsi*ctheta, (cpsi*cphi+spsi*stheta*sphi), (spsi*stheta*cphi-cpsi*sphi)]
    # [-stheta, ctheta*sphi, ctheta*cphi]
    return rota
