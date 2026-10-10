#!/usr/bin/env python3
"""HSE oscillator check for the ControlHat crystal (Wurth 830064296).

Uses the AN2867 critical-transconductance formula
    Gm_crit = 4 * ESR * (2*pi*F)^2 * (C0 + CL)^2
with datasheet values for the crystal (16 MHz, C0 = 7 pF, ESR <= 70 ohm,
CL = 8 pF, drive <= 100 uW) and the STM32F401 limit Gm_crit_max = 1 mA/V.
CL_eff = C1*C2/(C1+C2) + Cstray. Cstray (pins + board) is an estimate.
"""
import math

F = 16e6
C0 = 7e-12
ESR = 70.0          # datasheet maximum
CL_SPEC = 8e-12
GM_MAX = 1e-3       # STM32F401 datasheet Table 37, A/V

def gm_crit(cl, esr=ESR):
    return 4 * esr * (2 * math.pi * F) ** 2 * (C0 + cl) ** 2

def cl_eff(c1, c2, cstray):
    return c1 * c2 / (c1 + c2) + cstray

print("Crystal CL = 8 pF, ESR max 70 ohm, C0 = 7 pF, F = 16 MHz")
print("Gm_crit at the specified load (CL = 8 pF): %.2f mA/V" % (gm_crit(CL_SPEC) * 1e3))
print()
print("Board as built: C108 = C109 = 15 pF")
print("  Cstray   CL_eff   Gm_crit    vs ST limit 1.0 mA/V")
for cs in (3e-12, 4e-12, 5e-12, 7e-12, 10e-12):
    cl = cl_eff(15e-12, 15e-12, cs)
    g = gm_crit(cl)
    print("  %4.0f pF  %5.1f pF  %.2f mA/V  %s" % (cs * 1e12, cl * 1e12, g * 1e3,
          "OVER LIMIT" if g > GM_MAX else "within (%.0f%% of limit)" % (g / GM_MAX * 100)))
print()
print("Load caps needed for CL = 8 pF: C1 = C2 = 2 * (CL - Cstray)")
for cs in (3e-12, 4e-12, 5e-12):
    print("  Cstray %.0f pF -> %.1f pF each" % (cs * 1e12, 2 * (CL_SPEC - cs) * 1e12))
print()
print("Gm_crit with the capacitor values that would fit the 8 pF crystal")
for c, cs in ((8.2e-12, 4e-12), (6.8e-12, 4e-12), (10e-12, 3e-12)):
    cl = cl_eff(c, c, cs)
    print("  C = %.1f pF, Cstray %.0f pF -> CL_eff %.1f pF, Gm_crit %.2f mA/V" % (
        c * 1e12, cs * 1e12, cl * 1e12, gm_crit(cl) * 1e3))
