#!/usr/bin/env python

import  sympy as sp
import numpy as np
from sympy.solvers import solve


t, P00, P01, P02, P10, P11, P12, P20, P21, P22, r1, r2, r3 = sp.symbols('t P00 P01 P02 P10 P11 P12 P20 P21 P22 r1 r2 r3')

# create vectors
P0 = sp.Matrix([P00, P01, P02])
P1 = sp.Matrix([P10, P11, P12])
P2 = sp.Matrix([P20 ,P21, P22])
r = sp.Matrix([r1, r2, r3]) # remote points


# bezier
B = (1-t)*(1-t) * P0 + 2 * (1-t)*t*P1 + t*t*P2

# vector B - r
v = B - r

# norm squared
n = v.dot(v)

# take deivative
dn = sp.diff(n, t)

# find minimum t
#s = solve(dn, t)






