from pycuber import *
from pycuber.solver import CFOPSolver
import numpy as np

f = np.zeros([3,3])
b = np.ones([3,3])
l =	np.ones([3,3]) + np.ones([3,3])
r = np.ones([3,3]) + np.ones([3,3]) + np.ones([3,3])
u = np.ones([3,3]) + np.ones([3,3]) + np.ones([3,3]) + np.ones([3,3])
d = np.ones([3,3]) + np.ones([3,3]) + np.ones([3,3]) + np.ones([3,3]) + np.ones([3,3])

colordic={0:'green', 1:'blue', 2:'white', 3:'yellow', 4:'red', 5:'orange'}

c = Cube([
	Corner(B=Square(colordic[b[2][2]]),
		   L=Square(colordic[l[2][0]]),
		   D=Square(colordic[d[2][0]])),

	Corner(R=Square(colordic[r[2][0]]),
		   D=Square(colordic[d[0][2]]),
		   F=Square(colordic[f[2][2]])),

	Edge(L=Square(colordic[l[1][2]]),
		 F=Square(colordic[f[1][0]])),

	Edge(U=Square(colordic[u[1][0]]),
		 L=Square(colordic[l[0][1]])),

	Edge(R=Square(colordic[r[1][2]]),
		 B=Square(colordic[b[1][0]])),

	Edge(D=Square(colordic[d[0][1]]),
		 F=Square(colordic[f[2][1]])),

	Edge(R=Square(colordic[r[2][1]]),
		 D=Square(colordic[d[1][2]])),

	Centre(R=Square(colordic[r[1][1]])),

	Centre(U=Square(colordic[u[1][1]])),

	Corner(R=Square(colordic[r[0][2]]),
		   B=Square(colordic[b[0][0]]),
		   U=Square(colordic[u[0][2]])),

	Edge(B=Square(colordic[b[2][1]]),
		 D=Square(colordic[d[0][1]])),

	Corner(L=Square(colordic[l[2][2]]),
		   D=Square(colordic[d[0][0]]),
		   F=Square(colordic[f[2][0]])),

	Corner(U=Square(colordic[u[2][0]]),
		   L=Square(colordic[l[0][2]]),
		   F=Square(colordic[f[0][0]])),

	Edge(R=Square(colordic[r[0][1]]),
		 U=Square(colordic[u[1][2]])),

	Corner(R=Square(colordic[r[0][0]]),
		   U=Square(colordic[u[2][2]]),
		   F=Square(colordic[f[0][2]])),

	Edge(B=Square(colordic[b[1][2]]),
		 L=Square(colordic[l[1][0]])),

	Edge(L=Square(colordic[l[2][1]]),
		 D=Square(colordic[d[1][0]])),

	Centre(L=Square(colordic[l[1][1]])),

	Corner(R=Square(colordic[r[2][2]]),
		   B=Square(colordic[b[2][0]]),
		   D=Square(colordic[d[2][2]])),

	Edge(U=Square(colordic[u[2][1]]),
		 F=Square(colordic[f[0][1]])),

	Centre(F=Square(colordic[f[1][1]])),

	Edge(R=Square(colordic[r[1][0]]),
		 F=Square(colordic[f[1][2]])),

	Corner(B=Square(colordic[b[0][2]]),
		   U=Square(colordic[u[0][0]]),
		   L=Square(colordic[l[0][0]])),

	Centre(D=Square(colordic[d[1][1]])),

	Edge(B=Square(colordic[b[0][1]]),
		 U=Square(colordic[u[0][1]])),

	Centre(B=Square(colordic[b[1][1]]))
	])

print(c)

solver = CFOPSolver(c)
solution = solver.solve(suppress_progress_messages=True)
length = len(solution)
solution = str(solution)
i = 0

while True:
	if i >= length:
		break

