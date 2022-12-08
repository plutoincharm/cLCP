# cLCP
The problem involves contact dynamics<br />
Problem 1<br />
<br />
Since we wish to find the joint reaction forces, the normal reactions at the floor and wall, as well as the link
accelerations, we make use of the Generalized Augmented Lagrangian approach. We start by writing the 4 joint
constraint equations, with the lagrange multiplier corresponding to each of these constraints being equal to the
joint reaction force. They are <br />
<br />
jx12 = x1 − (L2 − r2) cos θ2 − r1 cos θ1 − x2 <br />
jy12 = y1 − (L2 − r2) sin θ2 − r1 sin θ1 − y2 <br />
jx23 = x2 − r2 cos θ2 − x3 <br />
jy23 = y2 − r2 sin θ2 − y3 <br />
<br />
where (x1, y1), (x2, y2) and (x3, y3) are the coordinates of the centre of mass of links 1, 2 and 3 respectively. These
constraint equations are differentiated twice to obtain a matrix equation of the form <br />
<br />
Cq * qddot = Gq <br />
<br />
where q is the transpose of  {x1, y1, θ1, x2, y2, θ2, x3, y3}<br />
and qddot is the vector of accelerations. Since link 1 slides on a wall having friction, the corresponding constraint
equation is given by <br />
<br />
l1 = (x1 + (L1 −r1) cos θ1)nx1 + (y1 + (L1 −r1) sin θ1)ny1 −μ((x1 + (L1 −r1) cos θ1)tx1 + (y1 + (L1 −r1) sin θ1)ty1) <br />
<br />
where {nx1, ny1}T and {tx1, ty1}T are the normal and tangential vectors respectively, at the point of contact. The
lagrange multiplier associated with this constraint is the normal reaction at the wall. <br />
This constraint equation is differentiated twice to obtain a matrix equation of the form <br />
<br />
Cf * qddot = Gf
<br />
Similarly, slider block 3 slides on a frictionless surface, and so the corresponding constraint equation is given by<br />
<br />
l3 = x3nx3 + (y3 − h3)ny3 <br />
<br />
where {nx3, ny3}T is the normal vector at the point of contact. The lagrange multiplier associated with this
constraint is the normal reaction at the floor. This constraint equation is differentiated twice to obtain a matrix
equation of the form <br />
<br />
Cn * qddot = Gn <br />
<br />
The augmented lagrangian La is given by <br />
<br />
La = (KE1 + KE2 + KE3) − (PE1 + PE2 + PE3) + λx1 * jx12 + λy1 * jy12 + λx2 * jx23 + λy2 * jy23 + λ1 * l1 + λ3 * l3 <br />
<br />

These equations can be compactly written in matrix form as <br />
<br />
M * qddot − Cq * λq − Cf * λf − Cn * λn = ϕ <br />
<br />
where M is the mass matrix and λq ,λf and λn are the vectors of lagrange multipliers corresponding to Cq, Cf
and Cn respectively.
Once the lagrange multipliers are found, the accelerations can simply be obtained by the equation<br />
<br />
qddot = inv(M) * (Cq * λq + Cf * λf + Cn * λn + ϕ) <br />
<br />
To solve the ODE in MATLAB, the expressions for the mass matrix M, ϕ, Cq, q etc. obtained from Mathematica,
are transferred to a MATLAB program. In the program, these expressions are used in conjunction with a FDM
such as RK-4.



