# Single position REDCHO Gazebo baseline

The implementation uses six independent offboard agent processes, a bidirectional six-node ring, control interval \(\Delta=0.01\) s and Gazebo physics interval 0.001 s. All vectors below are world-frame xyz unless stated otherwise. One position REDCHO bank has three states per axis, nine scalar internal consensus states per aircraft. A separate local SVF is a reference filter, not a second REDCHO bank.

## Local battery and reference generation

Initial charges are \([1,.95,.90,.65,.80,.75]\). The unchanged model computes

\[
V_{oc}(s)=3.45+.65s+.10s^2-.05s^3-.008\ln(s+.001),
\quad \chi=(V_{oc}-3)/(4.15-3),
\quad \nu(s)=.25+\frac{.40}{1+\exp[-6(\chi-.5)]}.
\]

The governor uses local charge through open-circuit voltage; terminal voltage separately limits actuation. Before 5 s the raw velocity input is zero. Afterwards, with \(\tau=t-5\),

\[
u_i^r=\nu(s_i)\frac{[\cos(.04\tau),\sin(.04\tau),.05\sin(.2\tau)]^T}
{\|[\cos(.04\tau),\sin(.04\tau),.05\sin(.2\tau)]\|}.
\]

At 17 s cf2 charge is set to .32 and its internal resistance multiplied by 1.5, once. The original computation order is retained: raw speed changes immediately with charge, while the voltage-dependent authority uses the last terminal-voltage update. After motor allocation,

\[
P=28\big(\sum_{m=1}^4 f_m\big)^{3/2}+.85,\quad I=P/\max(V,2.5),
\]
\[
s^+=\operatorname{clip}(s-\Delta I/(3600\cdot.050),.01,1),\quad
V_p^+=V_p+\Delta[-V_p/(.075\cdot450)+I/450],
\quad V^+=\max(2.5,V_{oc}(s^+)-V_p^+-IR_{int}).
\]

Each local third-order SVF, with \(\omega=2\), obeys

\[
\dot z_0=z_1,\quad\dot z_1=z_2,\quad
\dot z_2=\omega^3(u_i^r-z_0)-3\omega^2z_1-3\omega z_2.
\]

The existing explicit-midpoint SVF step is retained. XY displacement is accumulated with the trapezoidal rule
\(r_{xy,k}=r_{xy,k-1}+\Delta(z_{0,xy,k-1}+z_{0,xy,k})/2\).
Its supplied derivative chain is \((r_{xy},z_{0,xy},z_{1,xy})\), with third derivative \(z_{2,xy}\). This is still local velocity integration; removing the velocity REDCHO does not remove integration.

For height, \(T=1.8\), \(\sigma=\operatorname{clip}((t-3)/T,0,1)\), \(\phi=10\sigma^3-15\sigma^4+6\sigma^5\), and
\(r_{z,i}=(h_i-.0175)\phi\). The proposals \([.78,.82,.85,.75,.80,.80]\) m average .8 m. Analytic first/second/third derivatives of this quintic are supplied. Initial world positions are \((-3.5,-5.5+i,.0175)\), \(i=0,\ldots,5\).

## Exactly one position REDCHO

For each axis define \(z_i=[r_i,\dot r_i,\ddot r_i]^T\),

\[
\Gamma=\begin{bmatrix}-.15&1&0\\0&-.15&1\\0&0&-.15\end{bmatrix},\quad C=[1,0,0],\quad
G=\begin{bmatrix}C\\C\Gamma\\C\Gamma^2\end{bmatrix},\quad y_i=z_i-Gx_i.
\]

With \(\mu=0,1,2\), \(\alpha_\mu=(2-\mu)/3\), \(\beta_\mu=(\mu+1)/3\), gains \(k=[2,1.4,.8]\), \(\theta=1.2\), \(L=32\) and \(\epsilon=.04\),

\[
\dot x_{i,\mu}=-.15x_{i,\mu}+\mathbf1_{\mu<2}x_{i,\mu+1}
+k_\mu\theta^{\mu+1}L^{\beta_\mu}\sum_{j\in\mathcal N_i}
e_{ij}(e_{ij}^2+\epsilon^2)^{(\alpha_\mu-1)/2},\quad e_{ij}=y_{i,0}-y_{j,0}.
\]

This update is componentwise in xyz, using explicit Euler and ascending neighbor-ID summation, exactly as the shared kernel. Each agent receives only two same-cycle **position** estimates \(y_{j,0}\). The controller uses its own \(y_{i,0},y_{i,1},y_{i,2}\) as displacement, velocity and acceleration estimates. No other consensus state is computed. These are observer derivative outputs, not an additional differentiated velocity bank. With smoothing/sampling, exact finite-time convergence or exact equality of finite differences is not claimed.

The retained regularity budget checks

\[
\rho_i=r_i^{(3)}+.45\ddot r_i+.0675\dot r_i+.003375r_i,
\quad\|r_i\|\le40,\quad\|\rho_i\|\le L/2.
\]

`regularity_certificate()` bounds the continuous reference generator using SVF impulse-kernel L1 norms and quintic derivative bounds. Its disagreement bound is about 19.3245, below selected \(L=32\). This certifies the stated bounded reference-input class, not sampled closed-loop convergence or recursive QP feasibility.

## Formation and nominal acceleration

Let \(p_i^*=p_{i,0}+y_{i,0}\), \(v_i^*=y_{i,1}\), \(a_i^*=y_{i,2}\), \(e_i=p_i^*-p_i\). The original startup law is

\[a_i^{nom}=a_i^*+6e_i+4(v_i^*-v_i),\qquad t<5.\]

For forward flight, define \(c=\bar c\tanh(K_pe_i/\bar c)\), with \(K_p=[2.4,2.4,2.4]\), \(\bar c=[.8,.8,1]\), componentwise. A two-second quintic activation \(\eta\) starts at 5 s:

\[
a_i^{nom}=a_i^*-4.5(v_i-v_i^*)-.45\tanh(v_i-v_i^*)+[\eta c_x,\eta c_y,c_z]^T.
\]

The only architectural change is that **all three axes** obtain \(v_i^*,a_i^*\) from the position bank. The original controller used velocity-bank outputs for XY after takeoff. Feedback gains, activation and QP objectives are preserved.

## Unchanged HOCBF-QP and motor plant

Each agent receives all-six physical states, previous safe accelerations and same-cycle locally computed acceleration bounds. That safety context is separate from the two-peer estimator ring. With \(h_{ij}=\|p_i-p_j\|^2-.82^2\), \(\dot h=2p_{ij}^Tv_{ij}\), the reciprocal joint RHS is

\[
b_{ij}=2\|v_{ij}\|^2+6\dot h+8.96h-2\|p_{ij}\|(.4+.4).
\]

The unchanged authority-weighted allocation splits the acceleration condition \(-2p_{ij}^T(a_i-a_j)\le b_{ij}\) between agents using their clipped previous-acceleration anchors. The obstacle model is two spheres at \((0,-3.5,.2)\), \((6,2,.2)\), radius .5 m; design surface clearance .75 m gives \(E=I/1.25^2\). With \(\delta=p_i-p_o\), \(h_o=\delta^TE\delta-1\),

\[
-2\delta^TEa_i\le2v_i^TEv_i+2\dot h_o+.96h_o-2\|E\delta\|(.4).
\]

The objective is \(\frac12\|a-a^{nom}\|^2+\frac{2}{2}\|a-a^{prev}\|^2\), subject to all barrier rows, voltage-derived componentwise authority and \(|a-a^{prev}|\le8\Delta\). Physical clearance checks remain .7 m pair distance and .5 m obstacle surface distance. Design buffers remain .12 m and .25 m. Phase-I is diagnostic feasibility recovery; commands with positive slack/violation above the original strict tolerance are rejected. Tracking is an objective, not a hard equality constraint.

Terminal voltage limits each motor by \(.16\operatorname{clip}(V/4.15,.5,1)^{1.5}\) N. The original thrust/tilt acceleration envelope, geometric attitude controller and four-motor allocator remain. Mass is .0299 kg, inertias \((1.395,1.395,2.173)10^{-5}\) kg m², arm diagonal .03973 m and thrust/drag coefficients are unchanged. Gazebo receives **motor forces**, not requested poses or accelerations. The backend applies ten 1 ms motor-wrench steps, includes quadratic translational drag and returns physical states. Body-frame odometry twist is rotated into world-frame velocity before control. Consecutive free-flight acceleration residual must stay within .4 m/s²; floor contact is evaluated separately as in the original experiment.

## Timing, telemetry and limits

One 1 ms zero-force bootstrap establishes the native state and epoch. Each cycle prepares all local references/authorities, exchanges the one-bank ring packets, distributes the authority batch, collects six complete commands and submits one atomic motor batch. Sequence, identity, shapes, finiteness, motor limits, applied force readback, state frames and exact 10 ms advances are checked. Missing data causes failure; it is not replaced with numerical plant states. A 45 s inclusive log has 4501 batches and 45.01 s plant advancement after bootstrap, matching the original convention.

The world remains paused between batches and at experiment end. Lockstep execution is not a 100 Hz wall-time deadline guarantee. The Thevenin battery and geometric controller are simulation models, not Crazyflie firmware/SITL. A numerical screening run is not native Gazebo validation; the validation report distinguishes them. The eight saved figures and CSV/NPZ histories contain real one-bank outputs; no velocity/Levant-bank ablation is fabricated.
