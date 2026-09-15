# Multi-Agent Drone Swarm — Safety-Critical Control

A 2D "drone show" simulator: give it any image, and a swarm of planar
quadrotors — one per lit pixel — flies from a scrambled launch grid into that
picture, while a centralized **Control Barrier Function (CBF)** safety filter
provably keeps every pair of drones collision-free the whole way there. Built
for Caltech's CDS 233 (Safety-Critical Control).

<p align="center">
  <img src="gifs/heart.gif" width="380"/>
  <img src="gifs/bike.gif" width="380"/>
</p>
<p align="center">
  <img src="gifs/flamingo.gif" width="380"/>
  <img src="gifs/flappy_bird.gif" width="380"/>
</p>

Same controller, same safety filter, different target image each time.

## How it works ([`drone_show_sprite.py`](drone_show_sprite.py))

```
┌───────┐
│ image │
└───┬───┘
    │
    ▼
┌───────────────┐
│ pixel targets │
└───────┬───────┘
        │
        ▼
┌─────────────┐
│ LQR nominal │
└──────┬──────┘
       │
       ▼
┌──────────────────────┐
│ CBF-QP safety filter │
└───────────┬──────────┘
            │
            ▼
┌─────────────────────┐
│ flatness + attitude │
└──────────┬──────────┘
           │
           ▼
┌───────────────┐
│ rotor thrusts │
└───────────────┘
```

**1. Image → targets.** `load_sprite` reads a PNG/GIF and turns every opaque,
non-background pixel into one drone's target `(x, y)` (`pixels_to_targets`).
One lit pixel = one quadrotor — a 25×25 sprite means 625 agents, each with a
pairwise safety constraint against every other.

**2. Launch assignment via the Hungarian algorithm.** Rather than randomly
scrambling starting positions, `make_start_positions` lays out a launch grid
below the target image and solves an assignment problem — the cost matrix is
squared distance from every launch slot to every target — with
`scipy.optimize.linear_sum_assignment`. This minimizes total swarm travel
distance and cuts down on avoidable path crossings before the safety filter
even has to intervene.

**3. Nominal controller — LQR.** Each drone is treated, for planning
purposes, as a 2D double integrator ($\ddot p = a$). The output
$y = p - p_{des}$ has *relative degree 2*, so tracking is posed on the
extended state $\eta = [p - p_{des},\, v]$, with dynamics $\dot\eta = A\eta + Ba$
($A=\begin{bmatrix}0&1\\0&0\end{bmatrix}$, $B=\begin{bmatrix}0\\1\end{bmatrix}$,
decoupled per axis). Solving the CARE $A^\top P + PA - PBR^{-1}B^\top P + Q=0$
once (`_build_lqr`) gives the infinite-horizon-optimal gain
$K = R^{-1}B^\top P$, and every timestep each drone's nominal acceleration is
simply $a = -K\eta$ — pulling it toward its target with critically-damped,
optimal-in-the-LQR-sense gains.

**4. Safety filter — centralized min-norm CBF-QP.** Collision avoidance is one
pairwise barrier per drone pair:

$$h_{ij}(x) = \lVert p_i - p_j \rVert^2 - D_s^2 \ge 0$$

Since control enters through acceleration, $h_{ij}$ has relative degree 2, so
a first-derivative CBF condition doesn't apply — instead the filter enforces
the **exponential / high-order CBF** condition (poles at $-\alpha_1,-\gamma$):

$$\ddot h_{ij} + (\alpha_1+\gamma)\dot h_{ij} + \alpha_1\gamma\, h_{ij} \ge 0$$

which, writing $\Delta p = p_i-p_j,\ \Delta v = v_i-v_j$, expands to a
constraint affine in the two drones' accelerations:

$$2\Delta p\cdot a_i - 2\Delta p\cdot a_j \ \ge\ -2\lVert\Delta v\rVert^2 -2(\alpha_1+\gamma)\Delta p\cdot\Delta v -\alpha_1\gamma\left(\lVert\Delta p\rVert^2 - D_s^2\right)$$

Stacking one such constraint per pair gives the centralized **min-norm safety
filter**:

$$\min_{a_1,\dots,a_N} \sum_i \lVert a_i - a_i^{nom}\rVert^2 \quad\text{s.t. the above, for every pair } i<j$$

i.e. the least-restrictive correction to the LQR command that keeps every
pair provably safe — solved every timestep. An optional `epsilon` parameter
adds an **input-to-state safety (ISSf)** margin, tightening each constraint by
$\tfrac{1}{\epsilon}\lVert L_g h_e\rVert^2 = \tfrac{8}{\epsilon}\lVert\Delta p\rVert^2$
for robustness to model/actuation error.

A dense $O(N^2)$ QP quickly becomes intractable as $N$ grows into the
hundreds, so `SparseCBF` only generates a constraint for pairs within a
sensing radius `D_sense` (all others are provably inert and skipped), builds
the resulting sparse constraint matrix directly, and solves it with `osqp` —
scaling as $O(N\cdot\text{avg. neighbors})$ instead of $O(N^2)$.

**5. Acceleration → rotor thrusts: feedback linearization + differential
flatness.** `map_accel_to_thrusts` converts the safe 2D acceleration into the
two rotor thrusts $u_1,u_2$ in two steps. The desired total thrust and tilt
angle come from the quadrotor's differential flatness,
$T = m\sqrt{a_x^2+(a_y+g)^2}$, $\theta_d=\text{atan2}(-a_x,\ a_y+g)$ — exact
*if* $\theta=\theta_d$ instantaneously. Making that (approximately) true is
the job of the attitude loop: since $\ddot\theta = r(u_1-u_2)/I$ exactly, with
no other terms, commanding `torque = I * (kp_th*(theta_d-theta) - kd_th*theta_dot)`
and solving $u_1-u_2=\text{torque}/r$ is an **exact input-output feedback
linearization** of the rotational dynamics — it makes $\ddot\theta$ track the
PD virtual control exactly. The gains are set deliberately high
(`kp_th, kd_th = 100.0, 15.0`) precisely so this inner loop is fast enough to
make the outer flatness assumption ($\theta\approx\theta_d$) hold in practice.

**6. Early stopping & rendering.** The sim terminates as soon as every drone
is within `converge_pos_tol` of its target and nearly stopped (held for a few
steps to reject transients), instead of always running the full horizon.
`animate()` renders the swarm, optionally hiding each target's marker until
the drone actually arrives (`reveal_at_end`), and exports a GIF via
`matplotlib`'s Pillow writer.

## Running it

```bash
pip install numpy scipy matplotlib cvxpy osqp pillow
python drone_show_sprite.py
```

By default this loads `figs/flappy_bird.png` and flies the swarm into it;
edit `sprite_path` at the bottom of `drone_show_sprite.py` to point at any of
the other pre-rendered pixel-art images in [`figs/`](figs/).

## Acknowledgments

A team project for Caltech CDS 233 (Safety-Critical Control), built together
with [Deon Petrizzo](https://github.com/deonfpetrizzo) and Jawhara Emhemed.
