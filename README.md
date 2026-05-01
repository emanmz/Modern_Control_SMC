# Biped Robot SMC Simulation Suite

A MATLAB simulation framework for comparing control strategies on robotic systems — from a 2-link manipulator arm to a full 5-link biped walker. Implements and benchmarks **Computed Torque (CT)**, **Sliding Mode Control (SMC)**, and a novel **Bio-Inspired CPG-SMC** hybrid controller.

---

## 📁 File Overview

| File | Description |
|---|---|
| `robot_dynamics.m` | Core dynamics function for a 2-link planar robot arm. Supports Classic SMC and Smooth SMC. |
| `Sim_Script.m` | Simulation script for the 2-link arm. Generates trajectory tracking GIFs and phase portrait comparisons. |
| `biped_smc.m` | *(Supporting module)* Biped-specific SMC logic and helper utilities. |
| `bio.m` | Main comparison suite for the 5-link Tzafestas biped. Runs CT, SMC, and CPG-SMC; produces analysis plots, performance tables, and an animated GIF. |

---

## Controllers Implemented

### 1. Computed Torque (CT)
Classic model-based feedforward + PD feedback controller. Assumes perfect knowledge of robot dynamics.

```
τ = M(q)(q̈d - Kd·ė - Kp·e) + C(q,q̇)q̇ + G(q)
```

### 2. Sliding Mode Control (SMC)
Robust nonlinear controller that drives the system onto a sliding surface `s = ė + λe`. Two variants:

- **Classic SMC** — uses `sign(s)`, produces chattering
- **Smooth SMC** — uses `tanh(s/φ)` or saturation, reduces chattering

```
τ = M(q)(q̈d - λė) + C(q,q̇)q̇ + G(q) - K·sign(s)
```

### 3. Bio-Inspired CPG-SMC (Hybrid)
Integrates a **Central Pattern Generator (CPG)** oscillator network with SMC. The CPG generates rhythmic reference modulation mimicking biological motor neurons, and its output adapts the SMC gain in real time.

```
du/dt = (1/τ)(-u - β·v + ω₀ + feedback)
dv/dt = (1/T)(-v + max(0, u))
q_ref = 0.15·max(0,u) + q_desired
```

---

## 🔬 System Models

### 2-Link Planar Robot Arm (`robot_dynamics.m`, `Sim_Script.m`)
- 2 revolute joints, link lengths `l1 = l2 = 0.5 m`
- Masses: `m1 = 1.0 kg`, `m2 = 0.8 kg`
- Desired trajectory: `qd = [sin(t); cos(t)]`
- External disturbance: `τ_dist = [2sin(5t); 2cos(5t)]`

### 5-Link Tzafestas Biped (`robot_dynamics.m`,`biped_smc.m`,`bio.m`)
- Stance shank → stance thigh → torso → swing thigh → swing shank
- Masses: `[3, 6, 20, 6, 3] kg`; Lengths: `[0.4, 0.4, 0.5, 0.4, 0.4] m`
- Hybrid dynamical system with heel-strike impact map
- Sinusoidal reference gait with 5 DOF

---

## Getting Started

**Requirements:** MATLAB R2019b or later (no additional toolboxes required)

### Run the 2-link arm comparison:
```matlab
% In MATLAB, run:
Sim_Script
```
Outputs:
- `SMC_Comparison.gif` — Joint 1 tracking + phase portrait animation
- `robot_motion.gif` — Real robot arm motion (SMC)
- `torque_vs_smc.gif` — Side-by-side CT vs SMC animation

### Run the full biped comparison:
```matlab
% In MATLAB, run:
bio
```
Outputs:
- `biped_comparison.gif` — Side-by-side CT / SMC / CPG-SMC animation
- `fig_cpg_phase.png` — CPG oscillator phase portrait
- `fig_cpg_smc_interaction.png` — Neural modulation of joint motion
- `fig_gait_trajectory.png` — Hip trajectory smoothness comparison
- `fig_entrainment.png` — CPG-joint phase synchronisation
- `fig_control_comparison.png` — Torque effort comparison

---

## 📊 Performance Metrics

The suite automatically prints a comparison table to the console:

```
Metric          CT         SMC        CPG-SMC
------------------------------------------------------------
Steps           XX         XX         XX
RMS Err         X.XXXX     X.XXXX     X.XXXX
Energy          XX.XX      XX.XX      XX.XX
```

- **Steps** — Number of successful gait cycles completed
- **RMS Error** — Root mean square joint angle tracking error
- **Energy** — Mean total mechanical energy (kinetic + potential)

---

## ⚙️ Tuning Parameters (`bio.m`)

| Parameter | Default | Description |
|---|---|---|
| `sim.Kp` | `80 × I₅` | Proportional gain (CT) |
| `sim.Kd` | `18 × I₅` | Derivative gain (CT) |
| `sim.Lambda` | `10 × I₅` | Sliding surface slope |
| `sim.K_smc` | `25 × I₅` | SMC switching gain |
| `sim.phi` | `0.08` | Boundary layer thickness (chatter reduction) |
| `sim.cpg.tau` | `0.08` | CPG excitation time constant |
| `sim.cpg.beta` | `2.0` | CPG mutual inhibition gain |
| `sim.cpg.w0` | `1.2` | CPG tonic drive |
| `sim.steps` | `20` | Number of gait steps to simulate |

---

## 🏗️ Code Architecture

```
bio.m
├── runHybridSim()          — Hybrid ODE loop with impact detection
│   ├── bipedDynamics()     — Continuous dynamics + controller dispatch
│   │   ├── referenceGait() — Sinusoidal desired trajectory
│   │   └── robotDynamics() — M, C, G matrices
│   ├── heelStrikeEvent()   — ODE event for foot contact
│   └── impactMap()         — Velocity reset at heel strike
├── evaluatePerformance()   — Computes steps, RMS error, energy
├── printComparisonTable()  — Console output
├── plotAnalysis()          — Torque + phase portrait figures
├── plotBioInspiredFigures()— CPG-specific analysis figures
└── animateComparison()     — Side-by-side GIF export

robot_dynamics.m            — 2-link arm dynamics (standalone function)
Sim_Script.m                — 2-link arm driver + GIF generation
```

---

## 📖 Background

This project implements ideas from:
- Tzafestas et al. — 5-link biped walking model
- Slotine & Li — Applied Nonlinear Control (SMC theory)
- Ijspeert (2008) — Central Pattern Generators for locomotion control

The CPG-SMC hybrid is motivated by biological motor control: spinal cord CPGs generate rhythmic locomotion patterns while descending cortical signals modulate amplitude and phase. Here, the CPG output modulates both the reference trajectory and the SMC switching gain, resulting in a smoother, more adaptive gait.

---

