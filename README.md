# BLDC_Motor_Model – Drone Motor Performance & Loss Estimation

## 1. Overview

This project implements a simple analytical model for a BLDC / PM DC motor typically used in multirotor UAVs.  
The goal is to:

- Generate a **torque–speed curve** at a given DC bus voltage.
- Estimate **current, copper losses, and core/mechanical losses** vs speed.
- Compute **efficiency vs speed**.
- Estimate **steady-state motor case temperature** using a lumped thermal resistance model.

The model is not meant to be a production-accurate motor design tool.  
It is a fast way to understand how electrical parameters (KV, resistance, voltage) affect:

- Available torque
- Operating losses
- Thermal limits

---

## 2. Motor Parameters

The motor is chosen to roughly match a 2212-class multirotor motor:

- **Speed constant (KV)**: 920 rpm/V  
- **Bus voltage (V\_BUS)**: 14.8 V (4S LiPo)  
- **Phase resistance (R\_PHASE)**: 0.200 Ω  
- **No-load current (I\_NOLOAD)**: 0.8 A  
- **Core + mechanical loss at base speed**: 3 W  

Thermal model (lumped):

- **Thermal resistance (R\_th, case–ambient)**: 0.10 °C/W  
- **Ambient temperature**: 27 °C  

From the model:

- **No-load speed**: 13,616 rpm  
- **Max theoretical loss (at stall)**: 1,095.2 W  
- **Estimated ΔT at that loss**: 164.3 °C  
- **Estimated case temperature at that loss**: 191.3 °C  

> Note: The “max loss” point corresponds to **stall (0 rpm)** and is **not a realistic continuous operating point**.  
> Real UAV motors operate well away from stall; typical continuous losses are in the tens of watts range, not kilowatts.

---

## 3. Modeling Approach

### 3.1 Electrical & Mechanical Relations

1. Convert KV to back-EMF constant \(k_e\):

\[
k_e = \frac{1}{KV \cdot \frac{2\pi}{60}} \quad [\text{V/(rad/s)}]
\]

2. Torque constant \(k_t\) in SI units:

\[
k_t = k_e \quad [\text{Nm/A}]
\]

3. No-load speed (ideal):

\[
\omega_{0} = \frac{V_{bus}}{k_e} \quad [\text{rad/s}]
\]
\[
n_{0} = \omega_0 \cdot \frac{60}{2\pi} \quad [\text{rpm}]
\]

4. For each speed point:

- Back-EMF:
\[
E = k_e \, \omega
\]

- Phase current:
\[
I = \max\left( \frac{V_{bus} - E}{R_{phase}}, 0 \right)
\]

- Torque:
\[
T = k_t \, I
\]

- Mechanical output power:
\[
P_{mech} = T \, \omega
\]

- Electrical input power (approx):
\[
P_{in} = V_{bus} \cdot I
\]

---

### 3.2 Loss Modeling

- **Copper loss**:
\[
P_{cu} = I^2 R_{phase}
\]

- **Core + mechanical loss** (very crude):
\[
P_{core} = P_{core,base} \left( \frac{n}{n_{base}} \right)^2
\]

- **Total loss**:
\[
P_{loss} = P_{cu} + P_{core}
\]

- **Efficiency**:
\[
\eta = \frac{P_{mech}}{P_{in}}
\]

---

### 3.3 Thermal Model (Lumped)

Using a simple case-to-ambient thermal resistance:

\[
\Delta T = P_{loss,max} \cdot R_{th}
\]
\[
T_{case} = T_{ambient} + \Delta T
\]

Here:

- \(P_{loss,max} = 1{,}095.2\text{ W}\) (theoretical, at stall).
- \(R_{th} = 0.10 \,^\circ\text{C/W}\).
- \(\Delta T \approx 164.3^\circ\text{C}\).
- \(T_{case} \approx 191.3^\circ\text{C}\).

---

## 4. Generated Plots

The script saves the following plots to `plots/`:

1. **torque_speed.png**  
   - Torque vs speed (rpm).  
   - Shows the typical linear drop in torque as speed approaches no-load.

2. **efficiency_speed.png**  
   - Efficiency vs speed.  
   - Peaks in the mid-speed range, drops near stall and no-load.

3. **losses_speed.png**  
   - Copper loss, core loss, and total loss vs speed.  
   - Highlights that copper loss dominates at high torque/low speed, while core/mech loss grows with speed.

4. **current_speed.png**  
   - Phase current vs speed.  
   - Shows high current at low speed (including stall), dropping as speed approaches no-load.

---

## 5. Engineering Interpretation

- The model correctly predicts:
  - High stall current and thus very high **theoretical** copper loss at 0 rpm.
  - A mid-speed region where **efficiency is maximized** and losses are relatively modest.
- In real UAV operation:
  - The motor never runs at stall for more than milliseconds.
  - Continuous operation is mostly in the **30–80% of no-load speed** range.
  - In this region, losses are on the order of **tens of watts**, which is consistent with real hardware.

This project is mainly used to:

- Understand how **KV, bus voltage, and resistance** interact.
- Visualize how **electrical parameters limit torque and thermal performance**.
- Provide a quick way to reason about **safe operating points** for drone/EV-style BLDCs.

---

## 6. Limitations & Future Work

Limitations:

- Ignores detailed inverter and PWM effects.
- Uses very simple speed-squared core loss model.
- Thermal model is 1-node (single R\_th, no internal gradients).
- Assumes single lumped phase resistance.

Potential extensions:

- Incorporate more accurate core loss model (Steinmetz).
- Add inverter efficiency and DC link ripple.
- Couple this 1D model with a 3D FEA/CFD thermal simulation.
- Add operating-point filtering (hover load vs stall) to report realistic continuous loss ranges.

---

## 7. How to Run

```bash
python bldc_model.py
