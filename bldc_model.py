#!/usr/bin/env python3
"""
BLDC Motor Performance & Loss Model

Simple analytical model for a BLDC / PM DC motor:
- Computes torque-speed curve at a given DC bus voltage.
- Computes current, copper losses, approximate core + mechanical losses.
- Estimates efficiency vs speed.
- Estimates steady-state temperature rise using a lumped thermal resistance.

This is for engineering insight, NOT production-accurate design.
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


# ============================
# USER-EDITABLE PARAMETERS
# ============================

# Motor electrical constants (example for a small drone/EV-like motor)
KV_RPM_PER_V = 920.0      # speed constant (rpm per volt)
R_PHASE = 0.2            # phase resistance [ohm] (line-line equivalent if using DC model)
V_BUS = 14.8              # DC bus voltage [V]
I_NOLOAD = 0.8            # no-load current [A] at V_BUS (approx. mech + core loss)

# Loss modelling constants
CORE_LOSS_W_AT_BASE = 3.0   # core+mech losses [W] at base_speed_rpm (crudely)
BASE_SPEED_RPM = KV_RPM_PER_V * V_BUS  # base = no-load speed at V_BUS

# Thermal model (lumped)
THERMAL_RESISTANCE_C_PER_W = 0.15   # R_th [°C/W] (motor case to ambient)
AMBIENT_TEMP_C = 27.0              # ambient temp [°C]

# Speed sweep
N_POINTS = 200   # points on the torque-speed curves


# ============================
# MODEL FUNCTIONS
# ============================


def ke_from_kv(kv_rpm_per_v: float) -> float:
    """
    Convert KV (rpm/V) to back-EMF constant ke in V/(rad/s).
    For an ideal DC motor model, ke = 1 / (KV * (2π/60))
    """
    return 1.0 / (kv_rpm_per_v * (2.0 * np.pi / 60.0))


def kt_from_ke(ke: float) -> float:
    """
    For SI units, torque constant kt [Nm/A] equals back-EMF constant ke [V/(rad/s)]
    in an ideal motor model.
    """
    return ke


def motor_model(
    kv_rpm_per_v: float,
    r_phase: float,
    v_bus: float,
    i_noload: float,
    core_loss_at_base: float,
    base_speed_rpm: float,
    n_points: int = 200,
):
    """
    Compute torque-speed curve and losses for a BLDC/DC motor-like model.
    Returns a dict of arrays.
    """
    ke = ke_from_kv(kv_rpm_per_v)
    kt = kt_from_ke(ke)

    # No-load mechanical speed (approx)
    omega_no_load = v_bus / ke          # rad/s
    speed_rpm_no_load = omega_no_load * 60.0 / (2.0 * np.pi)

    # Speed sweep from 0 to ~no-load
    speed_rpm = np.linspace(0.0, speed_rpm_no_load, n_points)
    omega = speed_rpm * 2.0 * np.pi / 60.0  # rad/s

    # Back-EMF at each speed
    e_back = ke * omega  # [V]

    # Simple DC model: I = (V - E) / R, but must not go negative
    i_phase = (v_bus - e_back) / r_phase
    i_phase = np.maximum(i_phase, 0.0)

    # Torque
    torque = kt * i_phase  # [Nm]

    # Electrical input power (approx)
    p_in = v_bus * i_phase  # [W]

    # Mechanical output power
    p_mech = torque * omega  # [W]

    # Copper losses
    p_cu = i_phase ** 2 * r_phase  # [W]

    # Core + mechanical losses: scale with (speed/base)^2
    # At base_speed_rpm we want ~core_loss_at_base
    speed_ratio = np.clip(speed_rpm / base_speed_rpm, 0.0, None)
    p_core = core_loss_at_base * (speed_ratio ** 2)

    # Total loss
    p_loss_total = p_cu + p_core

    # Efficiency
    with np.errstate(divide='ignore', invalid='ignore'):
        eta = np.where(p_in > 1e-6, p_mech / p_in, 0.0)
        eta = np.clip(eta, 0.0, 1.0)

    # Estimate steady-state temperature rise from *max* loss
    p_loss_max = np.max(p_loss_total)
    delta_t_max = p_loss_max * THERMAL_RESISTANCE_C_PER_W  # [°C]
    t_case_max = AMBIENT_TEMP_C + delta_t_max

    results = {
        "speed_rpm": speed_rpm,
        "omega": omega,
        "torque": torque,
        "current": i_phase,
        "p_in": p_in,
        "p_mech": p_mech,
        "p_cu": p_cu,
        "p_core": p_core,
        "p_loss_total": p_loss_total,
        "efficiency": eta,
        "speed_rpm_no_load": speed_rpm_no_load,
        "ke": ke,
        "kt": kt,
        "p_loss_max": p_loss_max,
        "delta_t_max": delta_t_max,
        "t_case_max": t_case_max,
    }

    return results


# ============================
# PLOTTING
# ============================


def ensure_plots_dir() -> Path:
    outdir = Path("plots").resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    return outdir


def plot_torque_speed(speed_rpm, torque, outdir: Path):
    fig, ax = plt.subplots()
    ax.plot(speed_rpm, torque)
    ax.set_xlabel("Speed [rpm]")
    ax.set_ylabel("Torque [Nm]")
    ax.set_title("Torque-Speed Curve")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(outdir / "torque_speed.png", dpi=200)
    plt.close(fig)


def plot_efficiency(speed_rpm, eta, outdir: Path):
    fig, ax = plt.subplots()
    ax.plot(speed_rpm, eta * 100.0)
    ax.set_xlabel("Speed [rpm]")
    ax.set_ylabel("Efficiency [%]")
    ax.set_title("Efficiency vs Speed")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(outdir / "efficiency_speed.png", dpi=200)
    plt.close(fig)


def plot_losses(speed_rpm, p_cu, p_core, p_loss_total, outdir: Path):
    fig, ax = plt.subplots()
    ax.plot(speed_rpm, p_cu, label="Copper loss (I²R)")
    ax.plot(speed_rpm, p_core, label="Core+mech loss (approx)")
    ax.plot(speed_rpm, p_loss_total, label="Total loss")
    ax.set_xlabel("Speed [rpm]")
    ax.set_ylabel("Power [W]")
    ax.set_title("Losses vs Speed")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(outdir / "losses_speed.png", dpi=200)
    plt.close(fig)


def plot_current_speed(speed_rpm, current, outdir: Path):
    fig, ax = plt.subplots()
    ax.plot(speed_rpm, current)
    ax.set_xlabel("Speed [rpm]")
    ax.set_ylabel("Current [A]")
    ax.set_title("Phase Current vs Speed")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(outdir / "current_speed.png", dpi=200)
    plt.close(fig)


# ============================
# MAIN
# ============================


def main():
    outdir = ensure_plots_dir()
    res = motor_model(
        KV_RPM_PER_V,
        R_PHASE,
        V_BUS,
        I_NOLOAD,
        CORE_LOSS_W_AT_BASE,
        BASE_SPEED_RPM,
        n_points=N_POINTS,
    )

    print("=== BLDC Motor Model Summary ===")
    print(f"KV            : {KV_RPM_PER_V:.2f} rpm/V")
    print(f"Bus Voltage   : {V_BUS:.2f} V")
    print(f"Phase R       : {R_PHASE:.4f} ohm")
    print(f"ke (V/(rad/s)): {res['ke']:.6f}")
    print(f"kt (Nm/A)     : {res['kt']:.6f}")
    print(f"No-load speed : {res['speed_rpm_no_load']:.1f} rpm")
    print(f"Max loss      : {res['p_loss_max']:.1f} W")
    print(f"ΔT_est        : {res['delta_t_max']:.1f} °C")
    print(f"T_case_est    : {res['t_case_max']:.1f} °C (steady-state)")

    # Generate plots
    plot_torque_speed(res["speed_rpm"], res["torque"], outdir)
    plot_efficiency(res["speed_rpm"], res["efficiency"], outdir)
    plot_losses(res["speed_rpm"], res["p_cu"], res["p_core"], res["p_loss_total"], outdir)
    plot_current_speed(res["speed_rpm"], res["current"], outdir)

    print(f"[DONE] Plots saved in: {outdir}")


if __name__ == "__main__":
    main()
