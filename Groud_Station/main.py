import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

def plot_experiment_data(filepath):
    """
    Reads a telemetry CSV file and plots:
    1. Target Angle vs Estimated Angle
    2. PWM signals of the two motors
    """
    try:
        df = pd.read_csv(filepath)
        df.columns = df.columns.str.strip()

        time = df['Tempo Exp (s)']
        target = df['Target (deg)']
        angle = df['Angolo (deg)']
        pwm1 = df['PWM1']
        pwm2 = df['PWM2']

    except FileNotFoundError:
        print(f"Error: File '{filepath}' not found.")
        return
    except KeyError as e:
        print(f"Error: Column {e} not found. Check the headers in the CSV file.")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # --- Subplot 1: Attitude Response (Angles) ---
    ax1.plot(time, target, 'r--', label='Target (Reference)', linewidth=2)
    ax1.plot(time, angle, 'b-', label='Estimated Angle', linewidth=1.5)
    ax1.axhspan(-1, 1, color='green', alpha=0.2)
    ax1.set_title('Closed-Loop System Response (1-DoF)', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Roll Angle [deg]', fontsize=12)
    ax1.grid(True, linestyle=':', alpha=0.7)
    ax1.legend(loc='upper right')
    ax1.set_xlim([time.min(), time.max()])

    # --- Subplot 2: Control Effort (PWM) ---
    ax2.plot(time, pwm1, 'g-', label='Motor 1 PWM', linewidth=1.5)
    ax2.plot(time, pwm2, 'm-', label='Motor 2 PWM', linewidth=1.5)

    ax2.axhline(y=1000, color='k', linestyle=':', alpha=0.5, label='Min (1000 $\\mu s$)')
    ax2.axhline(y=2000, color='k', linestyle=':', alpha=0.5, label='Max (2000 $\\mu s$)')
    ax2.axhline(y=1200, color='gray', linestyle='-.', alpha=0.5, label='Hover Base (1200 $\\mu s$)')

    ax2.set_title('Control Effort', fontsize=12, fontweight='bold')
    ax2.set_xlabel('Time [s]', fontsize=12)
    ax2.set_ylabel('Pulse Width [$\\mu s$]', fontsize=12)
    ax2.grid(True, linestyle=':', alpha=0.7)
    ax2.legend(loc='upper right')
    ax2.set_ylim([900, 2100])

    plt.tight_layout()
    plt.savefig('experimental_plot.pdf', format='pdf', bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    percorso_file = Path.home() / "Desktop" / "esperimenti_csv" / "PID_Sbagliato.csv"
    plot_experiment_data(percorso_file)