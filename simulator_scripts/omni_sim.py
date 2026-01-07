import numpy as np

# Robot and wheel parameters
WHEEL_DIAMETER_CM = 3.8  # 38mm
TICKS_PER_REV = 520
WHEEL_CIRCUMFERENCE = np.pi * WHEEL_DIAMETER_CM
WHEEL_ANGLES_DEG = [45, 135, 225, 315]  # X layout (degrees)


def distance_to_ticks_omni(distance_cm, move_angle_deg):
    """
    For a given move direction, calculate expected ticks per wheel using omni wheel kinematics.
    """
    move_angle_rad = np.deg2rad(move_angle_deg)
    ticks_needed = []
    for theta_deg in WHEEL_ANGLES_DEG:
        theta_rad = np.deg2rad(theta_deg)
        # Effective contribution for this wheel
        cos_term = np.cos(move_angle_rad - theta_rad)
        if np.isclose(cos_term, 0, atol=1e-4):
            print(f"Warning: Move angle {move_angle_deg}° is nearly perpendicular to wheel at {theta_deg}°, result may be undefined.")
            ticks_needed.append(float('inf'))
            continue
        eff_dist = distance_cm / abs(cos_term)
        ticks = eff_dist / WHEEL_CIRCUMFERENCE * TICKS_PER_REV
        ticks_needed.append(int(round(ticks)))
    return ticks_needed


def main():
    print("Omni Bot Kinematics Simulator (X layout, 4 wheels, with correct omni math)")
    while True:
        print("\nChoose input method:")
        print("  1. r, theta (distance and angle)")
        print("  2. coordinates (x, y) in cm")
        method = input("Enter 1 or 2: ").strip()
        if method == '1':
            try:
                distance = float(input("Enter distance to move (cm): "))
                angle = float(input("Enter movement angle (deg, 0=forward, 90=left): "))
            except ValueError:
                print("Invalid input. Try again.")
                continue
            print(f"\n[r, theta] Move {distance}cm at {angle}°:")
        elif method == '2':
            try:
                x = float(input("Enter X coordinate to move to (cm): "))
                y = float(input("Enter Y coordinate to move to (cm): "))
            except ValueError:
                print("Invalid input. Try again.")
                continue
            distance = np.hypot(x, y)
            angle = np.rad2deg(np.arctan2(y, x)) % 360
            print(f"\n[coordinates] Move to ({x}, {y}) cm:")
            print(f"  Calculated distance: {distance:.2f} cm")
            print(f"  Calculated angle: {angle:.2f}° (0=forward, 90=left)")
        else:
            print("Invalid selection. Try again.")
            continue
        ticks = distance_to_ticks_omni(distance, angle)
        for i, t in enumerate(ticks):
            if t == float('inf'):
                print(f"  Wheel {i+1} (at {WHEEL_ANGLES_DEG[i]}°): undefined (pure lateral for this wheel)")
            else:
                print(f"  Wheel {i+1} (at {WHEEL_ANGLES_DEG[i]}°): {t} ticks")
        print()
        again = input("Simulate another move? (y/n): ").strip().lower()
        if again != 'y':
            break

if __name__ == '__main__':
    main() 