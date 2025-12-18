import numpy as np


class PI_period_checker:
    def __init__(self):
        self.alpha = 0.1  # low-pass filter alpha
        self.period = 0.0
        self.heat_time = []
        self.count = 0
        self.peak_threshold = 0.1  # time diff threshold for same peak 판단

    def measure_critical_oscillation_period(self, p_gain, i_gain, desired_value):
        """
        Measure the oscillation period of the system from experimental data.

        Returns:
            float: Estimated oscillation period Tu.
        """
        filepath = "/home/acca/all/pid_test/p:{}, i:{}, v:{}.txt".format(
            p_gain, i_gain, desired_value
        )

        with open(filepath) as f:
            lines = f.readlines()
            # Parse time-value pairs: [ [t0, v0], [t1, v1], ... ]
            tv = [list(map(float, line.strip().split(", "))) for line in lines[:-1]]

        # Append is_peak flag
        tv_with_peaks = []
        for i in range(len(tv)):
            if i == 0 or i == len(tv) - 1:
                tv_with_peaks.append([tv[i][0], tv[i][1], False])
            else:
                peak = self.is_peak(tv, i)
                tv_with_peaks.append([tv[i][0], tv[i][1], peak])

        last_peak_time = 0.0
        for row in tv_with_peaks:
            t, v, is_peak = row
            if is_peak:
                if not self.heat_time:
                    self.heat_time.append(t)
                    last_peak_time = t
                else:
                    if self.same_peak(last_peak_time, t):
                        self.count += 1
                        self.heat_time.append(t)
                        last_peak_time = t
                    else:
                        # Reinitialize
                        self.heat_time = [t]
                        last_peak_time = t
                        self.count = 0

                if self.count == 50:
                    periods = [
                        self.low_pass_filter(self.heat_time[i + 1] - self.heat_time[i])
                        for i in range(len(self.heat_time) - 1)
                    ]
                    self.period = np.mean(periods)
                    print(f"[INFO] Estimated Tu (oscillation period): {self.period:.3f} sec")
                    return self.period

        print("[WARN] Oscillation not sustained or insufficient peak count.")
        return None

    def low_pass_filter(self, time_diff):
        """
        Apply exponential low-pass filter to smooth period estimation.
        """
        self.period = self.alpha * self.period + (1 - self.alpha) * time_diff
        return self.period

    def is_peak(self, tv, i):
        """
        Check if the i-th point is a local maximum.

        Parameters:
            tv (list): Time-value list.
            i (int): Index of current point.

        Returns:
            bool: True if peak.
        """
        return tv[i][1] > tv[i - 1][1] and tv[i][1] > tv[i + 1][1]

    def same_peak(self, a, b):
        """
        Determine whether two peaks are close enough in time to be considered same-type peaks.

        Parameters:
            a (float): Time of previous peak.
            b (float): Time of current peak.

        Returns:
            bool: True if within threshold.
        """
        return abs(a - b) > self.peak_threshold


def ziegler_nichols_pi_tuning(Ku, Tu):
    """
    Ziegler-Nichols PI tuning based on critical gain and period.

    Parameters:
        Ku (float): Ultimate gain (critical gain).
        Tu (float): Oscillation period at Ku.

    Returns:
        dict: {'Kp': float, 'Ki': float}
    """
    Kp = 0.45 * Ku
    Ki = 0.54 * Ku / Tu
    return {"Kp": Kp, "Ki": Ki}


def inverse_ziegler_nichols_pi_tuning(Kp, Ki):
    """
    Infer Ku, Tu from PI gains using inverse Z-N equations.

    Parameters:
        Kp (float): Proportional gain.
        Ki (float): Integral gain.

    Returns:
        dict: {'Ku': float, 'Tu': float}
    """
    Ku = Kp / 0.45
    Tu = 0.54 * Ku / Ki
    return {"Ku": Ku, "Tu": Tu}


# 예시 실행
if __name__ == "__main__":
    # 기존 PI 계수로부터 Ku, Tu 추정 (역방향)
    Kp = 2.07
    Ki = 0.85
    exp = inverse_ziegler_nichols_pi_tuning(Kp, Ki)
    print("Expected Ku and Tu (from gains):", exp)

    # 로그 파일 기반 Tu 측정
    pid_checker = PI_period_checker()
    measured_Tu = pid_checker.measure_critical_oscillation_period(Kp, Ki, 6.5)

    # 자동 튜닝 (Ku는 수동 지정 필요)
    if measured_Tu:
        Ku = exp["Ku"]  # 또는 실험으로 얻은 Ku
        new_gains = ziegler_nichols_pi_tuning(Ku, measured_Tu)
        print("Ziegler–Nichols tuned PI gains:", new_gains)
