package org.firstinspires.ftc.teamcode.kalman_filter;

public class KalmanFilter {
    private double Q; // Process noise covariance
    private double R; // Measurement noise covariance
    private double p = 1; // Estimate covariance
    private double x = 0; // State estimate
    private double k = 0; // Kalman gain

    public KalmanFilter(double Q, double R) {
        this.Q = Q;
        this.R = R;
    }

    public void predict(double delta) {
        x += delta;
        p += Q;
    }

    public void update(double measurement) {
        k = p / (p + R);
        x = x + k * (measurement - x);
        p = (1 - k) * p;
    }

    public void setState(double state) {
        this.x = state;
    }

    public double getState() {
        return x;
    }
}
