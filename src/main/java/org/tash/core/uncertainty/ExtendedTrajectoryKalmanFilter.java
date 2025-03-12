package org.tash.core.uncertainty;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.apache.commons.math3.linear.*;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;
import java.time.temporal.ChronoUnit;

/**
 * Extended Kalman filter for non-linear trajectory prediction
 * Uses Apache Commons Math3 for matrix operations
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class ExtendedTrajectoryKalmanFilter {
    /**
     * Kalman filter state dimension
     */
    private static final int STATE_DIM = 6; // lat, lon, alt, speed, track, vertical speed

    /**
     * Kalman filter measurement dimension
     */
    private static final int MEASUREMENT_DIM = 3; // lat, lon, alt

    /**
     * Current state vector
     */
    private RealVector stateVector;

    /**
     * Current error covariance matrix
     */
    private RealMatrix errorCovariance;

    /**
     * Last measurement time
     */
    private ZonedDateTime lastMeasurementTime;

    /**
     * Process noise covariance
     */
    private RealMatrix processNoiseCovariance;

    /**
     * Measurement noise covariance
     */
    private RealMatrix measurementNoiseCovariance;

    /**
     * Initialize the extended Kalman filter for trajectory prediction
     *
     * @param initialPosition   Initial position
     * @param speed             Initial speed in knots
     * @param track             Initial track angle in degrees (true)
     * @param verticalSpeed     Initial vertical speed in feet per minute
     * @param latLonUncertainty Initial lat/lon uncertainty in degrees
     * @param altUncertainty    Initial altitude uncertainty in feet
     * @param speedUncertainty  Initial speed uncertainty in knots
     * @param trackUncertainty  Initial track uncertainty in degrees
     * @param vsUncertainty     Initial vertical speed uncertainty in feet per minute
     * @param measurementNoise  Measurement noise (lat/lon in degrees, alt in feet)
     * @param time              Initial time
     * @return Extended Kalman filter for trajectory prediction
     */
    public static ExtendedTrajectoryKalmanFilter initialize(
            GeoCoordinate initialPosition,
            double speed,
            double track,
            double verticalSpeed,
            double latLonUncertainty,
            double altUncertainty,
            double speedUncertainty,
            double trackUncertainty,
            double vsUncertainty,
            double[] measurementNoise2,
            ZonedDateTime time) {

        // Initial state vector
        RealVector initialState = new ArrayRealVector(STATE_DIM);
        initialState.setEntry(0, initialPosition.getLatitude());   // Latitude (degrees)
        initialState.setEntry(1, initialPosition.getLongitude());  // Longitude (degrees)
        initialState.setEntry(2, initialPosition.getAltitude());   // Altitude (feet)
        initialState.setEntry(3, speed);                           // Speed (knots)
        initialState.setEntry(4, track);                           // Track (degrees true)
        initialState.setEntry(5, verticalSpeed);                   // Vertical speed (feet per minute)

        // Initial error covariance matrix
        RealMatrix initialCovariance = new Array2DRowRealMatrix(STATE_DIM, STATE_DIM);
        initialCovariance.setEntry(0, 0, Math.pow(latLonUncertainty, 2)); // Latitude variance
        initialCovariance.setEntry(1, 1, Math.pow(latLonUncertainty, 2)); // Longitude variance
        initialCovariance.setEntry(2, 2, Math.pow(altUncertainty, 2));    // Altitude variance
        initialCovariance.setEntry(3, 3, Math.pow(speedUncertainty, 2));  // Speed variance
        initialCovariance.setEntry(4, 4, Math.pow(trackUncertainty, 2));  // Track variance
        initialCovariance.setEntry(5, 5, Math.pow(vsUncertainty, 2));     // Vertical speed variance

        // Process noise covariance
        RealMatrix processNoise = new Array2DRowRealMatrix(STATE_DIM, STATE_DIM);
        processNoise.setEntry(0, 0, Math.pow(latLonUncertainty, 2) * 0.1); // Latitude process noise
        processNoise.setEntry(1, 1, Math.pow(latLonUncertainty, 2) * 0.1); // Longitude process noise
        processNoise.setEntry(2, 2, Math.pow(altUncertainty, 2) * 0.1);    // Altitude process noise
        processNoise.setEntry(3, 3, Math.pow(speedUncertainty, 2) * 0.1);  // Speed process noise
        processNoise.setEntry(4, 4, Math.pow(trackUncertainty, 2) * 0.1);  // Track process noise
        processNoise.setEntry(5, 5, Math.pow(vsUncertainty, 2) * 0.1);     // Vertical speed process noise

        // Measurement noise covariance
        RealMatrix measurementNoise = new Array2DRowRealMatrix(MEASUREMENT_DIM, MEASUREMENT_DIM);
        measurementNoise.setEntry(0, 0, Math.pow(measurementNoise2[0], 2)); // Latitude measurement noise
        measurementNoise.setEntry(1, 1, Math.pow(measurementNoise2[1], 2)); // Longitude measurement noise
        measurementNoise.setEntry(2, 2, Math.pow(measurementNoise2[2], 2)); // Altitude measurement noise

        return ExtendedTrajectoryKalmanFilter.builder()
                .stateVector(initialState)
                .errorCovariance(initialCovariance)
                .lastMeasurementTime(time)
                .processNoiseCovariance(processNoise)
                .measurementNoiseCovariance(measurementNoise)
                .build();
    }

    /**
     * Update the filter with a new position measurement
     *
     * @param position New position measurement
     * @param time     Measurement time
     */
    public void update(GeoCoordinate position, ZonedDateTime time) {
        // Calculate time delta
        long deltaMillis = ChronoUnit.MILLIS.between(lastMeasurementTime, time);
        double dt = deltaMillis / 1000.0; // Convert to seconds

        // Scale process noise by time delta
        RealMatrix scaledProcessNoise = processNoiseCovariance.scalarMultiply(dt);

        // Prediction step
        predict(dt, scaledProcessNoise);

        // Measurement vector
        RealVector z = new ArrayRealVector(MEASUREMENT_DIM);
        z.setEntry(0, position.getLatitude());
        z.setEntry(1, position.getLongitude());
        z.setEntry(2, position.getAltitude());

        // Calculate Jacobian of measurement function
        RealMatrix H = calculateMeasurementJacobian();

        // Predicted measurement
        RealVector predictedZ = measurementFunction(stateVector);

        // Innovation: y = z - h(x)
        RealVector innovation = z.subtract(predictedZ);

        // Innovation covariance: S = H*P*H^T + R
        RealMatrix S = H.multiply(errorCovariance).multiply(H.transpose()).add(measurementNoiseCovariance);

        // Kalman gain: K = P*H^T*S^-1
        RealMatrix K = errorCovariance.multiply(H.transpose()).multiply(MatrixUtils.inverse(S));

        // Update state: x = x + K*y
        stateVector = stateVector.add(K.operate(innovation));

        // Update error covariance: P = (I - K*H)*P
        RealMatrix I = MatrixUtils.createRealIdentityMatrix(STATE_DIM);
        errorCovariance = I.subtract(K.multiply(H)).multiply(errorCovariance);

        // Update last measurement time
        lastMeasurementTime = time;
    }

    /**
     * Predict state using extended Kalman filter
     *
     * @param dt           Time delta in seconds
     * @param processNoise Process noise covariance matrix
     */
    private void predict(double dt, RealMatrix processNoise) {
        // Calculate Jacobian of state transition function
        RealMatrix F = calculateStateTransitionJacobian(dt);

        // Predict state: x = f(x)
        stateVector = stateTransitionFunction(stateVector, dt);

        // Predict error covariance: P = F*P*F^T + Q
        errorCovariance = F.multiply(errorCovariance).multiply(F.transpose()).add(processNoise);
    }

    /**
     * State transition function (non-linear)
     *
     * @param x  State vector
     * @param dt Time delta in seconds
     * @return Updated state vector
     */
    private RealVector stateTransitionFunction(RealVector x, double dt) {
        double lat = x.getEntry(0);
        double lon = x.getEntry(1);
        double alt = x.getEntry(2);
        double speed = x.getEntry(3);
        double track = x.getEntry(4);
        double vs = x.getEntry(5);

        // Convert from knots to degrees/second (approximate)
        double latChange = speed * Math.cos(Math.toRadians(track)) * dt / 60.0;
        double lonChange = speed * Math.sin(Math.toRadians(track)) * dt / (60.0 * Math.cos(Math.toRadians(lat)));

        // Convert from feet/minute to feet
        double altChange = vs * dt / 60.0;

        RealVector newState = new ArrayRealVector(STATE_DIM);
        newState.setEntry(0, lat + latChange);
        newState.setEntry(1, lon + lonChange);
        newState.setEntry(2, alt + altChange);
        newState.setEntry(3, speed);
        newState.setEntry(4, track);
        newState.setEntry(5, vs);

        return newState;
    }

    /**
     * Calculate Jacobian of state transition function
     *
     * @param dt Time delta in seconds
     * @return Jacobian matrix
     */
    private RealMatrix calculateStateTransitionJacobian(double dt) {
        double lat = stateVector.getEntry(0);
        double speed = stateVector.getEntry(3);
        double track = stateVector.getEntry(4);

        // Partial derivatives
        double dLat_dLat = 1.0;
        double dLat_dSpeed = Math.cos(Math.toRadians(track)) * dt / 60.0;
        double dLat_dTrack = -speed * Math.sin(Math.toRadians(track)) * Math.PI / 180 * dt / 60.0;

        double dLon_dLat = -speed * Math.sin(Math.toRadians(track)) * dt * Math.sin(Math.toRadians(lat)) /
                (60.0 * Math.pow(Math.cos(Math.toRadians(lat)), 2));
        double dLon_dLon = 1.0;
        double dLon_dSpeed = Math.sin(Math.toRadians(track)) * dt / (60.0 * Math.cos(Math.toRadians(lat)));
        double dLon_dTrack = speed * Math.cos(Math.toRadians(track)) * Math.PI / 180 * dt /
                (60.0 * Math.cos(Math.toRadians(lat)));

        double dAlt_dAlt = 1.0;
        double dAlt_dVs = dt / 60.0;

        // Create Jacobian matrix
        RealMatrix F = MatrixUtils.createRealIdentityMatrix(STATE_DIM);

        // Latitude derivatives
        F.setEntry(0, 0, dLat_dLat);
        F.setEntry(0, 3, dLat_dSpeed);
        F.setEntry(0, 4, dLat_dTrack);

        // Longitude derivatives
        F.setEntry(1, 0, dLon_dLat);
        F.setEntry(1, 1, dLon_dLon);
        F.setEntry(1, 3, dLon_dSpeed);
        F.setEntry(1, 4, dLon_dTrack);

        // Altitude derivatives
        F.setEntry(2, 2, dAlt_dAlt);
        F.setEntry(2, 5, dAlt_dVs);

        return F;
    }

    /**
     * Measurement function (non-linear)
     *
     * @param x State vector
     * @return Measurement vector
     */
    private RealVector measurementFunction(RealVector x) {
        RealVector z = new ArrayRealVector(MEASUREMENT_DIM);
        z.setEntry(0, x.getEntry(0)); // Latitude
        z.setEntry(1, x.getEntry(1)); // Longitude
        // Add altitude measurement if available (not included in this example)
        // z.setEntry(2, x.getEntry(2)); // Altitude
        return z;
    }

    /**
     * Calculate Jacobian of measurement function
     *
     * @return Jacobian matrix
     */
    private RealMatrix calculateMeasurementJacobian() {
        RealMatrix H = MatrixUtils.createRealIdentityMatrix(MEASUREMENT_DIM);
        H.setEntry(0, 0, 1.0); // Latitude derivative
        H.setEntry(1, 1, 1.0); // Longitude derivative
        // Add altitude derivative if available (not included in this example)
        // H.setEntry(2, 2, 1.0); // Altitude derivative
        return H;
    }
}
