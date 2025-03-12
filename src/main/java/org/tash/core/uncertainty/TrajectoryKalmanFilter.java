package org.tash.core.uncertainty;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import org.tash.data.GeoCoordinate;
import org.tash.data.Vector3D;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.tash.extensions.geodetic.ENUCoordinate;

import java.time.ZonedDateTime;
import java.time.temporal.ChronoUnit;
import java.util.ArrayList;
import java.util.List;

/**
 * Kalman filter for trajectory prediction with uncertainty
 * Uses Apache Commons Math3 KalmanFilter implementation
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class TrajectoryKalmanFilter {
    /** Kalman filter state dimension */
    private static final int STATE_DIM = 9; // 3 positions + 3 velocities + 3 accelerations

    /** Kalman filter measurement dimension */
    private static final int MEASUREMENT_DIM = 3; // 3 positions

    /** Kalman filter implementation */
    private KalmanFilter kalmanFilter;

    /** Current state vector */
    private RealVector stateVector;

    /** Current error covariance matrix */
    private RealMatrix errorCovariance;

    /** Previous measurement time */
    private ZonedDateTime lastMeasurementTime;

    /** Reference point for local ENU coordinates */
    private GeoCoordinate referencePoint;

    /** Process noise covariance elements */
    private double processNoisePosition;
    private double processNoiseVelocity;
    private double processNoiseAcceleration;

    /** Measurement noise covariance elements */
    private double measurementNoisePosition;

    /**
     * Initialize the Kalman filter with position and velocity
     *
     * @param initialPosition Initial position
     * @param initialVelocity Initial velocity in m/s
     * @param positionUncertainty Initial position uncertainty in meters
     * @param velocityUncertainty Initial velocity uncertainty in m/s
     * @param accelerationUncertainty Initial acceleration uncertainty in m/s²
     * @param measurementNoise Measurement noise in meters
     * @param time Initial time
     * @return Initialized Kalman filter
     */
    public static TrajectoryKalmanFilter initialize(
            GeoCoordinate initialPosition,
            Vector3D initialVelocity,
            double positionUncertainty,
            double velocityUncertainty,
            double accelerationUncertainty,
            double measurementNoise,
            ZonedDateTime time) {

        // Initial state vector (in ENU coordinates, relative to initial position as reference)
        RealVector initialState = new ArrayRealVector(STATE_DIM);
        initialState.setEntry(0, 0); // Initial x position (East) = 0
        initialState.setEntry(1, 0); // Initial y position (North) = 0
        initialState.setEntry(2, 0); // Initial z position (Up) = 0
        initialState.setEntry(3, initialVelocity.getX()); // x velocity (East)
        initialState.setEntry(4, initialVelocity.getY()); // y velocity (North)
        initialState.setEntry(5, initialVelocity.getZ()); // z velocity (Up)
        initialState.setEntry(6, 0); // x acceleration (East)
        initialState.setEntry(7, 0); // y acceleration (North)
        initialState.setEntry(8, 0); // z acceleration (Up)

        // Initial state covariance matrix
        RealMatrix initialCovariance = new Array2DRowRealMatrix(STATE_DIM, STATE_DIM);

        // Position error covariance
        initialCovariance.setEntry(0, 0, Math.pow(positionUncertainty, 2));
        initialCovariance.setEntry(1, 1, Math.pow(positionUncertainty, 2));
        initialCovariance.setEntry(2, 2, Math.pow(positionUncertainty, 2));

        // Velocity error covariance
        initialCovariance.setEntry(3, 3, Math.pow(velocityUncertainty, 2));
        initialCovariance.setEntry(4, 4, Math.pow(velocityUncertainty, 2));
        initialCovariance.setEntry(5, 5, Math.pow(velocityUncertainty, 2));

        // Acceleration error covariance
        initialCovariance.setEntry(6, 6, Math.pow(accelerationUncertainty, 2));
        initialCovariance.setEntry(7, 7, Math.pow(accelerationUncertainty, 2));
        initialCovariance.setEntry(8, 8, Math.pow(accelerationUncertainty, 2));

        // Initial state transition matrix (placeholder, will be updated before predictions)
        double dt = 1.0;
        RealMatrix stateTransition = createStateTransition(dt);

        // Process noise covariance
        RealMatrix processNoise = new Array2DRowRealMatrix(STATE_DIM, STATE_DIM);

        // Position noise
        processNoise.setEntry(0, 0, Math.pow(positionUncertainty, 2));
        processNoise.setEntry(1, 1, Math.pow(positionUncertainty, 2));
        processNoise.setEntry(2, 2, Math.pow(positionUncertainty, 2));

        // Velocity noise
        processNoise.setEntry(3, 3, Math.pow(velocityUncertainty, 2));
        processNoise.setEntry(4, 4, Math.pow(velocityUncertainty, 2));
        processNoise.setEntry(5, 5, Math.pow(velocityUncertainty, 2));

        // Acceleration noise
        processNoise.setEntry(6, 6, Math.pow(accelerationUncertainty, 2));
        processNoise.setEntry(7, 7, Math.pow(accelerationUncertainty, 2));
        processNoise.setEntry(8, 8, Math.pow(accelerationUncertainty, 2));

        // Control matrix (not used, set to zero)
        RealMatrix controlMatrix = new Array2DRowRealMatrix(STATE_DIM, 1);

        // Process model
        ProcessModel processModel = new DefaultProcessModel(
                stateTransition, controlMatrix, processNoise, initialState, initialCovariance);

        // Measurement matrix (we measure only position)
        RealMatrix measurementMatrix = new Array2DRowRealMatrix(MEASUREMENT_DIM, STATE_DIM);
        measurementMatrix.setEntry(0, 0, 1.0); // Measure x position
        measurementMatrix.setEntry(1, 1, 1.0); // Measure y position
        measurementMatrix.setEntry(2, 2, 1.0); // Measure z position

        // Measurement noise covariance
        RealMatrix measurementNoiseCov = new Array2DRowRealMatrix(MEASUREMENT_DIM, MEASUREMENT_DIM);
        measurementNoiseCov.setEntry(0, 0, Math.pow(measurementNoise, 2));
        measurementNoiseCov.setEntry(1, 1, Math.pow(measurementNoise, 2));
        measurementNoiseCov.setEntry(2, 2, Math.pow(measurementNoise, 2));

        // Measurement model
        MeasurementModel measurementModel = new DefaultMeasurementModel(
                measurementMatrix, measurementNoiseCov);

        // Create Kalman filter
        KalmanFilter kf = new KalmanFilter(processModel, measurementModel);

        return TrajectoryKalmanFilter.builder()
                .kalmanFilter(kf)
                .stateVector(initialState)
                .errorCovariance(initialCovariance)
                .lastMeasurementTime(time)
                .referencePoint(initialPosition)
                .processNoisePosition(Math.pow(positionUncertainty, 2))
                .processNoiseVelocity(Math.pow(velocityUncertainty, 2))
                .processNoiseAcceleration(Math.pow(accelerationUncertainty, 2))
                .measurementNoisePosition(Math.pow(measurementNoise, 2))
                .build();
    }

    /**
     * Create state transition matrix for a given time delta
     *
     * @param dt Time delta in seconds
     * @return State transition matrix
     */
    private static RealMatrix createStateTransition(double dt) {
        RealMatrix stateTransition = MatrixUtils.createRealIdentityMatrix(STATE_DIM);

        // Position update formulas: x = x_prev + v_prev*dt + 0.5*a_prev*dt^2
        stateTransition.setEntry(0, 3, dt);
        stateTransition.setEntry(0, 6, 0.5 * dt * dt);
        stateTransition.setEntry(1, 4, dt);
        stateTransition.setEntry(1, 7, 0.5 * dt * dt);
        stateTransition.setEntry(2, 5, dt);
        stateTransition.setEntry(2, 8, 0.5 * dt * dt);

        // Velocity update formulas: v = v_prev + a_prev*dt
        stateTransition.setEntry(3, 6, dt);
        stateTransition.setEntry(4, 7, dt);
        stateTransition.setEntry(5, 8, dt);

        return stateTransition;
    }

    /**
     * Update the Kalman filter with a new position measurement
     *
     * @param position New position measurement
     * @param time Measurement time
     */
    public void update(GeoCoordinate position, ZonedDateTime time) {
        // Calculate time delta
        long deltaMillis = ChronoUnit.MILLIS.between(lastMeasurementTime, time);
        double dt = deltaMillis / 1000.0; // Convert to seconds

        // Update the process model with the new time delta
        RealMatrix stateTransition = createStateTransition(dt);

        // Update last measurement time
        lastMeasurementTime = time;

        // Convert position to ENU coordinates relative to reference point
        ENUCoordinate enu = position.toENU(referencePoint);

        // Create measurement vector
        RealVector measurement = new ArrayRealVector(MEASUREMENT_DIM);
        measurement.setEntry(0, enu.getEast());
        measurement.setEntry(1, enu.getNorth());
        measurement.setEntry(2, enu.getUp());

        // Predict and update
        kalmanFilter.predict();
        kalmanFilter.correct(measurement);

        // Update state vector and error covariance
        stateVector = toReal(kalmanFilter.getStateEstimation());
        errorCovariance = toReal(kalmanFilter.getErrorCovariance());
    }

    private RealMatrix toReal(double[][] errorCovariance) {
        return new Array2DRowRealMatrix(errorCovariance);
    }

    private RealVector toReal(double[] stateEstimation) {
        return new ArrayRealVector(stateEstimation);
    }

    /**
     * Predict position at a future time
     *
     * @param time Future time
     * @return Predicted position with uncertainty
     */
    public ProbabilisticPosition predict(ZonedDateTime time) {
        // Calculate time delta
        long deltaMillis = ChronoUnit.MILLIS.between(lastMeasurementTime, time);
        double dt = deltaMillis / 1000.0; // Convert to seconds

        // Create a copy of the Kalman filter for prediction
        RealVector currentState = kalmanFilter.getStateEstimationVector().copy();
        RealMatrix currentErrorCov = kalmanFilter.getErrorCovarianceMatrix().copy();

        // Create a new process model with the updated time delta
        RealMatrix stateTransition = createStateTransition(dt);

        // Scale process noise by time delta
        RealMatrix processNoise = new Array2DRowRealMatrix(STATE_DIM, STATE_DIM);
        // Position noise
        processNoise.setEntry(0, 0, processNoisePosition * dt);
        processNoise.setEntry(1, 1, processNoisePosition * dt);
        processNoise.setEntry(2, 2, processNoisePosition * dt);
        // Velocity noise
        processNoise.setEntry(3, 3, processNoiseVelocity * dt);
        processNoise.setEntry(4, 4, processNoiseVelocity * dt);
        processNoise.setEntry(5, 5, processNoiseVelocity * dt);
        // Acceleration noise
        processNoise.setEntry(6, 6, processNoiseAcceleration * dt);
        processNoise.setEntry(7, 7, processNoiseAcceleration * dt);
        processNoise.setEntry(8, 8, processNoiseAcceleration * dt);

        RealMatrix controlMatrix = new Array2DRowRealMatrix(STATE_DIM, 1);
        ProcessModel predictProcessModel = new DefaultProcessModel(
                stateTransition, controlMatrix, processNoise, currentState, currentErrorCov);

        // Create a temporary Kalman filter for prediction
        MeasurementModel measurementModel = new DefaultMeasurementModel(
                MatrixUtils.createRealIdentityMatrix(MEASUREMENT_DIM),
                MatrixUtils.createRealIdentityMatrix(MEASUREMENT_DIM));
        KalmanFilter tempKf = new KalmanFilter(predictProcessModel,
                measurementModel);

        // Predict
        tempKf.predict();

        // Get predicted state and covariance
        RealVector predictedState = tempKf.getStateEstimationVector();
        RealMatrix predictedCovariance = tempKf.getErrorCovarianceMatrix();

        // Extract position from state
        double east = predictedState.getEntry(0);
        double north = predictedState.getEntry(1);
        double up = predictedState.getEntry(2);

        // Convert ENU back to geodetic
        ENUCoordinate enu = ENUCoordinate.builder()
                .east(east)
                .north(north)
                .up(up)
                .build();

        GeoCoordinate predictedPosition = enu.toGeodetic(referencePoint);

        // Extract position covariance submatrix (3x3)
        RealMatrix positionCovariance = predictedCovariance.getSubMatrix(0, 2, 0, 2);

        // Create the probabilistic position
        return ProbabilisticPosition.builder()
                .meanPosition(predictedPosition)
                .covarianceMatrix(positionCovariance)
                .referencePosition(referencePoint)
                .confidenceLevel(0.95) // Default confidence level
                .build();
    }

    /**
     * Predict a trajectory with uncertainty over multiple time steps
     *
     * @param startTime Start time for prediction
     * @param endTime End time for prediction
     * @param numPoints Number of points to generate
     * @return List of probabilistic positions
     */
    public List<ProbabilisticPosition> predictTrajectory(
            ZonedDateTime startTime, ZonedDateTime endTime, int numPoints) {

        List<ProbabilisticPosition> trajectory = new ArrayList<>();

        // Calculate time interval
        long totalMillis = ChronoUnit.MILLIS.between(startTime, endTime);
        long intervalMillis = totalMillis / (numPoints - 1);

        for (int i = 0; i < numPoints; i++) {
            ZonedDateTime pointTime = startTime.plus(intervalMillis * i, ChronoUnit.MILLIS);
            trajectory.add(predict(pointTime));
        }

        return trajectory;
    }

    /**
     * Get the current state as a probabilistic position
     *
     * @return Current state as a probabilistic position
     */
    public ProbabilisticPosition getCurrentState() {
        // Extract position from state
        double east = stateVector.getEntry(0);
        double north = stateVector.getEntry(1);
        double up = stateVector.getEntry(2);

        // Convert ENU back to geodetic
        ENUCoordinate enu = ENUCoordinate.builder()
                .east(east)
                .north(north)
                .up(up)
                .build();

        GeoCoordinate position = enu.toGeodetic(referencePoint);

        // Extract position covariance submatrix (3x3)
        RealMatrix positionCovariance = errorCovariance.getSubMatrix(0, 2, 0, 2);

        // Create the probabilistic position
        return ProbabilisticPosition.builder()
                .meanPosition(position)
                .covarianceMatrix(positionCovariance)
                .referencePosition(referencePoint)
                .confidenceLevel(0.95) // Default confidence level
                .build();
    }

    /**
     * Get the current velocity vector in m/s
     *
     * @return Current velocity vector
     */
    public Vector3D getCurrentVelocity() {
        return Vector3D.builder()
                .x(stateVector.getEntry(3))
                .y(stateVector.getEntry(4))
                .z(stateVector.getEntry(5))
                .build();
    }

    /**
     * Get the current acceleration vector in m/s²
     *
     * @return Current acceleration vector
     */
    public Vector3D getCurrentAcceleration() {
        return Vector3D.builder()
                .x(stateVector.getEntry(6))
                .y(stateVector.getEntry(7))
                .z(stateVector.getEntry(8))
                .build();
    }
}

