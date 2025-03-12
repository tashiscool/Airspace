package org.tash.core.uncertainty;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.distribution.MultivariateNormalDistribution;

import org.tash.data.GeoCoordinate;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.tash.extensions.geodetic.ENUCoordinate;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a position with uncertainty as a 3D error ellipsoid
 * This models position uncertainty using a multivariate normal distribution
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class ProbabilisticPosition {
    /** The mean position */
    private GeoCoordinate meanPosition;
    
    /** 
     * Covariance matrix (3x3) in local ENU coordinates (East, North, Up)
     * Units are in meters squared
     */
    private RealMatrix covarianceMatrix;
    
    /** Reference position for the local ENU frame */
    private GeoCoordinate referencePosition;
    
    /** 
     * Confidence level for error calculations (e.g., 0.95 for 95% confidence)
     * Default is 0.95 (95% confidence)
     */
    @Builder.Default
    private double confidenceLevel = 0.95;
    
    /**
     * Create a probabilistic position with circular horizontal error
     * 
     * @param position The mean position
     * @param horizontalErrorMeters The horizontal error (circular) in meters (1-sigma)
     * @param verticalErrorMeters The vertical error in meters (1-sigma)
     * @return A new ProbabilisticPosition
     */
    public static ProbabilisticPosition createWithCircularError(
            GeoCoordinate position, double horizontalErrorMeters, double verticalErrorMeters) {
        // Create diagonal covariance matrix with equal horizontal errors
        RealMatrix covariance = MatrixUtils.createRealMatrix(3, 3);
        
        // Set variances (squared errors)
        covariance.setEntry(0, 0, horizontalErrorMeters * horizontalErrorMeters); // East variance
        covariance.setEntry(1, 1, horizontalErrorMeters * horizontalErrorMeters); // North variance
        covariance.setEntry(2, 2, verticalErrorMeters * verticalErrorMeters);     // Up variance
        
        return ProbabilisticPosition.builder()
            .meanPosition(position)
            .covarianceMatrix(covariance)
            .referencePosition(position)
            .build();
    }
    
    /**
     * Create a probabilistic position with elliptical horizontal error
     * 
     * @param position The mean position
     * @param majorAxisMeters The major axis of the horizontal error ellipse in meters (1-sigma)
     * @param minorAxisMeters The minor axis of the horizontal error ellipse in meters (1-sigma)
     * @param orientationDeg The orientation of the ellipse major axis in degrees clockwise from north
     * @param verticalErrorMeters The vertical error in meters (1-sigma)
     * @return A new ProbabilisticPosition
     */
    public static ProbabilisticPosition createWithEllipticalError(
            GeoCoordinate position, double majorAxisMeters, double minorAxisMeters, 
            double orientationDeg, double verticalErrorMeters) {
        
        // Create covariance matrix for the elliptical error
        RealMatrix covariance = MatrixUtils.createRealMatrix(3, 3);
        
        // Convert orientation to radians counterclockwise from east
        // (navigation systems typically define orientation clockwise from north)
        double theta = Math.toRadians(90 - orientationDeg);
        
        // Calculate variances along the rotated axes
        double cos_theta = Math.cos(theta);
        double sin_theta = Math.sin(theta);
        double major_sq = majorAxisMeters * majorAxisMeters;
        double minor_sq = minorAxisMeters * minorAxisMeters;
        
        // Compute covariance terms for the ellipse
        double cov_xx = major_sq * cos_theta * cos_theta + minor_sq * sin_theta * sin_theta;
        double cov_yy = major_sq * sin_theta * sin_theta + minor_sq * cos_theta * cos_theta;
        double cov_xy = (major_sq - minor_sq) * sin_theta * cos_theta;
        
        // Set the covariance matrix entries
        covariance.setEntry(0, 0, cov_xx);  // East variance
        covariance.setEntry(1, 1, cov_yy);  // North variance
        covariance.setEntry(0, 1, cov_xy);  // East-North covariance
        covariance.setEntry(1, 0, cov_xy);  // North-East covariance
        covariance.setEntry(2, 2, verticalErrorMeters * verticalErrorMeters); // Up variance
        
        return ProbabilisticPosition.builder()
            .meanPosition(position)
            .covarianceMatrix(covariance)
            .referencePosition(position)
            .build();
    }
    
    /**
     * Get the probability density function value at a point
     * 
     * @param position The position to evaluate
     * @return The probability density value
     */
    public double getPdf(GeoCoordinate position) {
        // Convert to ENU coordinates relative to the reference position
        ENUCoordinate enuMean = ENUCoordinate.fromGeodetic(meanPosition, referencePosition);
        ENUCoordinate enuPoint = ENUCoordinate.fromGeodetic(position, referencePosition);
        
        // Create vector of deltas
        double[] delta = new double[] {
            enuPoint.getEast() - enuMean.getEast(),
            enuPoint.getNorth() - enuMean.getNorth(),
            enuPoint.getUp() - enuMean.getUp()
        };
        
        // Create multivariate normal distribution
        MultivariateNormalDistribution mvn = new MultivariateNormalDistribution(
            new double[] {0, 0, 0}, // mean is zero since we're using delta
            covarianceMatrix.getData()
        );
        
        // Return the PDF value
        return mvn.density(delta);
    }
    
    /**
     * Check if a point is within the error ellipsoid at the specified confidence level
     * 
     * @param position The position to check
     * @return True if the position is within the error ellipsoid
     */
    public boolean isWithinErrorEllipsoid(GeoCoordinate position) {
        // Convert to ENU coordinates relative to the reference position
        ENUCoordinate enuMean = ENUCoordinate.fromGeodetic(meanPosition, referencePosition);
        ENUCoordinate enuPoint = ENUCoordinate.fromGeodetic(position, referencePosition);
        
        // Create delta vector
        RealVector delta = MatrixUtils.createRealVector(new double[] {
            enuPoint.getEast() - enuMean.getEast(),
            enuPoint.getNorth() - enuMean.getNorth(),
            enuPoint.getUp() - enuMean.getUp()
        });
        
        // Get inverse of covariance matrix
        RealMatrix invCov = MatrixUtils.inverse(covarianceMatrix);
        
        // Calculate Mahalanobis distance squared
        double mahalanobisDistSq = delta.dotProduct(invCov.operate(delta));
        
        // For a 3D Gaussian, we use the chi-squared distribution with 3 degrees of freedom
        // The threshold value depends on the confidence level
        double threshold;
        if (confidenceLevel >= 0.99) {
            threshold = 11.345; // 99% confidence
        } else if (confidenceLevel >= 0.95) {
            threshold = 7.815; // 95% confidence
        } else if (confidenceLevel >= 0.90) {
            threshold = 6.251; // 90% confidence
        } else if (confidenceLevel >= 0.68) {
            threshold = 3.53; // 68% confidence (approximately 1-sigma)
        } else {
            // For other confidence levels, we'd use quantile of chi-squared distribution
            // but for simplicity, we'll use a default value
            threshold = 3.0;
        }
        
        // Check if point is within the ellipsoid
        return mahalanobisDistSq <= threshold;
    }
    
    /**
     * Get the horizontal and vertical error bounds at the specified confidence level
     * 
     * @return ErrorBounds containing horizontal and vertical errors
     */
    public ErrorBounds getErrorBounds() {
        // For 2D horizontal error, we use chi-squared with 2 degrees of freedom
        double horizontalThreshold;
        if (confidenceLevel >= 0.99) {
            horizontalThreshold = 9.21; // 99% confidence
        } else if (confidenceLevel >= 0.95) {
            horizontalThreshold = 5.991; // 95% confidence
        } else if (confidenceLevel >= 0.90) {
            horizontalThreshold = 4.605; // 90% confidence
        } else if (confidenceLevel >= 0.68) {
            horizontalThreshold = 2.28; // 68% confidence (approximately 1-sigma)
        } else {
            horizontalThreshold = 2.0;
        }
        
        // For 1D vertical error, we use normal distribution quantiles
        double verticalThreshold;
        if (confidenceLevel >= 0.99) {
            verticalThreshold = 2.576; // 99% confidence
        } else if (confidenceLevel >= 0.95) {
            verticalThreshold = 1.96; // 95% confidence
        } else if (confidenceLevel >= 0.90) {
            verticalThreshold = 1.645; // 90% confidence
        } else if (confidenceLevel >= 0.68) {
            verticalThreshold = 1.0; // 68% confidence (1-sigma)
        } else {
            verticalThreshold = 1.0;
        }
        
        // Get the horizontal (2D) submatrix
        RealMatrix horizontalCov = covarianceMatrix.getSubMatrix(0, 1, 0, 1);
        
        // Perform eigendecomposition to find major and minor axes
        EigenDecomposition eigen = new EigenDecomposition(horizontalCov);
        double[] eigenvalues = eigen.getRealEigenvalues();
        
        // Calculate semi-major and semi-minor axes
        double semiMajor = Math.sqrt(eigenvalues[eigenvalues.length - 1] * horizontalThreshold);
        double semiMinor = Math.sqrt(eigenvalues[0] * horizontalThreshold);
        
        // Calculate vertical error
        double verticalError = Math.sqrt(covarianceMatrix.getEntry(2, 2)) * verticalThreshold;
        
        // Calculate orientation (clockwise angle from north to major axis)
        double orientationRad = 0;
        if (eigenvalues[0] != eigenvalues[1]) { // Check if it's not a circle
            RealVector majorAxis = eigen.getEigenvector(eigenvalues.length - 1);
            orientationRad = Math.atan2(majorAxis.getEntry(0), majorAxis.getEntry(1));
            orientationRad = Math.PI/2 - orientationRad; // Adjust to clockwise from north
        }
        
        return new ErrorBounds(semiMajor, semiMinor, Math.toDegrees(orientationRad), verticalError);
    }
    
    /**
     * Generate a set of sample points on the error ellipsoid surface
     * Useful for visualization
     * 
     * @param numPoints Number of points to generate
     * @return List of points on the error ellipsoid surface
     */
    public List<GeoCoordinate> generateEllipsoidPoints(int numPoints) {
        List<GeoCoordinate> result = new ArrayList<>();
        
        // Perform eigendecomposition of the covariance matrix
        EigenDecomposition eigen = new EigenDecomposition(covarianceMatrix);
        double[] eigenvalues = eigen.getRealEigenvalues();
        RealMatrix eigenvectors = eigen.getV();
        
        // Calculate scaling factors for the confidence level
        // For a 3D Gaussian, we use chi-squared with 3 degrees of freedom
        double scaleFactor;
        if (confidenceLevel >= 0.99) {
            scaleFactor = Math.sqrt(11.345); // 99% confidence
        } else if (confidenceLevel >= 0.95) {
            scaleFactor = Math.sqrt(7.815); // 95% confidence
        } else if (confidenceLevel >= 0.90) {
            scaleFactor = Math.sqrt(6.251); // 90% confidence
        } else if (confidenceLevel >= 0.68) {
            scaleFactor = Math.sqrt(3.53); // 68% confidence
        } else {
            scaleFactor = Math.sqrt(3.0);
        }
        
        // Calculate the semi-axes of the ellipsoid
        double a = Math.sqrt(eigenvalues[2]) * scaleFactor;
        double b = Math.sqrt(eigenvalues[1]) * scaleFactor;
        double c = Math.sqrt(eigenvalues[0]) * scaleFactor;
        
        // Convert mean position to ENU
        ENUCoordinate enuMean = ENUCoordinate.fromGeodetic(meanPosition, referencePosition);
        
        // Generate points on the unit sphere
        for (int i = 0; i < numPoints; i++) {
            // Spherical coordinates
            double theta = Math.acos(2.0 * i / numPoints - 1) - Math.PI/2; // Latitude: -π/2 to π/2
            double phi = 2 * Math.PI * i / numPoints; // Longitude: 0 to 2π
            
            // Point on unit sphere
            double x = Math.cos(theta) * Math.cos(phi);
            double y = Math.cos(theta) * Math.sin(phi);
            double z = Math.sin(theta);
            
            // Scale to ellipsoid
            double x_scaled = a * x;
            double y_scaled = b * y;
            double z_scaled = c * z;
            
            // Rotate according to eigenvectors
            RealMatrix point = new Array2DRowRealMatrix(new double[][] {{x_scaled}, {y_scaled}, {z_scaled}});
            RealMatrix rotated = eigenvectors.multiply(point);
            
            // Translate to mean position
            double east = enuMean.getEast() + rotated.getEntry(0, 0);
            double north = enuMean.getNorth() + rotated.getEntry(1, 0);
            double up = enuMean.getUp() + rotated.getEntry(2, 0);
            
            // Convert to geodetic
            ENUCoordinate enuPoint = ENUCoordinate.builder()
                .east(east)
                .north(north)
                .up(up)
                .build();
                
            GeoCoordinate geoPoint = enuPoint.toGeodetic(referencePosition);
            
            result.add(geoPoint);
        }
        
        return result;
    }
    
    /**
     * Combine two probabilistic positions using covariance intersection method
     * This is a conservative fusion method that doesn't require independence assumption
     * 
     * @param other The other probabilistic position
     * @param weight Weight factor for the fusion (between 0 and 1)
     * @return Combined probabilistic position
     */
    public ProbabilisticPosition combine(ProbabilisticPosition other, double weight) {
        // Ensure weight is between 0 and 1
        weight = Math.max(0, Math.min(1, weight));
        
        // Both positions should use the same reference for covariance
        if (!this.referencePosition.equals(other.referencePosition)) {
            // Convert other position to use this reference
            other = other.withReference(this.referencePosition);
        }
        
        // Calculate weighted inverse covariances
        RealMatrix invCov1 = MatrixUtils.inverse(this.covarianceMatrix);
        RealMatrix invCov2 = MatrixUtils.inverse(other.covarianceMatrix);
        
        // Calculate combined inverse covariance
        RealMatrix combinedInvCov = invCov1.scalarMultiply(weight)
                                  .add(invCov2.scalarMultiply(1 - weight));
        
        // Calculate combined covariance
        RealMatrix combinedCov = MatrixUtils.inverse(combinedInvCov);
        
        // Calculate weighted means in ENU coordinates
        ENUCoordinate enu1 = ENUCoordinate.fromGeodetic(this.meanPosition, this.referencePosition);
        ENUCoordinate enu2 = ENUCoordinate.fromGeodetic(other.meanPosition, this.referencePosition);
        
        RealVector mean1 = MatrixUtils.createRealVector(
            new double[] {enu1.getEast(), enu1.getNorth(), enu1.getUp()});
        RealVector mean2 = MatrixUtils.createRealVector(
            new double[] {enu2.getEast(), enu2.getNorth(), enu2.getUp()});
        
        // Calculate combined mean
        RealVector combinedMeanVec = invCov1.scalarMultiply(weight)
                               .operate(mean1)
                               .add(invCov2.scalarMultiply(1 - weight)
                               .operate(mean2));
        
        combinedMeanVec = combinedCov.operate(combinedMeanVec);
        
        // Convert back to GeoCoordinate
        ENUCoordinate combinedEnuMean = ENUCoordinate.builder()
            .east(combinedMeanVec.getEntry(0))
            .north(combinedMeanVec.getEntry(1))
            .up(combinedMeanVec.getEntry(2))
            .build();
            
        GeoCoordinate combinedMean = combinedEnuMean.toGeodetic(this.referencePosition);
        
        // Create combined probabilistic position
        return ProbabilisticPosition.builder()
            .meanPosition(combinedMean)
            .covarianceMatrix(combinedCov)
            .referencePosition(this.referencePosition)
            .confidenceLevel(this.confidenceLevel)
            .build();
    }
    
    /**
     * Create a copy of this probabilistic position with a different reference point
     * 
     * @param newReference The new reference point
     * @return Copy with updated reference and appropriately transformed covariance
     */
    public ProbabilisticPosition withReference(GeoCoordinate newReference) {
        if (newReference.equals(this.referencePosition)) {
            return this; // No change needed
        }
        
        // Convert mean to ENU relative to current reference
        ENUCoordinate enuMean = ENUCoordinate.fromGeodetic(meanPosition, referencePosition);
        
        // The covariance transformation requires calculating the Jacobian of the 
        // coordinate transformation from old to new reference, which is complex.
        // For small distances between references, we can approximate by using the same covariance.
        // For large distances, a more accurate transformation would be needed.
        
        // Convert mean from ENU to geodetic
        GeoCoordinate meanGeo = meanPosition;
        
        // Now convert to ENU relative to new reference
        ENUCoordinate newEnuMean = ENUCoordinate.fromGeodetic(meanGeo, newReference);
        
        // Create new probabilistic position with new reference
        return ProbabilisticPosition.builder()
            .meanPosition(meanGeo)
            .covarianceMatrix(covarianceMatrix) // Approximate - assumes references are close
            .referencePosition(newReference)
            .confidenceLevel(confidenceLevel)
            .build();
    }
    
    /**
     * Error bounds for a probabilistic position
     */
    @Data
    @AllArgsConstructor
    public static class ErrorBounds {
        /** Semi-major axis length in meters */
        private double semiMajorAxisMeters;
        
        /** Semi-minor axis length in meters */
        private double semiMinorAxisMeters;
        
        /** Orientation in degrees (clockwise from north) */
        private double orientationDegrees;
        
        /** Vertical error in meters */
        private double verticalErrorMeters;
    }
}