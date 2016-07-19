package com.usfreu2016.blunavi.extendedkalmanfilter;


import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.NullArgumentException;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.CholeskyDecomposition;
import org.apache.commons.math3.linear.MatrixDimensionMismatchException;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.NonSquareMatrixException;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularMatrixException;
import org.apache.commons.math3.util.MathUtils;

// using apache commons code

public class ExtendedKalmanFilter {

    /** The process model used by this filter instance. */
    private final ProcessModel processModel;
    /** The measurement model used by this filter instance. */
    private final MeasurementModel measurementModel;
    /** The transition matrix, equivalent to A. */
    private RealMatrix transitionMatrix;
    /** The transposed transition matrix. */
    private RealMatrix transitionMatrixT;
    /** The control matrix, equivalent to B. */
    private RealMatrix controlMatrix;
    /** The measurement matrix, equivalent to H. */
    private RealMatrix measurementMatrix;
    /** The transposed measurement matrix. */
    private RealMatrix measurementMatrixT;
    /** The internal state estimation vector, equivalent to x hat. */
    private RealVector stateEstimation;
    /** The error covariance matrix, equivalent to P. */
    private RealMatrix errorCovariance;

    /** The bearing of the system */
    double radBearing;

    /**
     * Creates a new Kalman filter with the given process and measurement models.
     *
     * @param process
     *            the model defining the underlying process dynamics
     * @param measurement
     *            the model defining the given measurement characteristics
     * @throws NullArgumentException
     *             if any of the given inputs is null (except for the control matrix)
     * @throws NonSquareMatrixException
     *             if the transition matrix is non square
     * @throws DimensionMismatchException
     *             if the column dimension of the transition matrix does not match the dimension of the
     *             initial state estimation vector
     * @throws MatrixDimensionMismatchException
     *             if the matrix dimensions do not fit together
     */
    public ExtendedKalmanFilter(final ProcessModel process, final MeasurementModel measurement)
            throws NullArgumentException, NonSquareMatrixException, DimensionMismatchException,
            MatrixDimensionMismatchException {

        // Set the bearing to 0 initially
        this.radBearing = 0;

        MathUtils.checkNotNull(process);
        MathUtils.checkNotNull(measurement);

        this.processModel = process;
        this.measurementModel = measurement;

        transitionMatrix = processModel.getStateTransitionMatrix();
        MathUtils.checkNotNull(transitionMatrix);
        transitionMatrixT = transitionMatrix.transpose();

        // create an empty matrix if no control matrix was given
        if (processModel.getControlMatrix() == null) {
            controlMatrix = new Array2DRowRealMatrix();
        } else {
            controlMatrix = processModel.getControlMatrix();
        }

        measurementMatrix = measurementModel.getMeasurementMatrix();
        MathUtils.checkNotNull(measurementMatrix);
        measurementMatrixT = measurementMatrix.transpose();

        // check that the process and measurement noise matrices are not null
        // they will be directly accessed from the model as they may change
        // over time
        RealMatrix processNoise = processModel.getProcessNoise();
        MathUtils.checkNotNull(processNoise);
        RealMatrix measNoise = measurementModel.getMeasurementNoise();
        MathUtils.checkNotNull(measNoise);

        // set the initial state estimate to a zero vector if it is not
        // available from the process model
        if (processModel.getInitialStateEstimate() == null) {
            stateEstimation = new ArrayRealVector(transitionMatrix.getColumnDimension());
        } else {
            stateEstimation = processModel.getInitialStateEstimate();
        }

        if (transitionMatrix.getColumnDimension() != stateEstimation.getDimension()) {
            throw new DimensionMismatchException(transitionMatrix.getColumnDimension(),
                    stateEstimation.getDimension());
        }

        // initialize the error covariance to the process noise if it is not
        // available from the process model
        if (processModel.getInitialErrorCovariance() == null) {
            errorCovariance = processNoise.copy();
        } else {
            errorCovariance = processModel.getInitialErrorCovariance();
        }

        // sanity checks, the control matrix B may be null

        // A must be a square matrix
        if (!transitionMatrix.isSquare()) {
            throw new NonSquareMatrixException(
                    transitionMatrix.getRowDimension(),
                    transitionMatrix.getColumnDimension());
        }

        // row dimension of B must be equal to A
        // if no control matrix is available, the row and column dimension will be 0
        if (controlMatrix != null &&
                controlMatrix.getRowDimension() > 0 &&
                controlMatrix.getColumnDimension() > 0 &&
                controlMatrix.getRowDimension() != transitionMatrix.getRowDimension()) {
            throw new MatrixDimensionMismatchException(controlMatrix.getRowDimension(),
                    controlMatrix.getColumnDimension(),
                    transitionMatrix.getRowDimension(),
                    controlMatrix.getColumnDimension());
        }

        // Q must be equal to A
        MatrixUtils.checkAdditionCompatible(transitionMatrix, processNoise);

        // column dimension of H must be equal to row dimension of A
        if (measurementMatrix.getColumnDimension() != transitionMatrix.getRowDimension()) {
            throw new MatrixDimensionMismatchException(measurementMatrix.getRowDimension(),
                    measurementMatrix.getColumnDimension(),
                    measurementMatrix.getRowDimension(),
                    transitionMatrix.getRowDimension());
        }

        // row dimension of R must be equal to row dimension of H
        if (measNoise.getRowDimension() != measurementMatrix.getRowDimension()) {
            throw new MatrixDimensionMismatchException(measNoise.getRowDimension(),
                    measNoise.getColumnDimension(),
                    measurementMatrix.getRowDimension(),
                    measNoise.getColumnDimension());
        }
    }

    /**
     * Sets the radian bearing of the system
     *
     * @param radBearing the bearing expressed in radians
     */
    public void setRadBearing(double radBearing) {
        this.radBearing = radBearing;
    }

    /**
     * Returns the dimension of the state estimation vector.
     *
     * @return the state dimension
     */
    public int getStateDimension() {
        return stateEstimation.getDimension();
    }

    /**
     * Returns the dimension of the measurement vector.
     *
     * @return the measurement vector dimension
     */
    public int getMeasurementDimension() {
        return measurementMatrix.getRowDimension();
    }

    /**
     * Returns the current state estimation vector.
     *
     * @return the state estimation vector
     */
    public double[] getStateEstimation() {
        return stateEstimation.toArray();
    }

    /**
     * Returns a copy of the current state estimation vector.
     *
     * @return the state estimation vector
     */
    public RealVector getStateEstimationVector() {
        return stateEstimation.copy();
    }

    /**
     * Returns the current error covariance matrix.
     *
     * @return the error covariance matrix
     */
    public double[][] getErrorCovariance() {
        return errorCovariance.getData();
    }

    /**
     * Returns a copy of the current error covariance matrix.
     *
     * @return the error covariance matrix
     */
    public RealMatrix getErrorCovarianceMatrix() {
        return errorCovariance.copy();
    }

    /**
     * Predict the internal state estimation one time step ahead.
     */
    public void predict() {
        predict((RealVector) null);
    }

    /**
     * Predict the internal state estimation one time step ahead.
     *
     * @param u
     *            the control vector
     * @throws DimensionMismatchException
     *             if the dimension of the control vector does not fit
     */
    public void predict(final double[] u) throws DimensionMismatchException {
        predict(new ArrayRealVector(u, false));
    }

    /**
     * Predict the internal state estimation one time step ahead.
     *
     * @param u
     *            the control vector
     * @throws DimensionMismatchException
     *             if the dimension of the control vector does not match
     */
    public void predict(final RealVector u) throws DimensionMismatchException {
        // sanity checks
        if (u != null &&
                u.getDimension() != controlMatrix.getColumnDimension()) {
            throw new DimensionMismatchException(u.getDimension(),
                    controlMatrix.getColumnDimension());
        }

        // project the state estimation ahead (a priori state)
        // xHat(k)- = A * xHat(k-1) + B * u(k-1)
        stateEstimation = transitionMatrix.operate(stateEstimation);

        // add control input if it is available
        if (u != null) {
            stateEstimation = stateEstimation.add(controlMatrix.operate(u));
        }

        // project the error covariance ahead
        // P(k)- = A * P(k-1) * A' + Q
        errorCovariance = transitionMatrix.multiply(errorCovariance)
                .multiply(transitionMatrixT)
                .add(processModel.getProcessNoise());
    }

    /**
     * Correct the current state estimate with an actual measurement.
     *
     * @param z
     *            the measurement vector
     * @throws NullArgumentException
     *             if the measurement vector is {@code null}
     * @throws DimensionMismatchException
     *             if the dimension of the measurement vector does not fit
     * @throws SingularMatrixException
     *             if the covariance matrix could not be inverted
     */
    public void correct(final double[] z)
            throws NullArgumentException, DimensionMismatchException, SingularMatrixException {
        correct(new ArrayRealVector(z, false));
    }

    /**
     * Correct the current state estimate with an actual measurement.
     *
     * @param z
     *            the measurement vector
     * @throws NullArgumentException
     *             if the measurement vector is {@code null}
     * @throws DimensionMismatchException
     *             if the dimension of the measurement vector does not fit
     * @throws SingularMatrixException
     *             if the covariance matrix could not be inverted
     */
    public void correct(final RealVector z)
            throws NullArgumentException, DimensionMismatchException, SingularMatrixException {

        // sanity checks
        MathUtils.checkNotNull(z);
        if (z.getDimension() != measurementMatrix.getRowDimension()) {
            throw new DimensionMismatchException(z.getDimension(),
                    measurementMatrix.getRowDimension());
        }

        // S = H * P(k) * H' + R
        RealMatrix s = measurementMatrix.multiply(errorCovariance)
                .multiply(measurementMatrixT)
                .add(measurementModel.getMeasurementNoise());

        // Inn = z(k) - H * xHat(k)-
        RealVector innovation = z.subtract(measurementMatrix.operate(stateEstimation));

        // calculate gain matrix
        // K(k) = P(k)- * H' * (H * P(k)- * H' + R)^-1
        // K(k) = P(k)- * H' * S^-1

        // instead of calculating the inverse of S we can rearrange the formula,
        // and then solve the linear equation A x X = B with A = S', X = K' and B = (H * P)'

        // K(k) * S = P(k)- * H'
        // S' * K(k)' = H * P(k)-'
        RealMatrix kalmanGain = new CholeskyDecomposition(s).getSolver()
                .solve(measurementMatrix.multiply(errorCovariance.transpose()))
                .transpose();

        // update estimate with measurement z(k)
        // xHat(k) = xHat(k)- + K * Inn
        // Also, account for bearing of system
//        stateEstimation = stateEstimation.add(kalmanGain.operate(innovation));
        RealVector tempEstimation = stateEstimation.add(kalmanGain.operate(innovation));
        RealVector differenceVector = tempEstimation.subtract(stateEstimation);
        double[] differences = stateEstimation.toArray();

        // X value
        differences[0] = differenceVector.getEntry(0) * Math.abs(Math.sin(radBearing));
        // Y value
        differences[1] = differenceVector.getEntry(1) * Math.abs(Math.cos(radBearing));

        differenceVector = new ArrayRealVector(differences);
        stateEstimation = stateEstimation.add(differenceVector);

        // update covariance of prediction error
        // P(k) = (I - K * H) * P(k)-
        RealMatrix identity = MatrixUtils.createRealIdentityMatrix(kalmanGain.getRowDimension());
        errorCovariance = identity.subtract(kalmanGain.multiply(measurementMatrix)).multiply(errorCovariance);
    }
}
