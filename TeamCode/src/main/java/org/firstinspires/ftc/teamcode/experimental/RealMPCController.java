package org.firstinspires.ftc.teamcode.experimental;

/**
 * REAL MPC Controller with Quadratic Programming Solver
 * 
 * For a single-input system like the turret, we can solve the QP analytically
 * or use a simple iterative method. This implements a basic Active Set solver.
 * 
 * MPC formulates the problem as:
 *   minimize: J = sum(Q*(x-xref)^2 + R*u^2)
 *   subject to: x(k+1) = A*x(k) + B*u(k)
 *               u_min <= u <= u_max
 */
public class RealMPCController {

    // System model: x(k+1) = A*x(k) + B*u(k)
    // State: [position, velocity]
    // Input: motor power
    
    // Model parameters (tune these!)
    private double dt = 0.02;              // 50Hz control loop
    private double motorGain = 400;        // degrees/sec^2 per unit power
    private double friction = 0.85;        // velocity decay per step (1 = no friction)
    
    // MPC parameters
    private int horizon = 8;               // Prediction horizon (keep small for speed)
    private double Q_pos = 1.0;            // Position error weight
    private double Q_vel = 0.01;           // Velocity error weight  
    private double R = 0.1;                // Control effort weight
    
    // Constraints
    private double uMin = -0.6;
    private double uMax = 0.6;
    private double duMax = 0.15;           // Max change in control per step
    
    // State
    private double lastU = 0;
    private double estVelocity = 0;
    private double lastPosition = 0;
    
    // QP solver parameters
    private int maxIterations = 20;
    private double tolerance = 0.001;
    
    /**
     * Predict state forward using the linear model
     * x = [position, velocity]
     */
    private double[] predictState(double pos, double vel, double u) {
        double newVel = vel * friction + u * motorGain * dt;
        double newPos = pos + newVel * dt;
        return new double[]{newPos, newVel};
    }
    
    /**
     * Compute the gradient of the cost function w.r.t. control sequence
     * This is key to the QP solver!
     */
    private double[] computeGradient(double[] uSequence, double initPos, double initVel, double targetPos) {
        double[] gradient = new double[horizon];
        
        // Forward simulate to get trajectory
        double[] positions = new double[horizon];
        double[] velocities = new double[horizon];
        
        double pos = initPos;
        double vel = initVel;
        for (int i = 0; i < horizon; i++) {
            double[] next = predictState(pos, vel, uSequence[i]);
            pos = next[0];
            vel = next[1];
            positions[i] = pos;
            velocities[i] = vel;
        }
        
        // Compute gradient using chain rule (adjoint method)
        double[] lambda_pos = new double[horizon];
        double[] lambda_vel = new double[horizon];
        
        // Terminal cost gradient
        lambda_pos[horizon-1] = 2 * Q_pos * (positions[horizon-1] - targetPos);
        lambda_vel[horizon-1] = 2 * Q_vel * velocities[horizon-1];
        
        // Backward pass
        for (int i = horizon - 2; i >= 0; i--) {
            double nextLambdaPos = lambda_pos[i+1];
            double nextLambdaVel = lambda_vel[i+1];
            
            // Adjoint dynamics: λ(k) = A^T * λ(k+1) + ∂L/∂x
            lambda_pos[i] = nextLambdaPos + 2 * Q_pos * (positions[i] - targetPos);
            lambda_vel[i] = friction * nextLambdaVel + dt * nextLambdaPos + 2 * Q_vel * velocities[i];
        }
        
        // Gradient w.r.t. u
        for (int i = 0; i < horizon; i++) {
            // ∂J/∂u = ∂L/∂u + B^T * λ
            double BtLambda = motorGain * dt * dt * lambda_pos[i] + motorGain * dt * lambda_vel[i];
            gradient[i] = 2 * R * uSequence[i] + BtLambda;
        }
        
        return gradient;
    }
    
    /**
     * Project control onto constraint set
     */
    private double projectControl(double u, double lastU) {
        // Input magnitude constraint
        u = Math.max(uMin, Math.min(uMax, u));
        
        // Rate constraint
        double du = u - lastU;
        if (Math.abs(du) > duMax) {
            u = lastU + Math.signum(du) * duMax;
        }
        
        return u;
    }
    
    /**
     * Solve QP using projected gradient descent
     * Simple but effective for small problems! trust me
     */
    private double[] solveQP(double initPos, double initVel, double targetPos) {
        double[] u = new double[horizon];
        
        // Warm start: shift previous solution
        for (int i = 0; i < horizon - 1; i++) {
            u[i] = lastU;  // Simple warm start
        }
        u[horizon - 1] = 0;
        
        double stepSize = 0.1;
        
        for (int iter = 0; iter < maxIterations; iter++) {
            double[] gradient = computeGradient(u, initPos, initVel, targetPos);
            
            double maxGrad = 0;
            for (int i = 0; i < horizon; i++) {
                maxGrad = Math.max(maxGrad, Math.abs(gradient[i]));
            }
            
            if (maxGrad < tolerance) {
                break;  // Converged!
            }
            
            // Gradient descent step with projection
            double prevU = lastU;
            for (int i = 0; i < horizon; i++) {
                u[i] = u[i] - stepSize * gradient[i];
                u[i] = projectControl(u[i], prevU);
                prevU = u[i];
            }
            
            // Adaptive step size
            stepSize *= 0.95;
        }
        
        return u;
    }
    
    /**
     * Compute cost for a control sequence (for debugging)
     */
    public double computeCost(double[] uSequence, double initPos, double initVel, double targetPos) {
        double cost = 0;
        double pos = initPos;
        double vel = initVel;
        
        for (int i = 0; i < horizon; i++) {
            double[] next = predictState(pos, vel, uSequence[i]);
            pos = next[0];
            vel = next[1];
            
            cost += Q_pos * Math.pow(pos - targetPos, 2);
            cost += Q_vel * vel * vel;
            cost += R * uSequence[i] * uSequence[i];
        }
        
        return cost;
    }
    
    /**
     * Main update function - call every control loop
     */
    public double update(double currentPos, double targetPos) {
        // Estimate velocity
        estVelocity = (currentPos - lastPosition) / dt;
        lastPosition = currentPos;
        
        // Solve MPC optimization
        double[] optimalU = solveQP(currentPos, estVelocity, targetPos);
        
        // Apply first control only (receding horizon)
        double u = optimalU[0];
        
        // Final safety clamp
        u = projectControl(u, lastU);
        lastU = u;
        
        return u;
    }
    
    /**
     * Reset controller state
     */
    public void reset() {
        lastU = 0;
        estVelocity = 0;
        lastPosition = 0;
    }
    
    // Tuning setters
    public void setHorizon(int h) { this.horizon = h; }
    public void setPositionWeight(double q) { this.Q_pos = q; }
    public void setVelocityWeight(double q) { this.Q_vel = q; }
    public void setControlWeight(double r) { this.R = r; }
    public void setMaxPower(double max) { this.uMax = max; this.uMin = -max; }
    public void setMotorGain(double gain) { this.motorGain = gain; }
    public void setFriction(double f) { this.friction = f; }
    
    // Debug getters
    public double getEstimatedVelocity() { return estVelocity; }
    public double getLastControl() { return lastU; }
}
