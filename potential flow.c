while (simulationRunning) {
    
    // 1. State Propagation Inputs (from integrator)
    StateVector X = getCurrentState();   // u, v, w, p, q, r, phi, theta, psi, etc.
    ControlInputs U = getControls();     // stick, throttle, surfaces, etc.

    // 2. Flight Quantities
    FlightQuantities fq = computeFlightQuantities(X, environment); //computeFlightCondition
    // fq.alpha, fq.beta, fq.V, fq.qInf, fq.dynamicPressure, etc.

    // 3. Aerodynamic Coefficients
    AeroCoefficients coeffs = computeAeroCoefficients(fq, U, acParams);
    // CL, CD, CY, Cl, Cm, Cn, etc.

    // 4. Aerodynamic Forces & Moments
    Vector3 F_aero, M_aero;
    computeAeroForcesMoments(&F_aero, &M_aero, coeffs, fq, acParams);

    // 5. Other Forces & Moments
    Vector3 F_grav, F_thrust, M_thrust;
    computeGravityForces(&X, &acParams, &F_grav);
    computeThrustForces(&X, &U, &acParams, &F_thrust, &M_thrust);

    // 6. Total Forces & Moments
    Vector3 F_total = F_aero + F_grav + F_thrust;
    Vector3 M_total = M_aero + M_thrust;

    // 7. Equations of Motion (6DOF)
    dX = computeDerivatives(X, F_total, M_total, acParams);

    // 8. Integrate
    X = integrate(X, dX, dt);

    // 9. Save / Output
    logData(X, fq, coeffs, F_total, M_total);
}
