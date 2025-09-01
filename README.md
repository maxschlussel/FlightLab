
<p align="center">
  <img src="docs/images/logo.png" width="900" />
</p>

# FlightLab – Virtual Aviation Environment  

A modular simulation platform for aircraft dynamics, control algorithms, and flight visualization.  Written in C.

## 📋 Features

<div style="margin-left: 20px;">
  <img align="right" src="docs/images/flight.gif" width="35%" />
</div>

- 🛫 **6-DOF**: Realistic 6DOF flight dynamics  
- 🎮 **Control System**: Control system design & testing environment  
- 🔧 **Modular:** Modular, scalable, and extensible C-based simulation core  
- 🚀 **Models**: Extensible framework for new models and aircraft types  
- 🎲 **Monte Carlo**: Monte Carlo simulation for robustness and uncertainty analysis  
- 🌍 **Environment**: Wind, gusts, and atmospheric modeling  
- 📊 **Data**: Automatic data logging & plotting  
- 🖥️ **Visualization**: Integration with FlightGear for 3D visualization  
- 🎥 **Recording**: Aircraft Video/ GIF recording for animations  
- 🧩 **Clean**: Clean project structure for easy development  


## 📊 Example Output

- Complete data log timeseries (position, velocity, attitude, rate, alpha, etc.)  
- Control surface deflections and actuator responses  
- Sensor output logs (accelerometer, gyro, etc.)
- 3D animation/ trajectory visualization  

<p align="center">
  <img src="docs/images/plot.png" width="600" />
</p>


## ⚡ Quick Start

```bash
# Clone repo
git clone https://github.com/maxschlussel/FlightLab.git
cd FlightLab

# Build
make

# Run simulation
./flightlab.exe --config examples/projectile.cfg
```


## 🔄 Simulation Flow  
1. **Initialization**  
   - Load aircraft parameters, I.C. state vector, and control inputs  
   - Initialize sensors and actuators  

2. **Main Loop** (per time step `dt`)  
   - Read and process sensor data
   - Compute flight controls (U_cmd)
   - Actuate flight controls (servo dynamics)  
   - Compute aerodynamic forces & moments  
   - Compute state derivative (6-DOF dynamics EOM)
   - Integrate state derivative
   - Log outputs  

3. **Post-Processing**  
   - Results stored in `output/data_log.csv`  
   - Python plotting scripts visualize important timeseries parameters

---

## 📂 Project Structure  
```
FlighLab/
├─ src/                         # All code (headers colocated with sources)
│  ├─ core/                     # Core data structures/ types / parameters used everywhere
│  │  ├─ state.h/.c             # StateVector (u,v,w,p,q,r,φ,θ,ψ,x,y,z), helpers
│  │  ├─ controls.h/.c          # ControlVector (δa,δe,δr, throttles), limits, mapping
│  │  ├─ constants.h/.c         # Physical constants, sim constants (G, R, rho0, eps)
│  │  ├─ aircraft_params.h/.c   # Struct of mass/inertia/geometry/refs (public)
│  │  └─ config.h/.c            # Run config: dt, sim length, feature flags
│  │
│  ├─ math/                     # Small, dependency-free math utilities
│  │  ├─ vec3.h/.c              # add, sub, dot, cross, scale, norm, normalize
│  │  ├─ mat3.h/.c              # 3x3 ops: mul, transpose, det, inverse
│  │  ├─ quat.h/.c              # quaternions (if/when you add them)
│  │  ├─ dcm.h/.c               # Euler↔DCM, body↔NED, wind↔body, etc.
│  │  ├─ stats.h/.c             # clamp, saturate, lerp, deadband, moving avg
│  │  └─ utils.h/.c             # deg↔rad, sign, safe_atan2, eps, numeric guards
│  │
│  ├─ dynamics/                 # Flight dynamics “physics”
│  │  ├─ kinematics/            # Rates/angles/positions transforms
│  │  │  ├─ eom.h/.c            # computeStateDerivative(...) 6-DOF core
│  │  │  ├─ airdata.h/.c        # computeAlphaBetaTasQinf(...), Mach, Re, TAS→CAS
│  │  │  └─ frames.h/.c         # body↔inertial↔stability↔wind transforms
│  │  ├─ environment/           # Atmosphere & environment models
│  │  │  ├─ isa.h/.c            # ISA/US Standard Atmosphere, density/pressure/T
│  │  │  ├─ wind.h/.c           # steady wind, turbulence models (Dryden, von Karman)
│  │  │  └─ gravity.h/.c        # gravity model (WGS-84/NED), body projection
│  │  ├─ forces/                # Force contributors
│  │  │  ├─ aero.h/.c           # computeAerodynamicLoads(...) (F_aero,M_aero)
│  │  │  ├─ propulsion.h/.c     # thrust models, engine/prop torque, moments
│  │  │  ├─ landing_gear.h/.c   # ground reaction, friction, steering
│  │  │  └─ miscellaneous.h/.c  # tail strike, parasite forces, etc (optional)
│  │  ├─ massprops/             # Mass properties & CG management
│  │  │  ├─ inertia.h/.c        # inertia tensor build, parallel-axis, updates
│  │  │  └─ cg.h/.c             # cg position, fuel shift, payload ops
│  │  └─ integrators/           # Time integration schemes
│  │     ├─ euler.h/.c          # integrateEulerStep(...)
│  │     ├─ rk4.h/.c            # integrateRK4(...)
│  │     └─ semi_implicit.h/.c  # symplectic/semi-implicit Euler (optional)
│  │
│  ├─ systems/                  # Higher-level aircraft/system models
│  │  ├─ aero_model/            # Coefficient maps, lookup, blending
│  │  │  ├─ c172_coeffs.h/.c    # polars, derivatives for C172
│  │  │  ├─ flaps_model.h/.c    # flap effects on CL/CD/CM, drag buckets
│  │  │  └─ ground_effect.h/.c  # near-ground lift/drag mods
│  │  ├─ propulsion_model/
│  │  │  ├─ fixed_pitch.h/.c    # fixed-pitch prop model
│  │  │  └─ engine_map.h/.c     # engine torque map vs RPM, throttle
│  │  └─ controls_model/
│  │     ├─ mixer.h/.c          # control input mixing, trims, limits
│  │     └─ actuator.h/.c       # actuator rate limits, lags
│  │
│  ├─ io/                       # Input/output & utilities
│  │  ├─ logger.h/.c            # CSV/TSV logger, rotating files, headers
│  │  ├─ telemetry.h/.c         # optional runtime stream (UDP/serial)
│  │  └─ cli.h/.c               # parse args, config files
│  │
│  ├─ sim/                      # Simulation “orchestration”
│  │  ├─ main.c                 # main(), run loop, wiring modules
│  │  ├─ loop.h/.c              # stepSim(...): read→forces→integrate→log
│  │  └─ scenarios.h/.c         # scenario setups (trim, climb, turn, etc.)
│  │
│  ├─ controllers               # Autopilot/RL controllers
│  ├─ actuators                 # Servo models, propulsion actuators
│  ├─ sensors                   # Sensors, filters (INS, complementary/KF), estimators
│  └─ tests/                    # Unit tests (if adding)
│
├─ output/                      # Runtime results: CSV logs, plots, data
│  ├─ logs/
│  ├─ plots/
│  └─ data/
├─ scripts/                     # Post-processing Python/Matlab
├─ build/                       # Build artifacts (obj/bin separated if you like)
├─ Makefile
└─ README.md

```

---

## 🗺️ Roadmap  

- [x] Implement fundamental 6DOF dynamic EOM
- [x] Implement state integrator models
- [x] Create basic logging and plotting pipeline
- [x] Create core data structures (StateVectos, ControlVector, Vec3, AircraftParams, etc.)
- [x] Integrate basic aircraft model (aerodynamics parameters and coefficient equations)
- [x] Develop basic actuator and sensor models
- [ ] Add control surface modeling  
- [ ] Design autopilot & control laws (PID, ML etc.) 
- [ ] Connect with FlightGear for 3D visualization
- [ ] Increase fidelity (arodynamics, sensors, environment, mass, etc.)
- [ ] Monte-Carlo simulation for robustness studies  
- [ ] Real-time simulator mode  
- [ ] Expand aircraft database  

## 📜 License  
This project is licensed under the **MIT License** – see the [LICENSE](LICENSE) file for details.  
