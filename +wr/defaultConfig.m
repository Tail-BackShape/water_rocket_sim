function cfg = defaultConfig()
%DEFAULTCONFIG Return a template/demo configuration structure for water rocket optimization.
%   cfg = wr.defaultConfig() returns a struct containing physical constants,
%   solver settings, and DEMO rocket/nozzle parameters.
%
%   NOTE: All rocket dimensions, dry mass, initial pressure, and nozzle
%   bounds in this template are DEMO values for verification only. They do
%   not guarantee pressure vessel safety or actuator feasibility on real
%   hardware.

    cfg = struct();

    % Flag indicating this struct starts from the demo template
    cfg.isDemo = true;

    % --- Rocket geometry and mass (SI units; optional user-unit fields below) ---
    cfg.Vb = 1.5e-3;          % Tank internal volume [m^3] (1.5 L demo PET bottle)
    cfg.Vb_L = [];            % Optional: tank volume in liters [L] (used if Vb is empty)
    cfg.mDry = 0.250;         % Dry mass including nozzle & electronics [kg]
    cfg.D = 0.090;            % Body aerodynamic reference diameter [m] (90 mm)
    cfg.D_mm = [];            % Optional: diameter in [mm] (used if D is empty)

    % --- Launch and initial thermodynamic state ---
    cfg.pAmb = 101325.0;      % Ambient absolute pressure [Pa]
    cfg.pGauge = 400000.0;    % Initial tank gauge pressure [Pa] (400 kPaG demo)
    cfg.pGauge_kPa = [];      % Optional: gauge pressure in [kPa] (used if pGauge & p0Abs empty)
    cfg.p0Abs = [];           % Initial tank absolute pressure [Pa] (computed from pGauge if empty)
    cfg.theta0 = deg2rad(85); % Launch angle from horizontal [rad] (pi/2 = vertical)
    cfg.theta0_deg = [];      % Optional: launch angle in [deg] (used if theta0 is empty)
    cfg.T0 = 293.15;          % Initial tank air temperature [K]

    % --- Physical constants and environment ---
    cfg.rhoAir = 1.204;       % Ambient air density [kg/m^3]
    cfg.rhoW = 998.2;         % Water density [kg/m^3]
    cfg.Cd = 0.45;            % Aerodynamic drag coefficient [-]
    cfg.gamma = 1.4;          % Specific heat ratio of air [-]
    cfg.Rair = 287.05;        % Specific gas constant of air [J/(kg*K)]
    cfg.g = 9.80665;          % Gravitational acceleration [m/s^2]

    % --- Variable nozzle effective area bounds and slew rate ---
    cfg.Amin = pi * (0.005 / 2)^2; % Min effective nozzle area [m^2] (5 mm eq. dia.)
    cfg.Amax = pi * (0.015 / 2)^2; % Max effective nozzle area [m^2] (15 mm eq. dia.)
    cfg.dMin_mm = [];         % Optional: min equivalent diameter [mm] (used if Amin is empty)
    cfg.dMax_mm = [];         % Optional: max equivalent diameter [mm] (used if Amax is empty)
    cfg.ARateMax = 1.2e-3;    % Max rate of change of effective area [m^2/s]

    % --- Control parameterization ---
    cfg.tControlEnd = 0.30;   % End time of control knot schedule [s]
    cfg.nControlKnots = 6;    % Number of control knots [-]
    cfg.fillMin = 0.10;       % Minimum initial water fill fraction Vw0/Vb [-]
    cfg.fillMax = 0.65;       % Maximum initial water fill fraction Vw0/Vb [-]

    % --- Numerical integration and optimization settings ---
    cfg.vEpsilon = 0.20;      % Speed threshold for thrust-to-velocity alignment [m/s]
    cfg.relTol = 1e-6;        % ODE relative tolerance [-]
    cfg.absTol = 1e-8;        % ODE absolute tolerance [-]
    cfg.maxStep = 0.01;       % Maximum ODE step size [s]
    cfg.maxFlightTime = 15.0; % Maximum flight integration time before timeout [s]

    % --- Optimization options ---
    cfg.randomSeed = 42;      % Random seed for reproducible multi-start search
    cfg.nMultiStart = 3;      % Number of initial guesses for local optimization
    cfg.useGA = false;        % Optional Global Optimization Toolbox (ga) search
end
