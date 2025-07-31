% Motion Filter for Surface Vehicles with non-linear Dynamics
% 
% Construct an object using 'motionfilter.MotionFilter3DoF()' and run the 'Step()' function of the object. Inputs and outputs
% are stuctured values that can be created by 'motionfilter.MotionFilter3DoF.GetInputStructFormat()' and
% 'motionfilter.MotionFilter3DoF.GetOutputStructFormat()'. The motion filter uses the first measurement for initialization.
% 
% This motion filter estimates the motion state and external disturbance force based on a non-linear dynamical model using a
% square-root spherical simplex unscented kalman filter (SRSSUKF). The dynamics of the vehicle are given by
% 
% ( x_dot )          ( u )
% ( y_dot ) = Rb2n * ( v )
% (psi_dot)          ( r )
% 
% ( u_dot )                                                          (( X )            ( Xe ))
% ( v_dot ) = F * [u; v; r; v*r; u*r; u*v; r^2; u^3; v^3; r^3] + B * (( Y ) + Rb2n^T * ( Ye )) + w_uvr
% ( r_dot )                                                          (( N )            ( Ne ))
% 
% ( Xe_dot)
% ( Ye_dot) = w_XYN
% ( Ne_dot)
% 
% where R = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1] is a rotation matrix (body-frame to navigation-frame), F and B
% are constant matrices and (Xe,Ye,Ne)^T denotes the external disturbance force (given in the navigation-frame). w_uvr and
% w_XYN represents the process noise for velocity and external force model, respectively. (X,Y,N)^T denotes the model input.
% 
% The internal Kalman filter operates in euclidean space, using x and y for position. However, the measurement data as well as
% the resulting state estimation is represented in geographic coordinates according to WGS84, e.g. latitude and longitude.
% Therefore this motion filter uses the latest position measurement as geographic origin and runs the Kalman filter for the
% tangential plane around the current origin.
classdef MotionFilter3DoF < handle
    methods
        function this = MotionFilter3DoF()
            %motionfilter.MotionFilter3DoF.MotionFilter3DoF Constuct a motion filter object for motion and disturbance estimation of a surface vehicle.

            % construct internal Kalman filter
            w0 = 0.5;             % weight of 0-th sigma-point
            xDim = 9;             % state vector dimension (x,y,psi,u,v,r,Xe,Ye,Ne)
            wDim = 0;             % no state augmentation for process noise
            idxAngle = 3;         % heading angle at x(i=3)
            idxQuaternion = [];   % no quaternion
            cyDim = {6};          % measurement dimension (x,y,psi,u,v,r)
            cvDim = {0};          % no state augmentation for measurement noise
            this.srssukf = SRSSUKF(w0, xDim, wDim, idxAngle, idxQuaternion, cyDim, cvDim);

            % set initial values
            this.Reset();
        end
        function structOutput = Step(this, structInput)
            %motionfilter.MotionFilter3DoF.Step Perform a one step calculation of the motion filter by processing the input structure.
            % This motion filter requires at least two step calls for the initialization to be completed.
            % 
            % PARAMETER
            % structInput ... Input structure containing model and measurement data. A default input structure is obtained by motionfilter.MotionFilter3DoF.GetInputStructFormat().
            % 
            % RETURN
            % structOutput ... Output structure containing the estimation result. The structure format is equal to motionfilter.MotionFilter3DoF.GetOutputStructFormat().

            % check for reset
            if(structInput.reset)
                this.Reset(); % sets isInitialized to false
            end

            % do job depending on state
            if(~this.isInitialized)
                this.StepInitialize(structInput);
            else
                this.StepMain(structInput);
            end
            this.previousInputTimestamp = structInput.input.timestamp; % do this before calculate output
            this.previousMeasureTimestamp = structInput.measurement.timestamp;

            % assign output structure
            structOutput = this.CalculateOutput();
        end
    end
    methods(Static)
        function xdot = ProcessModelODE(x, XYN, model)
            %motionfilter.MotionFilter3DoF.ProcessModelODE The ode function for the process model of the surface vehicle.
            % 
            % PARAMETER
            % x     ... The current state vector given as [x; y; psi; u; v; r; Xe; Ye; Ne].
            % XYN   ... 3-by-1 vector indicating the current input force to the model.
            % model ... Structure containing model parameters that has been input to the motion filter.
            % 
            % RETURN
            % xdot  ... 9-by-1 vector indicating the time derivative of the state for the current time point.

            % some precalculations
            c = cos(x(3));
            s = sin(x(3));
            R = [c -s 0; s c 0; 0 0 1];
            u = x(4);
            v = x(5);
            r = x(6);
            extXYN_body = R' * x(7:9);

            % ordinary differential equation
            xdot = [
                R * [u; v; r];
                model.matF * [u; v; r; v*r; u*r; u*v; r*r; u*u*u; v*v*v; r*r*r] + model.matB * (XYN + extXYN_body);
                zeros(3,1)
            ];
        end
        function structInput = GetInputStructFormat()
            structInput = struct( ...
                'reset',                 false, ...            % True if filter is to be reset, false otherwise.
                'model', struct( ...                           % ### MODEL PARAMETERS
                    'matF',              zeros(3,10), ...      % F matrix for model uvr_dot = F * [u; v; r; v*r; u*r; u*v; r^2; u^3; v^3; r^3] + B * tau.
                    'matB',              zeros(3) ...          % B matrix for model uvr_dot = F * [u; v; r; v*r; u*r; u*v; r^2; u^3; v^3; r^3] + B * tau.
                ), ...
                'stddevModel', struct( ...                     % ### MODEL STANDARD DEVIATION
                    'velocityUVR', zeros(3,1), ...             % Uncertainty for velocity model uvr_dot = F * [u; v; r; v*r; u*r; u*v; r^2; u^3; v^3; r^3] + B * tau.
                    'externalXYN', zeros(3,1) ...              % Uncertainty for external force model XeYeNe_dot = 0.
                ), ...
                'input', struct( ...                           % ### MODEL INPUT VALUE
                    'timestamp',         0.0, ...              % Monotonically increasing timestamp in seconds indicating a new model prediction step.
                    'XYN',               zeros(3,1) ...        % Input force to the model.
                ), ...
                'measurement', struct( ...                     % ### MEASUREMENT VALUE
                    'timestamp',         0.0, ...              % Monotonically increasing timestamp in seconds indicating a new measurement update step.
                    'positionLLA',       zeros(3,1), ...       % Geographic position according to WGS84 given in latitude [rad], longitude [rad] and altitude [m] (positive upwards).
                    'orientationYaw',    0.0, ...              % Heading angle in radians.
                    'velocityUVR',       zeros(3,1) ...        % Body-fixed velocity given as surge velocity [m/s], sway velocity [m/s] and angular rate [rad/s].
                ), ...
                'stddevMeasurement', struct( ...               % ### MEASUREMENT STANDARD DEVIATION
                    'positionNorthEast', zeros(2,1), ...       % Stddev in north/east direction in meters.
                    'orientationYaw',    0.0, ...              % Stddev for heading angle in radians.
                    'velocityUVR',       zeros(3,1) ...        % Stddev for body-fixed velocities given as surge velocity [m/s], sway velocity [m/s] and angular rate [rad/s].
                ) ...
            );
        end
        function structOutput = GetOutputStructFormat()
            structOutput = struct( ...
                'valid',                 false, ...            % True if motion filter is initialized and running, false otherwise.
                'timestamp',             0.0, ...              % Monotonically increasing timestamp in seconds equal to the input timestamp.
                'motion', struct( ...                          % ### MOTION STATE ESTIMATION
                    'positionLLA',       zeros(3,1), ...       % Geographic position according to WGS84 given in latitude [rad], longitude [rad] and altitude [m] (positive upwards).
                    'orientationYaw',    0.0, ...              % Heading angle in radians.
                    'velocityUVR',       zeros(3,1) ...        % Body-fixed velocity given as surge velocity [m/s], sway velocity [m/s] and angular rate [rad/s].
                ), ...
                'disturbance', struct( ...                     % ### DISTURBANCE ESTIMATION
                    'XYN',               zeros(3,1) ...        % External force.
                ) ...
            );
        end
    end
    methods(Access = private)
        function Reset(this)
            % Reset all variables to default values causing the motion filter to go into initialization mode
            this.isInitialized = false;
            this.previousInputTimestamp = Inf;
            this.previousMeasureTimestamp = Inf;
            this.originLLA = zeros(3,1);
        end
        function StepInitialize(this, in)
            % The initialization waits for the next measurement data (depending on timestamp) and uses the input data as initial estimate and initial standard deviation.
            % At the beginning and after each reset the previousMeasureTimestamp is non-finite.
            % The actual initialization is delayed by at least one step but therefore prevents old measurement data from being used for initialization.

            % set origin and initial state
            this.originLLA = in.measurement.positionLLA;
            xNorth = 0;
            yEast = 0;
            externalForce = zeros(3,1);
            x0 = [xNorth; yEast; in.measurement.orientationYaw; in.measurement.velocityUVR; externalForce];

            % if new measurement available, initialize the filter
            if(isfinite(this.previousMeasureTimestamp) && (in.measurement.timestamp > this.previousMeasureTimestamp))
                % initialize internal filter
                dt = in.measurement.timestamp - this.previousMeasureTimestamp; % might be greater than actual input sampletime but it's fine for initial uncertainty to be greater
                S0 = dt * diag([in.stddevMeasurement.positionNorthEast; in.stddevMeasurement.orientationYaw; in.stddevMeasurement.velocityUVR; 1e-2*ones(3,1)]);
                this.srssukf.Initialize(x0, S0);

                % initialization completed
                this.isInitialized = true;
            else
                % assign measurement data to filter state to bypass measurement to output during initialization
                this.srssukf.SetInternalState(x0);
            end
        end
        function StepMain(this, in)
            % Predict the motion state using a dynamical model of the surface vehicle and update the state using pose and velocity measurement.
            % The internal filter uses an euclidean coordinate system but measurements are given in WGS84.
            % The latest measurement is used as geographic origin for the internal filter state.
            dt = in.input.timestamp - this.previousInputTimestamp;
            if(dt > 0)
                % predict motion state
                sqrtQ = dt * diag([zeros(3,1); in.stddevModel.velocityUVR; in.stddevModel.externalXYN]);
                this.srssukf.Predict(@motionfilter.MotionFilter3DoF.PredictRK4, dt, in.input.XYN, sqrtQ, in.model);

                if(in.measurement.timestamp > this.previousMeasureTimestamp)
                    % update motion state
                    [north, east, ~] = ave.LLA2NED(in.measurement.positionLLA(1), in.measurement.positionLLA(2), in.measurement.positionLLA(3), this.originLLA(1), this.originLLA(2), this.originLLA(3));
                    y = [north; east; in.measurement.orientationYaw; in.measurement.velocityUVR];
                    sqrtR = dt * diag([in.stddevMeasurement.positionNorthEast; in.stddevMeasurement.orientationYaw; in.stddevMeasurement.velocityUVR]);
                    measurementModel = @(x,v,optArg)(x(1:6));
                    this.srssukf.Update(measurementModel, 1, y, 3, sqrtR, []); % sensorIndex=1, idxAngle=3

                    % reset origin to measurement and transform internal filter state to new origin
                    this.originLLA = in.measurement.positionLLA;
                    x = this.srssukf.GetInternalState();
                    x(1:2) = x(1:2) - [north; east];
                    this.srssukf.SetInternalState(x);
                end
            end
        end
        function structOutput = CalculateOutput(this)
            % Calculate the output structure based on the current estimation of the internal filter.
            % Note, that the filter uses an euclidean coordinate system, while the output is to be represented in geographic coordinates according to WGS84.
            x = this.srssukf.GetState();
            [lat, lon, ~] = ave.NED2LLA(x(1), x(2), 0, this.originLLA(1), this.originLLA(2), this.originLLA(3));
            structOutput = this.GetOutputStructFormat();
            structOutput.valid = this.isInitialized;
            structOutput.timestamp = this.previousInputTimestamp; % stores the current timestamp
            structOutput.motion.positionLLA = [lat; lon; this.originLLA(3)];
            structOutput.motion.orientationYaw = ave.SymmetricalAngle(x(3));
            structOutput.motion.velocityUVR = x(4:6);
            structOutput.disturbance.XYN = x(7:9);
        end
    end
    methods(Static, Access = private)
        function xout = PredictRK4(x, ~, u, Ts, model)
            % Prediction callback function for the process model.
            % It uses the 4-th order runge-kutta method (RK4) to predict the state of a system.
            Ts_half = Ts/2;
            k1 = motionfilter.MotionFilter3DoF.ProcessModelODE(x, u, model);
            k2 = motionfilter.MotionFilter3DoF.ProcessModelODE(x + Ts_half * k1, u, model);
            k3 = motionfilter.MotionFilter3DoF.ProcessModelODE(x + Ts_half * k2, u, model);
            k4 = motionfilter.MotionFilter3DoF.ProcessModelODE(x + Ts * k3, u, model);
            xout = x + Ts/6 * (k1 + k2+k2 + k3+k3 + k4);
        end
    end
    properties(Access = private)
        srssukf;                    % Internal Kalman filter object. Estimates motion state in euclidean space w.r.t. originLLA.
        isInitialized;              % True if filter has been initialized, false otherwise.
        previousInputTimestamp;     % Timestamp of the previous input or non-finite if no previous input.
        previousMeasureTimestamp;   % Timestamp of the previous measurement or non-finite if no previous measurement.
        originLLA;                  % The current geographic origin for the filter state according to WGS84: latitude [rad], longitude [rad], altitude [m] (positive upwards).
    end
end

