%% BASEADO NO EXEMPLO:
% https://la.mathworks.com/help/nav/ug/pose-estimation-from-asynchronous-sensors.html
% openExample('shared_positioning/PoseEstimationFromAsynchronousSensorsExample')

clear
clc

%% IMPORTAR DADOS .csv
nome = 'LOG24.csv';
LD = dlmread(nome);
clear nome

% ESTRUTURA DO ARQUIVO
% [DateTime_UTC, Satelites, X_gps, Y_gps, Alt_m, Speed_gps, Course_gps,
% hdop, AccX, AccY, AccZ, GyX, GyY, GyZ, MgX, MgY, MgZ, AcelX, AcelY,
% AcelZ, GravX, GravY, GravZ, QtW, QtX, QtY, QtZ, tempo_millis];

% Acc: VECTOR_ACCELEROMETER
% Gy: VECTOR_GYROSCOPE
% Mg: VECTOR_MAGNETOMETER
% Acel: VECTOR_LINEARACCEL
% Grav: VECTOR_GRAVITY
% Qt: QUATERNION

%% SEPARAR O ARQUIVO LD EM VARIAVEI PARA MELHOR COMPREENDER O CODIGO
REFLOC = [LD(1,4), LD(1,3), LD(1,5)]; % REFLOC = [lat, long, alt];
IMUFS = 10; % Maximo MARG rate (IMU) (10 leituras por segundo)
GPSFS = 1; % Maximo GPS rate
ratio = IMUFS/GPSFS;
GPS = [LD(:,1:8)];
ACC = [LD(:,9:11)];
GYRO = [LD(:,12:14)];
MAG = [LD(:,15:17)];
ACEL = [LD(:,18:20)];
GRAV = [LD(:,21:23)];
QTRN = [LD(:,24:27)];
TEMPO = LD(:,28);

%% FUSINO FILTER
% Create an |insfilterAsync| to fuse IMU + GPS measurements. This fusion
% filter uses a continuous-discrete extended Kalman filter (EKF) to track
% orientation (as a quaternion), angular velocity, position, velocity,
% acceleration, sensor biases, and the geomagnetic vector.
%
% This |insfilterAsync| has several methods to process sensor data:
% |fuseaccel|, |fusegyro|, |fusemag| and |fusegps|. Because
% |insfilterAsync| uses a continuous-discrete EKF, the |predict| method can
% step the filter forward an arbitrary amount of time.

INSF = insfilterAsync('ReferenceLocation', REFLOC)

%% INICIALIZAR O VETEOR STATE DO |insfilterAsync|
% The |insfilterAsync| tracks the pose states in a 28-element vector.
% The states are:
%
%       States                          Units    Index
%    Orientation (quaternion parts)             1:4
%    Angular Velocity (XYZ)            rad/s    5:7
%    Position (NED)                    m        8:10
%    Velocity (NED)                    m/s      11:13
%    Acceleration (NED)                m/s^2    14:16
%    Accelerometer Bias (XYZ)          m/s^2    17:19
%    Gyroscope Bias (XYZ)              rad/s    20:22
%    Geomagnetic Field Vector (NED)    uT       23:25
%    Magnetometer Bias (XYZ)           uT       26:28

Nav = 100;
INITSTATE = zeros(28,1);
INITSTATE(1:4) = [mean(QTRN(1:Nav,1));mean(QTRN(1:Nav,2));mean(QTRN(1:Nav,3));mean(QTRN(1:Nav,4))];
INITSTATE(5:7) = [mean(GYRO(1:Nav,1));mean(GYRO(1:Nav,2));mean(GYRO(1:Nav,3))];
INITSTATE(8:10) = [mean(GPS(1:Nav,4));mean(GPS(1:Nav,3));mean(GPS(1:Nav,5))];
INITSTATE(11:13) = [0;0;0];
INITSTATE(14:16) = [mean(ACEL(1:Nav,1));mean(ACEL(1:Nav,2));mean(ACEL(1:Nav,3))]; % PRECISA ROTACIONAR
INITSTATE(17:19) = [0;0;0];
INITSTATE(20:22) = [0;0;0];
INITSTATE(23:25) = [0;0;0];
INITSTATE(26:28) = [0;0;0];

% The gyroscope bias initial value estimate is low for the Z-axis. This is
% done to illustrate the effects of fusing the magnetometer in the
% simulation.
INITSTATE(20:22) = deg2rad([3.125 3.125 3.125]); % Preciso ver qual é o valor correto
INSF.State = INITSTATE;

%% Set the Process Noise Values of the |insfilterAsync|
% The process noise variance describes the uncertainty of the motion model
% the filter uses.
INSF.QuaternionNoise = 1e-2;
INSF.AngularVelocityNoise = 100;
INSF.AccelerationNoise = 100;
INSF.MagnetometerBiasNoise = 1e-7;
INSF.AccelerometerBiasNoise = 1e-7;
INSF.GyroscopeBiasNoise = 1e-7;

%% Define the Measurement Noise Values Used to Fuse Sensor Data
% Each sensor has some noise in the measurements. These values can
% typically be found on a sensor's datasheet.
Rmag = 0.4;
Rvel = 0.01;
Racc = 610;
Rgyro = 0.76e-5;
Rpos = 3.4;

INSF.StateCovariance = diag(1e-3*ones(28,1));

%%
% Preallocate variables for position and orientation. Allocate a variable for indexing into the GPS data.
tam = size(ACC,1);
p = zeros(tam,3);
q = zeros(tam,1,'quaternion');

for ii=1:tam
    INSF.predict(1./IMUFS);
    
    % Fuse Accelerometer
    INSF.fuseaccel(ACC(ii,:), Racc);
    % Fuse Gyroscope
    INSF.fusegyro(GYRO(ii,:), Rgyro);
    % Fuse Magnetometer
    INSF.fusemag(MAG(ii,:), Rmag);
    
    % Fuse GPS
    if ~mod(ii,IMUFS)
        LLA = [GPS(ii,4),GPS(ii,3),GPS(ii,5)];
        GPSVEL = [0,0,0]; % Eu possuo apenas a velocidade linear, preciso calcular essa velocidade em NED
        INSF.fusegps(LLA, Rpos, GPSVEL, Rvel);
    end
    
    % Log the current pose estimate
    [p(ii,:),q(ii)] = pose(INSF);
    
end




















