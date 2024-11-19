%% Setup for 2.740 HW 5
clear; clc;

% Add paths
addpath(genpath('./codegen'));
addpath(genpath('./utilities'));

% % Setup CasADi
if ismac
    addpath(genpath("./casadi/casadi_osx"));
elseif isunix
    addpath(genpath("./casadi/casadi_linux"));
elseif ispc
    addpath(genpath("./casadi/casadi_windows"));
end
import casadi.*

% Generate dynamics + load functions for leg
buildBallerinaDynamics();
A_fn = casadi.Function.load('codegen/A_fn.casadi');
b_fn = casadi.Function.load('codegen/b_fn.casadi');
energy_fn = casadi.Function.load('codegen/energy_fn.casadi');
pos_end_effector = casadi.Function.load('codegen/pos_end_effector.casadi');
vel_end_effector = casadi.Function.load('codegen/vel_end_effector.casadi');
J_end_effector = casadi.Function.load('codegen/J_end_effector.casadi');
keypoints_fn = casadi.Function.load('codegen/keypoints_fn.casadi');

% Parameters for leg
m1 =.0393 + .2;         m2 =.0368; 
m3 = .00783;            m4 = .200;
I1 = 25.1 * 10^-6;      I2 = 53.5 * 10^-6;
I3 = 9.25 * 10^-6;      I4 = 2.86141935483871 * 10^-4;
I5 = 1.086 * 10^-4; 
l_OA=.011;              l_OB=.042; 
l_AC=.096;              l_DE=.091;
l_O_m1=0.032;           l_B_m2=0.0344; 
l_A_m3=0.0622;          l_C_m4=0.0610;
N = 18.75;
Ir = 0.0035/N^2;
g = 9.81;    

%% Parameter vector
params   = [m1 m2 m3 m4 I1 I2 I3 I4 I5 Ir N l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_OA l_OB l_AC l_DE g]';
q_min_val = [-pi/2; 0.3];
q_max_val = [pi/3; 0.8*pi];

tau_max_val = 1;
tau_min_val = -1;
% tau_max_val = 0.05;
% tau_min_val = -0.05;