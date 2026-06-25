%% main_quadrotor2D_lab.m
% Minimal 2D quadrotor mini-lab for Lecture 7.
% Run this file. Change only the block below during the lecture.

clear; close all; clc;

p = uav2d_params();

scenario     = 'cascade';   % 'attitude' | 'translation' | 'cascade' is cascade by default
compareGains = true;        % only used for 'cascade'
doAnimation  = true;
doPlots      = true;

switch scenario
    case 'attitude'
        % Stage 1: attitude only. Translation is frozen in the dynamics.
        p.name = 'attitude only';
        p.tEnd = 4;
        p.x0 = [0; 1; 0; 0; -25*pi/180; 0];
        p.ref.theta = 15*pi/180; %the value before *pi seems to be the target angle, defualt 15
        P = p;

    case 'translation'
        % Stage 2: point-mass translation with direct virtual force input.
        % Attitude is ignored: the vehicle is drawn level on purpose.
        p.name = 'translation only';
        p.tEnd = 12;
        p.x0 = [0; 1; 0; 0; 0; 0];
        p.ref.pos = [3; 2];
        P = p;

    case 'cascade'
        % Stage 3: outer loop creates force; atan2 creates theta_d;
        % inner loop tracks theta_d with torque.
        p.tEnd = 8;
        p.x0 = [0; 1; 0; 0; 0; 0];
        p.ref.pos = [3; 2];
        if compareGains
            P = uav2d_gain_cases(p);
        else
            p.name = 'cascade nominal';
            P = p;
        end

    otherwise
        error('Unknown scenario. Use attitude, translation, or cascade.');
end

for i = 1:numel(P)
    sims(i) = simulate_uav2d(scenario, P(i)); %#ok<SAGROW>
end

if doAnimation
    animate_uav2d(sims, scenario);
end

if doPlots
    plot_uav2d_summary(sims, scenario);
end
