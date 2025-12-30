function [sys, mats] = get_system_matrices(conf)
% GET_SYSTEM_MATRICES - Generates State-Space matrices from config

ms = conf.ms; mu = conf.mu;
ks = conf.ks; bs = conf.bs; kt = conf.kt;

% Matrix A (System Matrix)
A = [ 0, 1, 0, -1;
      -ks/ms, -bs/ms, 0, bs/ms;
      0, 0, 0, 1;
      ks/mu, bs/mu, -kt/mu, -bs/mu];

% Matrix B (Input Matrix: Road Vel, Force)
B = [ 0, 0;
      0, 1/ms;
      -1, 0;
      0, -1/mu];

% Matrix C (Output Matrix)
% [Suspension Deflection; Sprung Accel; Tire Deflection]
C = [ 1, 0, 0, 0;
      -ks/ms, -bs/ms, 0, bs/ms;
      0, 0, 1, 0];

% Matrix D (Feedforward Matrix)
D = [ 0, 0;
      0, 1/ms;
      0, 0];

% Return raw matrices in a struct
mats.A = A; mats.B = B; mats.C = C; mats.D = D;

% Return SS object
sys = ss(A, B, C, D);
end