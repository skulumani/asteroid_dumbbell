% Function to load and reduce the number of faces for the asteroid shape
% models
clear all
clc

% load the asteroid
load castalia.mat

F_full = F;
V_full = V;

[F_2048, V_2048] = reducepatch(F_full,V_full, 2048);
[F_1024, V_1024] = reducepatch(F_full,V_full, 1024);
[F_512, V_512] = reducepatch(F_full,V_full, 512);
[F_256, V_256] = reducepatch(F_full,V_full, 256);
[F_128, V_128] = reducepatch(F_full,V_full, 128);
[F_64, V_64] = reducepatch(F_full,V_full, 64);
[F_32, V_32] = reducepatch(F_full,V_full, 32);

save('castalia_model.mat','F_full','V_full','F','V','F_2048','V_2048','F_1024','V_1024','F_512','V_512','F_256','V_256','F_128','V_128','F_64','V_64','F_32','V_32');