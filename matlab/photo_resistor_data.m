%% clear all

clc; clear; close all;

%% Photo Resistor data

phores_ambiance = csvread("phores-ambiance.csv");
phores_blocked = csvread("phores-blocked.csv");
phores_white = csvread("phores-white-paper.csv");

figure
hold on;
plot(phores_ambiance)
plot(phores_blocked)
plot(phores_white)
title("photoresistor performance")

legend("ambiance","blocked","white")

disp("These data are much cleaner & thusly don't need as much analysis as the whisker_data.")
