clear
clc
close all

A = [0 3; 2 4];
B = [-2; -1];
rank(ctrb(A,B)); % check controllability
p_desired = 