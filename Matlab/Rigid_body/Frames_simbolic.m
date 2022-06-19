clear; close; clc;

syms('th1')
syms('d1')
syms('th2')
syms('d2')

H =  SE3.Ry(th1)*SE3(0,0,-d1)*SE3.Rx(th2)*SE3(0,0,d2)
