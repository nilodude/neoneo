clc, clear;
%%y = logspace(0, log10(40), 40);
y = round(logspace(0, log10(30), 40), 2);

yf = flip(y);
b = (1 - yf);
c = b - 2;
