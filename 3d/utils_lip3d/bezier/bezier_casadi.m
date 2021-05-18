function [h,dhds,ddhds] = bezier_casadi(a,s,varargin)
%BEZIER
import casadi.*
%% DOES NOT WORK WHEN S=1 YET
%% 

% Reshape vector into matrix if needed
if any(size(a)==[1,1])
    assert(~isempty(varargin{1}),'Need to specify number of bezier curves.')
    assert(~isempty(varargin{2}),'Need to specify degree of bezier polynomial.')
    a = reshape(a,varargin{1},varargin{2}-1);
end
[N,M] = size(a);
M = M-1;

h = SX.zeros(N,1);
dhds = SX.zeros(N,1);
ddhds = SX.zeros(N,1);

for i=1:N
    for k=0:M 
         h(i) = h(i) + a(i,k+1) * (factorial(M)/(factorial(k)*factorial(M-k))) * (s^k*(1-s)^(M-k));
         dhds(i) = dhds(i) + a(i,k+1) * (factorial(M)/(factorial(k)*factorial(M-k))) * (k*s^(k-1)*(1-s)^(M-k) - (M-k)*s^k*(1-s)^(M-k-1));
         ddhds(i) = ddhds(i) + a(i,k+1) * (factorial(M)/(factorial(k)*factorial(M-k))) * ((k*(k-1)*s^(k-2)*(1-s)^(M-k)-k*(M-k)*s^(k-1)*(1-s)^(M-k-1)) - (k*(M-k)*s^(k-1)*(1-s)^(M-k-1)-(M-k)*(M-k-1)*s^k*(1-s)^(M-k-2)));
    end
end





