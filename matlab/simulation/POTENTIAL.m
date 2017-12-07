clear
clc

x = -10 :0.1:10;
y = -10: 0.1:10;
xt=5; %121
yt=5; %171
k=0.5;
ox=-3;
oy=-5;
R=1.5;
[X,Y] = meshgrid(x,y);

r = (X-xt).^2+ (Y-yt).^2;

disto=  sqrt((X-ox).^2+ (Y-oy).^2);

beta_l =  1./( 1+exp (-1 * ( disto -(R/2) )*(24/R) ) );

r_k = r.^k ;

phi = (r)./ ((r.^k + beta_l).^(1/k)) ;
surf(X,Y,phi,'LineStyle','none');
