function [ x ] = slewRateFilter( x, xDelayed )

Fs = 50;                % Hertz
PosRampingTime = 1.0;   % In seconds
NegRampingTime = 0.5;   % In seconds

maxCurrent = 30;
minCurrent = 20;

maxSlewRate = maxCurrent / ( Fs * PosRampingTime );
minSlewRate = minCurrent / ( Fs * NegRampingTime );

topLim = xDelayed + maxSlewRate;
bottomLim = xDelayed - minSlewRate;

if x > topLim
    x = topLim;
end
if x < bottomLim
    x = bottomLim;
end 

end

