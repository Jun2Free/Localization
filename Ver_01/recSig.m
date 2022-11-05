function [strength] = recSig(posData, transPower, antGain, noisePower)
  % Signal Strength
  sigStrength = transPower + antGain;

  % Recived Power is 
  strength = zeros(10,1);
  for i = 1:10
      % noise
      noise = sqrt(10^(noisePower/10)) * randn(1);
      recPower = sigStrength - posData(i,5) + noise;
      strength(i,1) = recPower;
  end
end