function [strength] = recSig(posData, transPower, antGain, noisePower, indexK)

  % RSSI = Transmit Power(상수) + antenna gain(상수) - path loss + noise
      %{
      noise = sqrt(10^(noisePower/10)) * randn(1);  % dB to linear
      recPower = sigStrength / posData(indexK,4) + noise;
      strength = recPower;
      %}
     
    r_sig = transPower + antGain - posData(indexK,4);
    r_sig_lin = sqrt(10.^(r_sig/10));
    w_noise = sqrt(10^(noisePower/10))*randn(1);
    r_sig_n = r_sig_lin + w_noise;
    
    SNR = r_sig - noisePower;
    strength = r_sig_n;

  %{
  r_sig = transPower + antGain - posData(indexK,4);
  SNR = r_sig - noisePower; % SNR [dB]
  strength = SNR;
  %}
end