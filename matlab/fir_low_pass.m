function y = firfun(u)
%#codegen
    persistent lowPassFIR;
    if isempty(lowPassFIR)
        N   = 15;         % FIR filter order
        Fp  = 5e3;        % 5 kHz passband-edge frequency
        Fs  = 10e3;       % 10 kHz sampling frequency
        Rp  = 0.00057565; % Corresponds to 0.01 dB peak-to-peak ripple
        Rst = 1e-4;       % Corresponds to 80 dB stopband attenuation

        eqnum = firceqrip(N,Fp/(Fs/2),[Rp Rst],'passedge'); % eqnum = vec of coeffs
        lowPassFIR = dsp.FIRFilter('Numerator', eqnum);
    end

    y = step(lowPassFIR, u);
end

