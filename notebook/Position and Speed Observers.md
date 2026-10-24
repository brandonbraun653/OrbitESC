# Sliding Mode Observer
- [Sensorless Field-Oriented Control: Sliding Mode Observer](https://www.youtube.com/watch?v=lgnGyWdFEKI)
- [Matlab Reference SMO Implementation](https://www.mathworks.com/help/releases/R2025b/mcb/ref/slidingmodeobserver.html)

SMO estimates the rotor position based on the back-emf, which is perfectly synchronized with the rotor. The back-emf signal comes from the rotation of the rotor, which injects current on the stator windings. The difficult part is that we *also* are injecting current with the PWM motor drive circuitry.

How do we extract the back-emf signal from the measured phase currents?

![[Pasted image 20251213105831.png]]

Take va, vb, ia, ib and estimate:

![[Pasted image 20251213105923.png]]

Current Observer: Estimate the currents that should appear after Valpha, Vbeta are applied
![[Pasted image 20251213110005.png]]
![[Pasted image 20251213110810.png]]

Generate an error signal:
![[Pasted image 20251213110146.png]]
![[Pasted image 20251213110838.png]]


The `sign`function acts as the core correction mechanism. It outputs +/-1 depending on the sign of the current error signal. In practice, an ideal sign function isn't used.
![[Pasted image 20251213110223.png]]

Instead, a linear region around zero is used to prevent high frequency chattering:
![[Pasted image 20251213110338.png]]

This has the effect of quickly reducing the error to zero, compared to a PI controller:
![[Pasted image 20251213110534.png]]

The estimated back-emf is then computed as:
![[Pasted image 20251213110914.png]]

And the rotor speed estimate as:
![[Pasted image 20251213110958.png]]