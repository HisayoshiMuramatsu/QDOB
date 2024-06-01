# QDOB - Quasi-periodic Disturbance Observer

This is a control algorithm of the quasi-periodic disturbance observer (QDOB) proposed in [1]. It is implemented for a position control simulation of a mechanical system 1/(Ms^2) under harmonic disturbances with an outer proportional-derivative position control in C++.

The QDOB is used to estimate and compensate for a quasi-periodic disturbance, including harmonics. It is two-degree-of-freedom control affecting only disturbance suppression and is usually used with command tracking control. The QDOB has the following three characteristics.

1. Wideband harmonic elimination
2. Robustness against variations (quasi-periodicity) of a disturbance
3. Non-amplification of aperiodic disturbances and non-deviation of harmonic suppression frequencies

When you want to tune the hyperparameters, see the paper [1].

[1] Hisayoshi Muramatsu, “Periodic/Aperiodic Motion Control Using Periodic/Aperiodic Separation Filter,” IEEE Transactions on Industrial Electronics, vol. 67, no. 9, pp. 7649-7658, Sep. 2020.
(https://ieeexplore.ieee.org/abstract/document/8858034)

## Licence

[MIT License](https://github.com/HisayoshiMuramatsu/PASF/blob/master/LICENSE) © Hisayoshi Muramatsu
