# QDOB - Quasiperiodic Disturbance Observer

This is a control algorithm of the quasiperiodic disturbance observer (QDOB) proposed in the preprint [1]. It is implemented for a position control simulation of a mechanical system 1/(Ms^2) under harmonic disturbances with an outer proportional-derivative position controller in C++.

The QDOB is used to estimate and compensate for a quasi-periodic disturbance, including harmonics. It is two-degree-of-freedom control affecting only disturbance suppression and is usually used with command tracking control. The QDOB has the following characteristics.

1. Wideband harmonic elimination, which is robustness against quasiperiodicity (variations of periodic disturbances)
3. Non-amplification of aperiodic disturbances
4. Non-deviation of harmonic suppression frequencies

When you want to tune the hyperparameters, see the preprint [1].

[1] Hisayoshi Muramatsu, “Quasiperiodic Disturbance Observer for Wideband Harmonic Suppression,” arXiv, arXiv:2406.00362, Jan. 2025.
(https://arxiv.org/abs/2406.00362)

## Licence

[MIT License](https://github.com/HisayoshiMuramatsu/PASF/blob/master/LICENSE) © Hisayoshi Muramatsu
