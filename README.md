# QDOB - Quasiperiodic Disturbance Observer

This repository provides a C++ implementation of the quasiperiodic disturbance observer (QDOB), as proposed in [1], for position control of a mechanical system (1/Ms^2) under harmonic disturbances. The QDOB is integrated with an outer proportional-derivative controller in a simulation setting.

The QDOB is designed to estimate and compensate for quasiperiodic disturbances, which include harmonics and waves at surrounding frequencies. It is a two-degree-of-freedom controller that suppresses disturbances without interfering with the tracking control. The QDOB has the following features:

1. Wideband harmonic suppression robust against quasiperiodicity
2. Non-amplification of aperiodic disturbances
3. Non-deviation of harmonic suppression frequencies

See the paper [1] for details.

## Reference

[1] Hisayoshi Muramatsu, “Quasiperiodic Disturbance Observer for Wideband Harmonic Suppression,” IEEE Transactions on Control Systems Technology, 2025. DOI: [10.1109/TCST.2025.3566560]
(https://ieeexplore.ieee.org/document/11006295)

## Licence

[MIT License](https://github.com/HisayoshiMuramatsu/PASF/blob/master/LICENSE) © Hisayoshi Muramatsu
