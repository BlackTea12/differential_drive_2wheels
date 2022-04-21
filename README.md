# differential_drive_2wheels
## 1. Summary
2 wheels differential drive tracking and planning testing

## 2. Description
### 2.1 SMC Tracking with distance and heading angle error
File 'differential_drive_2wheels/Matlab/simulink/DifferentialDriveSMCModel' model is based on journal paper below.

![image](https://user-images.githubusercontent.com/41279501/164385048-01ba3c5f-80a2-435b-9b1a-27ea9c9dfb58.png)

A. Dòria-Cerezo, D. Biel, J. M. Olm and V. Repecho, "Sliding mode control of a differential-drive mobile robot following a path," 2019 18th European Control Conference (ECC), 2019, pp. 4061-4066, doi: 10.23919/ECC.2019.8796166.

Mostly, the model follows the overall equation described in the paper. However, for distance and heading error, the writer chose to use simple calculation based on slope trajectory. Here, the model is tested with 30 degrees slope trajectory with initial position of [xm ,ym, thetam] = [0, 0, 0].
