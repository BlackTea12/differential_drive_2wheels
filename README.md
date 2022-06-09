# differential_drive_2wheels
## 1. Summary
2 wheels differential drive tracking and planning testing

## 2. Description
### 2.1 SMC Tracking with distance and heading angle error
File 'differential_drive_2wheels/Matlab/simulink/DifferentialDriveSMCModel' model is based on journal paper below.

![image](https://user-images.githubusercontent.com/41279501/164385048-01ba3c5f-80a2-435b-9b1a-27ea9c9dfb58.png)

- A. Dòria-Cerezo, D. Biel, J. M. Olm and V. Repecho, "Sliding mode control of a differential-drive mobile robot following a path," 2019 18th European Control Conference (ECC), 2019, pp. 4061-4066, doi: 10.23919/ECC.2019.8796166.

Mostly, the model follows the overall equation described in the paper. However, for distance and heading error, the writer chose to use simple calculation based on slope trajectory. Here, the model is tested with 30 degrees slope trajectory with initial position of [xm ,ym, thetam] = [0, 0, 0].

### 2.2 Model Predictive Control (MPC)
File 'differential_drive_2wheels/Matlab/mpc_main' is based on journal paper below.

![image](https://user-images.githubusercontent.com/41279501/172786305-cf9a1827-50f9-4d89-8c87-d0818cb687e6.png)

- Kühne, Felipe et al. “Model Predictive Control of a Mobile Robot Using Linearization.” .

The image shown above shows the overall system flow of how MPC is activated. Based on the equations given in the paper, .m file is written following the system flow.
Here, differential drive model which is non-linear, is linearized using Taylor series expansion and then state x = [x, y, heading angle] and u = [v omega].
