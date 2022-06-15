# interaction_controllers
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/mcbed/interaction_controllers/actions/workflows/build.yaml/badge.svg)](https://github.com/mcbed/interaction_controllers/actions/workflows/build.yaml)

This package contains the implementation of the following controllers for `ros2_control` that are used for physical human-robot-environment interaction control.
## Impedance Controller
Impedance Control is a widely used interaction control method, particularly efficient for human-robot interactions. It consists in imposing an impedance model for the relationship between the manipulator and the environment.
## Admittance Controller
Whereas IC requires the system to be controlled in effort, Admittance Control is a control strategy that allows to impose an impedance model generating position commands.  
## Model Predictive Impedance Controller (MPIC)
Model Predictive Impedance Control is a control strategy based on Model Predictive Control (MPC) that allows compliant behavior when interacting with an environment, while respecting practical robotic constraints. A detailed description of the controller algorithm can be found in the following article: 
```
@inproceedings{ Bednarczyk_MPIC_2020,
 	title={Model Predictive Impedance Control},
  	author={Bednarczyk, Maciej and Omran, Hassan and Bayle, Bernard},
	booktitle={2020 IEEE International Conference on Robotics and Automation (ICRA)},
	place={Paris, France},
  	year={2020},
  	month={May},
 	pages={4702-4708}
}
```
## Variable Impedance Controller With Passivity Guarantees (VIC)
The Variable Impedance Control With Passivity Guarantees is a control strategy that allows online modification of impedance parameters while guaranteeing the overall system passivity. A detailed description of the controller algorithm can be found in the following article: 
```
@inproceedings{ Bednarczyk_VIC_2020,
	title={Passivity Filter for Variable Impedance Control},
	author={Bednarczyk, Maciej and Omran, Hassan and Bayle, Bernard},
	booktitle={2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
	place={Las Vegas, USA},
	year={2020},
	month={Oct},
	pages={7159-7164}
}
```
## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Maciej Bednarczyk:__ [m.bednarczyk@unistra.fr](mailto:m.bednarczyk@unistra.fr), @github: [mcbed](mailto:macbednarczyk@gmail.com)
