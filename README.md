# Aquaculture Worlds


## Environment and Requirements

Deep sea fish farming provides protective floating cages to facilitate healthy and environmentally sustainable fish habitats. Regular maintenance is required to clean surfaces of growth, remove dead fish, and perform miscellaneous tasks. Regular monitoring and inspection is required to track fish health, environmental conditions, and human intervention level alerts.

Energy must be generated on-site; sources include wave, solar, and wind.

Communication between on-site and land is strained and energetically costly. Underwater data communication is unreliable and hard wire is necessary.

## Solution

A two vehicle design is proposed to achieve the desired goal. A surface and underwater pair tethered by a energy/data chord. 

### ASV - Autonomous Surface Vessel

- Generate energy for self and AUV
- Navigate cooperatively with AUV
- Provide surface level sensing and awareness
- Communicate with land

Primary energy generation comes from wave energy conversion. Large plates under the surface of the water counteracts the forces of the buoyant body of the ASV. Additional energy harvesting from solar and wind have been proposed.

#### Structure and Motion Model

| Constant | Value | Units | Notes |
| -------- | ----- | ----- | ----- |
| mass | 180 | kg | taken from WAM-V |
| inertial moments | 120, 0, 0; 0, 400, 0; 0, 0, 420 | kg m ² |  taken from WAM-V |
| length | 3 | m ||
| beam (width) | 1.5 | m | |
| draft (height underwater) | 1 | m | constant during navigation, variable during wave energy harvesting |
| clearence (height above water) | 2 | m | constant during navigation, variable during wave energy harvesting |
| starbord thruster position | -0.6, 0.6, -0.25 | m ||
| port thruster position | -0.6, -0.6, -0.25 | m ||
| max thruster angle | π/2 | radians | taken from WAM-V |
| max forward thruster force | 250 | N | taken from WAM-V |
| max reverse thruster force | -100 | N | taken from WAM-V |

#### Sensor Model

| Constant | Value | Units | Notes |
| -------- | ----- | ----- | ----- |
| starbord lidar position | 0.24, 0.65, 0.24 | m ||
| port lidar position | 0.24, -0.65, 0.24 | m ||
| fore camera position | 0.6, 0, 0.24 | m ||
| aft camera position | -0.6, 0, 0.24 | m ||
| starbord camera position | 0, 0.6, 0.24 | m ||
| port camera position | 0, -0.6, 0.24 | m ||
| gps position | 0, 0, 2 | m ||
| imu position | 0.1, 0.6, 0.24 | m ||

### AUV - Autonomous Underwater Vessle

- Perform operational goals
- Navigate cooperatively with ASV
- Commuicate data and metrics to ASV



Clone into catkin workspace, catkin make, and launch world. Requires uuv_simulator (https://github.com/ivanacollg/uuv_simulator) and vrx


![Floating by cage](FloatingByCage.png)
![Screenshot from 2022-07-14 10-59-41](https://user-images.githubusercontent.com/22798343/179013512-5792b9db-6403-4bab-b9af-3b5dfe56ec6b.png)
![Screenshot from 2022-07-14 11-02-23](https://user-images.githubusercontent.com/22798343/179014296-f65e1cf9-aee1-434d-b6ef-10b41a258fea.png)
![Screenshot from 2022-07-14 11-03-46](https://user-images.githubusercontent.com/22798343/179014301-93876fe3-cc91-43f2-b2e7-af68f3f5eae5.png)
![Screenshot from 2022-07-14 11-31-45](https://user-images.githubusercontent.com/22798343/179020223-8897eb67-8bc6-43ce-a320-318b532f7c47.png)
![Screenshot from 2022-07-14 11-41-36](https://user-images.githubusercontent.com/22798343/179022472-acf9dc01-1933-4208-be5d-e16877d53927.png)
![Screenshot from 2022-07-14 11-42-10](https://user-images.githubusercontent.com/22798343/179022475-1328f607-7d3e-4757-b352-1fbeea6c9353.png)
![Screenshot from 2022-07-14 15-11-07](https://user-images.githubusercontent.com/22798343/179064491-f7cd1dd9-e194-45f2-938f-f1e432f0ff9c.png)
![Screenshot from 2022-07-14 15-11-35](https://user-images.githubusercontent.com/22798343/179064493-dfe6f2b5-f8e6-44c8-bf7d-a07b458177e4.png)
![Screenshot from 2022-07-14 15-11-37](https://user-images.githubusercontent.com/22798343/179064494-ef332c4c-3a9f-4c53-91dc-2f9950e06533.png)
