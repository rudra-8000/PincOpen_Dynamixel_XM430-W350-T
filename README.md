# Pinc'Open with a Dynamixel XM430-W350-T
This project is FORK of the amazing [Pinc'Open Project](https://github.com/pollen-robotics/PincOpen/tree/main), only difference being that the design files have been modified to use a **Dynamixel XM430-W350-T** for driving the gripper. Also a few slight design changes like using a 25mm dowel instead of a 24mm one (because of availability).

## Table of contents
- [Pinc'Open with a Dynamixel XM430-W350-T](#pincopen-with-a-dynamixel-xm430-w350-t)
  - [Table of contents](#table-of-contents)
- [About the project](#about-the-project)
- [Build Resources](#build-resources)
  - [BOM (Bill Of Materials)](#bom-bill-of-materials)
  - [STL and SolidWorks Files](#stl-and-solidworks-files)
  - [Configure the motor before assembly and control via python](#configure-the-motor-before-assembly-and-control-via-python)
  - [Assembly Guide](#assembly-guide)







# About the project
Made out of necessity and availability of resources. Also added some extra attachments to allow mounting an Intel **RealSense D435i camera** to act as a wrist mounted camera. The angle of this camera is adjustable.
I've also designed a custom 2 part mount which allows the gripper to be securely mounted to a UR10 (I believe the same can be used for the UR5 and UR3).

# Build Resources
## BOM (Bill Of Materials)
The list of all needed components:

1. Dynamixel XM430-W350-T
2. U2D2
3. U2D2 Power Hub
4. 3D printed parts
5. M3 16mm Dowels x 4
6. M3 25mm Dowels x 4
7. Threaded Inserts M3 * 4.2mm (OD) * 5mm (Length)  x 4
8. M3 Bolts
9. M2.5 Bolts
10. Bushings: 3(ID)* 6(OD) * 5(L)
11. something else which i may have forgotten


## STL and SolidWorks Files
SLDPRT, SLDASM, STL and Steps files can be found [here](https://github.com/rudra-8000/PincOpen_Dynamixel_XM430-W350-T/tree/main/cad)  

I've edited the step files provided by the original creators using SolidWorks.  



## Configure the motor before assembly and control via python
You can use the dynamixel wizard to configure the motor to your needs. We use a U2D2 and a U2D2 Power Hub to interface with the motor.

I've also written a function 
```python
control_gripper(
    angle: float,
    speed: float  = 1.0, #speed scale factor
    torque: float = 1.0, #torque scale factor
    open_angle: float  = 10.0, #in degrees
    close_angle: float = 100.0, #in degrees
    port: str  = DEVICE_PORT,
    baud: int  = BAUD_RATE,
    dxl_id: int = DXL_ID,
)
```
in dynamixel_control.py to allow control of the gripper.

before use, please run the following in your environment:
```bash
pip install dynamixel-sdk
```

## Assembly Guide
Assembly of this version follows the same process as the original PincOpen.