# Inverted Pendulum
This repository contains the software that controls my inverted pendulum.

## Schematics

Caveat - only the first driver and motor are represented on the drawing.
In my setup, I connected both drivers to the exact same GPIO ports.

![Schematics](./schematics.svg)

## 3D printing

3D STL files are available on [onshape](https://cad.onshape.com/documents/99b23c697db4fbadab78f74f/w/2d03e69731836f0563bc861c/e/28dfe77486ea93adba587849?renderMode=0&uiState=687cd84c7bea8912093d7078).

## Kit list

- Raspberry Pi 3 Model B
- 2EA [Stepper Motor](https://www.amazon.fr/17Hs4023-Moteur-Fournitures-Électriques-Imprimante/dp/B08DCGHLYP/ref=sr_1_1?__mk_fr_FR=ÅMÅŽÕÑ&crid=12HM7EQGLOSRE&keywords=17HS4023+Stepper+Motor%2C+0.7A+12V+Nema+17+Stepper+Motor+4+Cable+Eletrical+Supplies+for+3D+Printer&qid=1707743697&sprefix=17hs4023+stepper+motor%2C+0.7a+12v+nema+17+stepper+motor+4+cable+eletrical+supplies+for+3d+printer%2Caps%2C99&sr=8-1)
- 2EA [DRV8825 Stepper Motor Driver](https://www.amazon.fr/AZDelivery-DRV8825-dissipateur-thermique-Imprimante/dp/B07YWV6W4W/ref=sr_1_2?__mk_fr_FR=ÅMÅŽÕÑ&crid=11C1Y9V4HGTJF&keywords=DRV8825&qid=1707742369&sprefix=drv8825%2Caps%2C71&sr=8-2-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1)
- 1EA [Accelerometer Gyroscope](https://www.amazon.fr/modules-MPU-6050-gyroscope-Gyroscope-Convertisseur/dp/B07XRK5FHP/ref=sr_1_1?crid=2YAI468S8FFUK&keywords=mpu6050&qid=1707762342&sprefix=MPU+%2Caps%2C73&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1)
- 1EA 12 VDC power generator


## Python dependencies

Mind that some of the [dependencies](https://stackoverflow.com/questions/7225900/how-can-i-install-packages-using-pip-according-to-the-requirements-txt-file-from) listed in [requirements.txt](./requirements.txt) are not strictly required.

## Acknowledgments

Driver handling in [AngularVelocityController.py](./AngularVelocityController.py) comes from https://github.com/gavinlyonsrepo/RpiMotorLib
