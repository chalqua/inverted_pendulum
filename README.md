# Inverted Pendulum
This repository contains the software that controls my inverted pendulum.

## Schematics
![Schematics](./schematics.svg)

## 3D printing

3D STL files are available on [onshape](https://cad.onshape.com/documents/99b23c697db4fbadab78f74f/w/2d03e69731836f0563bc861c/e/28dfe77486ea93adba587849?renderMode=0&uiState=687cd84c7bea8912093d7078).

## Kit list
- Raspberry Pi Zero W V1.1
- 2EA [DC Motor](https://www.lextronic.fr/motoreducteur-e37025gm-060035-76789.html)
- 1EA [Inertial unit](https://www.amazon.fr/modules-MPU-6050-gyroscope-Gyroscope-Convertisseur/dp/B07XRK5FHP/ref=sr_1_1?crid=2YAI468S8FFUK&keywords=mpu6050&qid=1707762342&sprefix=MPU+%2Caps%2C73&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1)
- 1EA [DualShock 4 Wireless Controller for PS4](https://www.amazon.com/DualShock-Wireless-Controller-PlayStation-Black-4/dp/B01LWVX2RG/ref=sr_1_3?crid=3VCD8SNJ5OC9V&dib=eyJ2IjoiMSJ9.mHQUtBWr0GQpXHREK6nnQEUyRpLpvnN86JYjhenJDKbV48k0wxb9eXeOUzyv2C6hn9T7rK1UDBS58P-s-1elOe2bwho5zmyKHl8KX1XMsJIuBGSassvNYHDKVCJYVAizYRj_qrmT899FHdCAkoKw8F_R0QEjiPyb3pfZH0L1V6UXVmwbwY_1jkm5lMH7D08OzeYzKQX0Bv4sIj8BOBHuH0jpfj0sBARS24TXX9jJZKA.LztNaB6LOqICfW5aKPGYqn6nGXZjnlPaZ67Fa8IhqAk&dib_tag=se&keywords=ps4%2Bcontroller&qid=1753014405&sprefix=PS4%2Bcont%2Caps%2C155&sr=8-3&th=1)
- 1EA 27000mAh 7.4 LIPO BATTERY
- 2EA [Step-Down Buck Converter](https://www.amazon.fr/Hailege-Lithium-Battery-Charging-Converter/dp/B07XXSQ327/ref=sr_1_5?__mk_fr_FR=ÅMÅŽÕÑ&crid=8SCSFFI6OPCE&dib=eyJ2IjoiMSJ9.4buCqRudgf8loGGEMWY3wDicZFtQ1-IuGG5LDQR1g8bNy9QyZn1kpUzlyKKtXp-KcQow9VngT7igrp7Ruh6GD6kgL-n9pV8R7CFZ1ViEjByVoN9uNJobswyZcAxDDWCHsHfh3dsiBYsWsnU83GsQK26NMCDmHygTpvQoJq-pp1U4hkCrcztT3_vy1DN8IiheqrkVnUC1llNdhPwYi3ePeuXn9lIz66hasj9xcSPtvcnZ7rqmjrlCO0Nl1cH9FD2kLu-9QmoLW7C_lQCQEB4iXGoe_YLgG9jRno8by5QU2FA.Ohd6FMWHcP3oTBmaMpP-MaoSY3Sb9mYYc1kOiWKx_4Y&dib_tag=se&keywords=XL4015&qid=1728540447&sprefix=xl4015%2Caps%2C109&sr=8-5)
- 1EA [HW-95 Motor Driver](https://www.amazon.fr/xocome-Controller-commande-voiture-intelligent/dp/B094FHW2TY/ref=sxin_23_pa_sp_search_thematic_sspa?__mk_fr_FR=ÅMÅŽÕÑ&content-id=amzn1.sym.b772482c-63e4-4eb4-a1c6-4190add0a415%3Aamzn1.sym.b772482c-63e4-4eb4-a1c6-4190add0a415&crid=2RC7D3ZGZETQU&cv_ct_cx=HW-95+Motor+Driver&keywords=HW-95+Motor+Driver&pd_rd_i=B094FHW2TY&pd_rd_r=dbc74131-e9e3-4c04-9cc4-040acca3f291&pd_rd_w=j0rer&pd_rd_wg=HtYkv&pf_rd_p=b772482c-63e4-4eb4-a1c6-4190add0a415&pf_rd_r=6EEP06CXYYACH2KZ4Q95&qid=1753014788&sbo=RZvfv%2F%2FHxDF%2BO5021pAnSA%3D%3D&sprefix=hw-95+motor+driver%2Caps%2C67&sr=1-3-86ee67e3-2ea6-4725-8419-71cfe38eb657-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9zZWFyY2hfdGhlbWF0aWM&psc=1)

## Python dependencies

Mind that some of the [dependencies](https://stackoverflow.com/questions/7225900/how-can-i-install-packages-using-pip-according-to-the-requirements-txt-file-from) listed in [requirements.txt](./requirements.txt) are not strictly required.

