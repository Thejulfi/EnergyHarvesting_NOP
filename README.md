# EnergyHarvesting_NOP

Le dépôt regroupe l'ensemble des codes développé dans le cadre du projet technique réalisé en 5e année de Polytech Nantes en ETN et spécialité SETR. Ces codes ont été réalisés sur une carte MSP430FR5060.

Notons que le préfix "E_" désigne des codes utilisés depuis le framework EnergiaIDE. Les dossiers n'ayant pas de préfix doivent être utilisés avec l'environnement de développement Code Composer Studio.

## Logiciels utilisés



- [Uniflash](https://www.ti.com/tool/UNIFLASH) : Uniflash est un outil graphique autonome utilisé pour programmer la mémoire flash sur puce produites par TI.
- [Code composer studio](https://www.ti.com/tool/CCSTUDIO#overview) : Outils basés sur Éclipse fournissant des outils de développement tel que l'optimisation C/C++, éditeur de codes, debugger et autres fonctionnalités.
- [Energia](https://energia.nu/) : Logiciel open source permettant d'utiliser le framework Arduino avec les microprocesseurs de TI.

## Codes

### UART_Hello_world

Permet d'affcher sur la console de Code composer studio le message "Hello World".

### E_Magneto_I2C_Serial

Code utilisant la librarie Wire.h et LIS3MDL.h afin de réaliser une connexion I2C avec les deux magnétomètres.

### I2C_FRAM_storage_LIS3MDL

### I2C_LIS3MDL

Driver I2C réalisé au niveau 


