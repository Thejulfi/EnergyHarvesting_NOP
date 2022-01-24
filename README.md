# EnergyHarvesting_NOP

## Introduction

Le dépôt regroupe l'ensemble des codes développés dans le cadre du projet technique réalisé en 5e année de Polytech Nantes en ETN et spécialité SETR. Ces codes ont été réalisés sur une carte MSP430FR5969.

Dans la première partie de ce document, nous listons les logiciels utilisés pour charger et analyser les codes. Enfin, nous listons et décrivons chaque code.

Notons que le préfix "E_" désigne des codes utilisés depuis le framework EnergiaIDE. Les dossiers n'ayant pas de préfix doivent être utilisés avec l'environnement de développement Code Composer Studio.

## Logiciels utilisés



- [Uniflash](https://www.ti.com/tool/UNIFLASH) : Uniflash est un outil graphique autonome utilisé pour programmer la mémoire flash sur puce produites par TI.
- [Code composer studio](https://www.ti.com/tool/CCSTUDIO#overview) : Outils basés sur Éclipse fournissant des outils de développement tel que l'optimisation C/C++, éditeur de codes, debugger et autres fonctionnalités.
- [Energia](https://energia.nu/) : Logiciel open source permettant d'utiliser le framework Arduino avec les microprocesseurs de TI.

## Codes

### ADC_Shunt_Remake

Code permettant de lire la tension sur le pin 1.2 (ADC). Il est utilisé pour lire la tension image du courant de l'interface shunt.

### UART_Hello_world

Permet d'affcher sur la console de Code composer studio le message "Hello World".

### E_Magneto_I2C_Serial

Code utilisant la librarie Wire.h et LIS3MDL.h afin de réaliser une connexion I2C avec les deux magnétomètres.

### I2C_FRAM_storage_LIS3MDL

### I2C_LIS3MDL

Driver I2C réalisé bas-niveau (configuration des registres). Ce code permet de transférer une configuration aux magnétomètres et de lire, par la suite, les registres chargés de stocker les mesures réalisés par les magnétomètres.

### FRAM_storage

Programme permettant de stocker dans la FRAM la valeur data. Data est un entier qui s'incrémente et est ensuite stocké dans la FRAM entre l'adresse 0x0000 et 0xFFFF. On alloue une mémoire de 1000h dans la FRAM.

### TIMER_INTERRUPT_P1.0

Code exemple permettant d'implémenter une interruption cadencée au rythme d'un timer interne divisé. Le code réalise un blink de la LED0 (pin 1.0) au rythme de 1Hz.
