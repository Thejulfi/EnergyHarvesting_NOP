# EnergyHarvesting_NOP

## Introduction

Le dépôt regroupe l'ensemble des codes développés dans le cadre du projet technique réalisé en 5e année de Polytech Nantes en ETN et spécialité SETR. Ces codes ont été réalisés sur une carte MSP430FR5969.

Dans la première partie de ce document, nous listons les logiciels utilisés pour charger et analyser les codes. Enfin, nous listons et décrivons chaque code.

Notons que le préfix "E_" désigne des codes utilisés depuis le framework EnergiaIDE. Les dossiers n'ayant pas de préfix doivent être utilisés avec l'environnement de développement Code Composer Studio.

## Table des matières

[1] - [Codes - ADC_Shunt_Remake](#adc_shunt_remake)  
[2] - [Codes - UART_Hello_world](#uart_hello_world)  
[3] - [Codes - E_Magneto_I2C_Serial](#e_magneto_i2c_serial)  
[4] - [Codes - I2C_FRAM_storage_LIS3MDL](#i2c_fram_storage_lis3mdl)  
[5] - [Codes - I2C_FRAM_manage_Button](#i2c_fram_manage_button)  
[6] - [Codes - I2C_LIS3MDL](#i2c_lis3mdl)  
[7] - [Codes - FRAM_storage](#fram_storage)  
[8] - [Codes - TIMER_INTERRUPT](#timer_interrupt)  
[9] - [Codes - Button_interruption](#button_interruption)
[10] - [Codes - Cout_mesure_ADC](#cout_mesure_adc)
[11] - [Codes - UART_Hellow_world_MSP430](#uart_hellow_world_msp430)
[12] - [Codes - E_EWMA_simple_implementation](#e_ewma_simple_implementation)

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

Utilisation d'un timer RTC générant une interruption réveillant notre système toutes les secondes. À cette fréquence, notre système vient réaliser une mesure du champ magnétique en Z via une interaction I2C et stock ensuite dans la FRAM le résultat de la mesure de courant incluant ce calcul.

### I2C_FRAM_manage_Button

Il s'agit d'une version améliorée de "I2C_FRAM_storage_LIS3MDL". Cette version utilise les deux boutons P1.1 et P4.5 afin de commander l'écrasement ou non de la FRAM et le démarrage ou l'arrêt des mesures.

- Le bouton P1.1 permet de lancer le protocole de mesure (1 mesure par seconde),
- Le bouton P4.5 permet d'effacer les données de la mémoire FRAM.

### I2C_LIS3MDL

Driver I2C réalisé bas-niveau (configuration des registres). Ce code permet de transférer une configuration aux magnétomètres et de lire, par la suite, les registres chargés de stocker les mesures réalisés par les magnétomètres.

### FRAM_storage

Programme permettant de stocker dans la FRAM la valeur data. Data est un entier qui s'incrémente et est ensuite stocké dans la FRAM entre l'adresse 0x0000 et 0xFFFF. On alloue une mémoire de 1000h dans la FRAM.

### TIMER_INTERRUPT

Code exemple permettant d'implémenter une interruption cadencée au rythme d'un timer interne divisé. Le code réalise un blink de la LED0 (pin 1.0) au rythme de 0.5Hz.

### Button_interruption 

Exemple d'une interruption déclenché par un bouton-poussoir relié à la broche P1.1.

### Cout_mesure_ADC


### UART_Hellow_world_MSP430

### E_EWMA_simple_implementation
