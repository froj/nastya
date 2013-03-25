/** @file adresses.h
 * @author Antoine Albertelli <a.albertelli@cvra.ch
 * @date 2011
 * @brief Sert de couche d'abstraction pour permettre de renommer les
 * modules hardware sans devoir chercher dans tout le code pour trouver
 * les references.
 */
#ifndef _ADRESSES_H_
#define _ADRESSES_H_


#ifdef COMPILE_ON_ROBOT

/* System.h est un fichier cree par le toolchain qui contient les adresses
 * du hardware. */
#include <system.h>

#define ANALOG_SPI_ADRESS   (int *)(ANALOGIN_BASE)      /**< SPI board address for analog stuff  */
#define DIGITAL_OUTPUT0     (int *)(DIGITALOUT_BASE)    /**< Digital output board address */

/** TODO : Les addresse si on compile sur le robot */
/** TODO : document the naming convetion of the motors */
#define MOTOR0_ADRESS       (int *)(0x0)
#define MOTOR1_ADRESS       (int *)(0x0)
#define MOTOR2_ADRESS       (int *)(0x0)

#else
#define HEXMOTORCONTROLLER_BASE (int *)(0x0)
#define MOTOR0_ADRESS       (int *)(0x0)
#define MOTOR1_ADRESS       (int *)(0x0)
#define MOTOR2_ADRESS       (int *)(0x0)

#define ANALOG_SPI_ADRESS   (int *)(0x0)
#define DIGITAL_OUTPUT0     (int *)(0x0)

#endif
#endif
