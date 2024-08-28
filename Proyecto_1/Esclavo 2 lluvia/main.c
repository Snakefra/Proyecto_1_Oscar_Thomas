/*Universidad del Valle de Guatemala 
 * main.c
 * IE3054 Electronica Digital 2
 *	Proyecto Red de sensores
 * Created: 20/08/2024 13:01:49
 *  Author: Thomas 21626 - Oscar Melchor 
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "Servo/Servo.h"  // Incluir la librería del servo para controlar el servomotor

#define SLAVE_ADDRESS 0x0B // Dirección I2C del esclavo

volatile uint16_t rain_sensor_value = 0;  // Variable para almacenar el valor del sensor de lluvia

// Inicialización del ADC
void adc_init() {
	ADMUX = (1 << REFS0); // Usar Vcc como referencia
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Habilitar ADC y prescaler de 64
}

// Leer un valor analógico del canal ADC especificado
uint16_t adc_read(uint8_t channel) {
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Seleccionar canal
	ADCSRA |= (1 << ADSC); // Iniciar conversión
	while (ADCSRA & (1 << ADSC)); // Esperar a que la conversión termine
	return ADC;
}

// Inicialización de I2C como Esclavo
void i2c_init_slave(uint8_t address) {
	TWAR = address << 1; // Configurar la dirección del esclavo
	TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWINT) | (1<<TWIE); // Habilitar TWI, ACK y habilitar interrupciones
}

// Función para manejar las interrupciones de I2C
ISR(TWI_vect) {
	switch (TWSR & 0xF8) {
		case 0xA8: // Dirección del esclavo con bit de lectura recibida (esclavo transmisor)
		TWDR = (rain_sensor_value >> 8); // Enviar MSB del valor del sensor de lluvia
		TWCR |= (1<<TWINT) | (1<<TWEA);
		break;
		case 0xB8: // Enviar el siguiente byte (LSB)
		TWDR = rain_sensor_value & 0xFF; // Enviar LSB del valor del sensor de lluvia
		TWCR |= (1<<TWINT) | (1<<TWEA);
		break;
		case 0xC0: // Transmisión finalizada
		TWCR |= (1<<TWINT) | (1<<TWEA);
		break;
		default:
		TWCR |= (1<<TWINT) | (1<<TWEA);
		break;
	}
}

int main(void) {
	adc_init(); // Inicializar ADC
	timer1_init(); // Inicializar Timer1 para controlar el servomotor
	i2c_init_slave(SLAVE_ADDRESS); // Inicializar I2C como esclavo
	sei(); // Habilitar interrupciones globales

	while (1) {
		rain_sensor_value = adc_read(1); // Leer valor del sensor de lluvia en A1

		// Control del servo según el valor del sensor
		if (rain_sensor_value < 700) {
			mover_servomotor(90); // Mover el servo a 90 grados si está mojado
			} else if (rain_sensor_value > 1000) {
			mover_servomotor(0); // Mover el servo a 0 grados si no hay lluvia
		}

		_delay_ms(1000); // Esperar 1 segundo antes de la siguiente lectura
	}
}