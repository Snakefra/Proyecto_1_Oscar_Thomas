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

// Dirección I2C del esclavo (puedes cambiarla si es necesario)
#define SLAVE_ADDRESS 0x09

volatile uint16_t humidityValue = 0;

// Inicialización del ADC para leer del pin A1
void adc_init() {
	ADMUX = (1 << REFS0) | (1 << MUX0); // Referencia AVcc, seleccionar A1
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Habilitar ADC, prescaler 64
}

// Lectura del ADC
uint16_t read_adc() {
	ADCSRA |= (1 << ADSC); // Iniciar conversión
	while (ADCSRA & (1 << ADSC)); // Esperar a que termine
	return ADC;
}

// Inicialización del I2C como esclavo
void i2c_init_slave(uint8_t address) {
	// Establecer la dirección del esclavo
	TWAR = (address << 1);
	
	// Habilitar TWI, recibir y transmitir con ACK
	TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
	
	// Habilitar interrupciones globales
	sei();
}

// Interrupción del TWI
ISR(TWI_vect) {
	uint8_t status = TWSR & 0xF8;
	
	switch (status) {
		case 0x60: // Dirección propia recibida con R/W = 0 (escritura por el maestro)
		// Preparar para recibir datos si es necesario
		break;
		
		case 0xA8: // Dirección propia recibida con R/W = 1 (lectura por el maestro)
		// Preparar los datos para enviar al maestro
		// Enviar el valor de humedad dividido en dos bytes (MSB y LSB)
		TWDR = (humidityValue >> 8) & 0xFF; // MSB
		TWCR |= (1 << TWINT) | (1 << TWEA);
		break;
		
		case 0xB8: // Solicitud de más datos por parte del maestro
		// Enviar el LSB
		TWDR = humidityValue & 0xFF; // LSB
		TWCR |= (1 << TWINT) | (1 << TWEA);
		break;
		
		default:
		// Manejar otros estados si es necesario
		TWCR |= (1 << TWINT) | (1 << TWEA);
		break;
	}
}

int main(void) {
	// Inicializar ADC
	adc_init();
	
	// Configurar el pin A0 como salida para el relé
	DDRC |= (1 << DDC0); // Configurar A0 como salida
	PORTC &= ~(1 << PORTC0); // Inicialmente LOW
	
	// Inicializar I2C como esclavo
	i2c_init_slave(SLAVE_ADDRESS);
	
	while (1) {
		// Leer el valor del sensor de humedad
		humidityValue = read_adc();
		
		// Controlar el pin A0 basado en el valor de humedad
		if (humidityValue < 700) {
			PORTC |= (1 << PORTC0); // HIGH (Humedad)
			} else if (humidityValue > 1000) {
			PORTC &= ~(1 << PORTC0); // LOW (Seco)
		}
		
		_delay_ms(1000); // Esperar antes de la siguiente lectura
	}
}