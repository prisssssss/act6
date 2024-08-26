#include <Arduino.h>

/*JO! variables de 64 bits!! (uso extensivo de memoria)*/
uint64_t Tiempo1, Tiempo2, Tiempo3, Tiempo4, total_ms, total_ms2 = 0;
/* Funcion de configuracion del TIMER0*/
void config_TIMER0(void)
{
  TCCR0A = (1 << WGM01);  // Activa el bit CTC (clear timer on compare match)                        // del TCCR0A (timer counter/control register)
  OCR0A = 62;             // valor de comparacion de int cada 1ms
  TCCR0B = (1 << CS02);   // divido por 256 y generar interrupciones cada 1 ms
  TIMSK0 = (1 << OCIE0A); // Habilita las interrupciones entimer compare
}
int main()
{
  // entradas, no hace falta indicar
  PORTB |= (1 << PB4);                                       // pull-up
  DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5); // salidas
  config_TIMER0();                                           // configuro TIMER0 para efectuar interrupciones
  sei();                                                     // macro que habilita las interrupciones globales
  Tiempo1 = total_ms;
  Tiempo2 = Tiempo1;
  typedef enum Estadoboton
  {
    BT_UP,
    BT_PRESS,
    BT_DOWN,
    BT_REALEASE
  } Estadoboton_t;

  Estadoboton_t estadoBoton = BT_UP;
  uint8_t flag = 0;
  uint8_t numero = 0;

  while (1)
  {
    asm("nop");
    PORTD = (numero << 2);
    switch (estadoBoton)
    {
    case BT_UP:
      // PORTD |= (1 << PD2);
      if (bit_is_clear(PINB, 4)) // boton presionado
      {
        estadoBoton = BT_PRESS;
      }
      else
        estadoBoton = BT_UP;

      break;

    case BT_PRESS:
      // PORTD |= (1 << PD3);
      if (bit_is_set(PINB, 4)) // boton suelto
      {
        estadoBoton = BT_UP;
      }
      if (bit_is_clear(PINB, 4) && (Tiempo2 - Tiempo1) > 40) //Utilizo el delta entre ambos valores y solo si paso un segundo es verdadero
      {
        //_delay_ms(40);

        if (numero < 15 && flag == 0)
        {
          numero++;
        }
        else
        {
          flag = 1;
          if (numero > 0 && flag == 1)
          {
            numero--;
          }
          else
          {
            flag = 0;
            numero = 1;
          }
        }
        estadoBoton = BT_DOWN;
        Tiempo1 = Tiempo2; //"reseteo" el delta para contabilizar otra vez 1000 en este caso
      }
    Tiempo2 = total_ms; //actualizo el valor en ms de Tiempo2 una vez por bucle while

      break;
    case BT_DOWN:
      // PORTD |= (1 << PD4);
      if (bit_is_set(PINB, 4)) // boton suelto
      {
        estadoBoton = BT_REALEASE;
      }
      else
        estadoBoton = BT_DOWN;
      break;
    case BT_REALEASE:
      //_delay_ms(40);
      // PORTD |= (1 << PD5);
      if (bit_is_clear(PINB, 4))// boton presionado
      {
        estadoBoton = BT_DOWN;
      }
      if (bit_is_set(PINB, 4) && (Tiempo4 - Tiempo3) > 40) // boton suelto
      {
        estadoBoton = BT_UP;
         Tiempo3 = Tiempo4;
      }
      Tiempo4 = total_ms2;
      break;
    }
  }
}
ISR(TIMER0_COMPA_vect) //Funcion de invocacion del vector de interrupciones (TIMERO en comparacion en nuestro caso)
{
  //PORTB ^= (1 << PB4); // Descomentar solo para ver el tiempo de interrupcion
  total_ms++;
  total_ms2++;
}