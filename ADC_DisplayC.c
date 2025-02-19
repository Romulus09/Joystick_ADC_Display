/*
 * Por: Wilton Lacerda Silva
 *    Interface Homem-Máquina (IHM) com o Display OLED
 * 
 * Este programa utiliza o display OLED SSD1306 com resolução de 128x64 pixels
 * e o microcontrolador RP2040 (Raspberry Pi Pico) para exibir informações
 * do conversor analógico-digital (ADC) e do Joystick.
 * Também mostra a leitura dos botões do Joystick e do botão A.
 * 
 * 
*/

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h" 
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define JOYSTICK_X_PIN 26  // GPIO para eixo X
#define JOYSTICK_Y_PIN 27  // GPIO para eixo Y
#define JOYSTICK_PB 22 // GPIO para botão do Joystick
#define LED_G 11
#define LED_B 12
#define LED_R 13
#define Botao_A 5 // GPIO para botão A

const uint16_t WRAP_PERIOD = 2048; //valor máximo do contador - WRAP
const float PWM_DIVISER = 1; //divisor do clock para o PWM
bool led_activate = true;
bool borda = false; 

static volatile uint32_t last_time = 0; //para debouncing

//config pwm led azul
void pwm_setup_blue()
{
    gpio_set_function(LED_B, GPIO_FUNC_PWM); //habilitar o pino GPIO como PWM

    uint slice = pwm_gpio_to_slice_num(LED_B); //obter o canal PWM da GPIO

    pwm_set_clkdiv(slice, PWM_DIVISER); //define o divisor de clock do PWM

    pwm_set_wrap(slice, WRAP_PERIOD); //definir o valor de wrap

    pwm_set_enabled(slice, true); //habilita o pwm no slice correspondente
}

//config pwm led vermelho
void pwm_setup_red()
{
    gpio_set_function(LED_R, GPIO_FUNC_PWM); //habilitar o pino GPIO como PWM

    uint slice = pwm_gpio_to_slice_num(LED_R); //obter o canal PWM da GPIO

    pwm_set_clkdiv(slice, PWM_DIVISER); //define o divisor de clock do PWM

    pwm_set_wrap(slice, WRAP_PERIOD); //definir o valor de wrap
    
    pwm_set_enabled(slice, true); //habilita o pwm no slice correspondente
}

//funcoes para acender os leds com modulação pwm
void pwm_change_blue(uint16_t analog_input){

if(led_activate)
{
uint16_t blue_level = (analog_input > 2048)?(analog_input - 2048):(2048 - analog_input);
pwm_set_gpio_level(LED_B,blue_level);
}

}

void pwm_change_red(uint16_t analog_input){

if(led_activate)
{
uint16_t red_level = (analog_input > 2048)?(analog_input - 2048):(2048 - analog_input);
pwm_set_gpio_level(LED_R,red_level);
}

}


//interrupção dos buttons
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
  // Obtém o tempo atual em microssegundos
  uint32_t current_time = to_us_since_boot(get_absolute_time());
  
  // Verifica se passou tempo suficiente desde o último evento
  if (current_time - last_time > 400000) // 400 ms de debouncing
  {
    last_time = current_time;

    if(gpio == Botao_A)
    {
      led_activate = !led_activate;// ativa/desativa mudanças pwm nos pinos
    }

    else if(gpio == JOYSTICK_PB)
    {
      gpio_put(LED_G,!gpio_get(LED_G)); //alterna estado do led verde
      borda = !borda;//alterna o estado da borda 

    }

    if(gpio == botaoB)
    {
      reset_usb_boot(0, 0);//bootsel
    }
  }
  
}

int main()
{
  
  stdio_init_all();

  gpio_init(LED_R);
  gpio_init(LED_G);
  gpio_init(LED_B);
  gpio_set_dir(LED_R, GPIO_OUT);
  gpio_set_dir(LED_G, GPIO_OUT);
  gpio_set_dir(LED_B, GPIO_OUT);

  gpio_init(botaoB);
  gpio_set_dir(botaoB, GPIO_IN);
  gpio_pull_up(botaoB);

  gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(Botao_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
  gpio_set_irq_enabled_with_callback(JOYSTICK_PB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

  gpio_init(JOYSTICK_PB);
  gpio_set_dir(JOYSTICK_PB, GPIO_IN);
  gpio_pull_up(JOYSTICK_PB); 

  gpio_init(Botao_A);
  gpio_set_dir(Botao_A, GPIO_IN);
  gpio_pull_up(Botao_A);

  pwm_setup_red();
  pwm_setup_blue();

  // I2C Initialisation. Using it at 400Khz.
  i2c_init(I2C_PORT, 400 * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_pull_up(I2C_SDA); // Pull up the data line
  gpio_pull_up(I2C_SCL); // Pull up the clock line
  ssd1306_t ssd; // Inicializa a estrutura do display
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd); // Configura o display
  ssd1306_send_data(&ssd); // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);

  adc_init();
  adc_gpio_init(JOYSTICK_X_PIN);
  adc_gpio_init(JOYSTICK_Y_PIN);  
  


  uint16_t adc_value_x;
  uint16_t adc_value_y;  
  int x_cord;
  int y_cord;
  char str_x[5];  // Buffer para armazenar a string
  char str_y[5];  // Buffer para armazenar a string  
  
  bool cor = true;
  
  while (true)
  {
    adc_select_input(0); // Seleciona o ADC para eixo X. O pino 26 como entrada analógica
    adc_value_y = adc_read();
    adc_select_input(1); // Seleciona o ADC para eixo Y. O pino 27 como entrada analógica
    adc_value_x = adc_read();    
    sprintf(str_x, "%d", adc_value_x);  // Converte o inteiro em string
    sprintf(str_y, "%d", adc_value_y);  // Converte o inteiro em string
    printf("valor x : %d\n", adc_value_x);
    printf("valor y : %d\n", adc_value_y);

    pwm_change_blue(adc_value_y);
    pwm_change_red(adc_value_x);
    
    //conversão para coordenadas do display (128x64, subtraindo os 8 pixels do lado do quadrado)
    x_cord = (adc_value_x * 120)/4095; //converte o valor adc para coordenada x válida no display 
    y_cord = (adc_value_y * 56)/4095;  //converte o valor adc para coordenada y válida no display
    y_cord = (y_cord > 28)?(28 - (y_cord - 28)):(28 + (28 - y_cord)); //trata o fato da coordenada y aumentar de cima pra baixo no display

   
    
    ssd1306_fill(&ssd, !cor); // Limpa o display
    ssd1306_rect(&ssd, y_cord, x_cord, 8, 8, cor, cor); //posiciona o quadrado na tela de acordo com as coordenadas obtidas

    
    ssd1306_rect(&ssd, 3, 3, 122, 58, borda, false); // Desenha a borda

    ssd1306_send_data(&ssd); 

    


    sleep_ms(100);
  }
}