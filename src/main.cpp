#include <Arduino.h>

#include <avr/io.h>       // definições do componente especificado
#include <avr/pgmspace.h> // para o uso do PROGMEM, gravação de dados na memória flash
#include <util/delay.h> // biblioteca para o uso das rotinas de _delay_ms e _delay_us()

#define F_CPU 16000000UL // define a frequencia do microcontrolador - 16MHz
/**
 * É o registrador onde estão conectado os pinos DB4 - DB7 do LCD
 */
#define DADOS_LCD PORTD
#define DATA_DIR_REG_LINHAS_LCD DDRD
#define LCD_DB7 PD7
#define LCD_DB6 PD6
#define LCD_DB5 PD5
#define LCD_DB4 PD4
/**
 * Essa flag indica se os pinos que recebem dados no LCD são os 4 LSB (Px0-D4
 * a Px3-D7) ou os 4 MSB (Px4-D4 a Px7-D7).

 * Obs.: 1 nibble é meio byte (ou 4 bits).
 */
#define LCD_DATA_NIBBLE 1
/**
 * Registrador com os pinos de controle do LCD (pino R/W e Enable).
 */
#define CONTR_LCD PORTB
/**
 * Registrador de direção de dados dos pinos de controle do LCD
 */
#define DATA_DIR_REG_CONTR_LCD DDRB
/**
 * Pino de habilitação do LCD (enable)
 */
#define LCD_ENABLE PB1
/**
 * Pino que informar se o dado é uma instrução ou caractere
 * - 0: indica que é uma instrução;
 * - 1: indica que é um caractere.
 */
#define LCD_DATA_TYPE_PIN PB0

#pragma region macros

/// Coloca em 1 o bit x da variável Y
#define set_bit(y, bit) (y |= (1 << bit))
/// Coloca em 0 o bit x da variável Y
#define reset_bit(y, bit) (y &= ~(1 << bit))
/// Troca o estado lógico do bit x da variável Y
#define toggle_bit(y, bit) (y ^= (1 << bit))
/// Retorna 0 ou 1 conforme leitura do bit
#define get_bit(y, bit) (y & (1 << bit))

#if (LCD_DATA_NIBBLE == 1)
#define set_most_significant_nibble(data)                                      \
  DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & data);

#define set_less_significant_nibble(data)                                      \
  DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & (data << 4));
#else
#define set_most_significant_nibble(trimmed_data_nibble)                       \
  DADOS_LCD = (DADOS_LCD & 0xF0) | (data >> 4);

#define set_less_significant_nibble(data)                                      \
  DADOS_LCD = (DADOS_LCD & 0xF0) | (0x0F & data);
#endif

#pragma endregion
#pragma region classes definitions

class Lcd
{
public:
  enum class MessageType
  {
    Instruction,
    Character
  };

  enum class Line
  {
    Upper,
    Lower
  };

  static void setup();
  static void send_command(const uint8_t command_byte);
  static void send_character(const uint8_t character_byte);
  static void send_string(const String string);
  static void initialize_lcd();
  static void clear();
  static void turn_cursor_on();
  static void turn_cursor_off();
  /**
   * Move o cursor para a linha `line`, a `offset` colunas após a
   * primeira coluna.
   */
  static void move_cursor(const Line line, const uint8_t offset);

private:
  static void send_message(const uint8_t data, const MessageType msg_type);
  static void set_message_data_type_to_pin(const MessageType msg_type);
  static void set_data_to_lcd(const uint8_t data);
  static void enable_pulse();
  static void register_custom_characters();
};

#pragma once

#include <stdint.h>

class PinManager
{
private:
  const uint8_t pin_;
  volatile uint8_t *const port_input_reg_;
  volatile uint8_t *const port_data_reg_;
  volatile uint8_t *const data_dir_reg_;

public:
  enum DataDirection
  {
    Input = false,
    Output = true,
  };

  PinManager(uint8_t pin, volatile uint8_t *port_input_reg,
             volatile uint8_t *port_data_reg, volatile uint8_t *data_dir_reg)
      : pin_(pin), port_input_reg_(port_input_reg),
        port_data_reg_(port_data_reg), data_dir_reg_(data_dir_reg)
  {
  }

  bool get_digital_level() const
  {
    return get_bit(*this->port_input_reg_, this->pin_) != 0;
  }

  void set_digital_level(bool digital_level) const
  {
    digital_level ? set_bit(*this->port_data_reg_, this->pin_)
                  : reset_bit(*this->port_data_reg_, this->pin_);
  }

  void set_data_direction(DataDirection direction) const
  {
    direction ? set_bit(*this->data_dir_reg_, this->pin_)
              : reset_bit(*this->data_dir_reg_, this->pin_);
  }
};

class Button
{
private:
  const PinManager pin_manager;
  // Inicializa ambos como não pressionados.
  // Lembre-se que os botões, quando pressionados, ficam no estado
  // lógico/digital `LOW` (`false`).
  bool previous_state = true;
  bool current_state = true;

public:
  Button(PinManager pin_manager) : pin_manager(pin_manager) {}
  /**
   * Estado de tempo utilizando pra aplicar o debounce em cada botão
   */
  unsigned long time_state = 0;
  bool has_changed() const
  {
    return this->current_state != this->previous_state;
  }
  /**
   * Se, e somente se, `has_changed() == true`, este método retorna:
   * - `true` se o botão foi pressionado;
   * - `false` se o botão foi solto.
   *
   * Caso contrário, o resultado desta função não tem significado.
   */
  bool has_been_pressed() const { return !this->current_state; }
  /**
   * Checa o nível digital do botão para descobrir se houve interação.
   */
  void check()
  {
    this->previous_state = this->current_state;
    this->current_state = this->pin_manager.get_digital_level();
  }

  void setup()
  {
    this->pin_manager.set_data_direction(PinManager::Input);
    // ativa o registor pull-up
    this->pin_manager.set_digital_level(true);
  }
};

#pragma endregion
#pragma region global vars & configs

#pragma endregion
#pragma region main

auto power_btn = Button(PinManager(PC2, &PINC, &PORTC, &DDRC));
auto configuration_btn = Button(PinManager(PC3, &PINC, &PORTC, &DDRC));
auto increment_btn = Button(PinManager(PC4, &PINC, &PORTC, &DDRC));
auto decrement_btn = Button(PinManager(PC5, &PINC, &PORTC, &DDRC));

void setup()
{
  Lcd::setup();
  Lcd::turn_cursor_off();
  Lcd::send_string(String("Time        Temp"));
  Lcd::move_cursor(Lcd::Line::Lower, 0);
  // A oitava posição de caracteres customizados na verdade espelha o caractere
  // 0. Isso permite que imprimamos o símbolo de grau Celsius diretamente na
  // string. Observe que colocar \0 não funciona devido a esse ser o caractere
  // nulo das strings em C, então o truque do x8 serve para espelhar o
  // caractere 0.
  Lcd::send_string(String("01:17        10\x8"));

  power_btn.setup();
  configuration_btn.setup();
  increment_btn.setup();
  decrement_btn.setup();

  set_bit(DDRD, PD2);
}

void loop()
{
  static const uint8_t buttons_rate_limit = 15;
  auto now = millis();

  auto toggle_led = false;

  if (now - power_btn.time_state > buttons_rate_limit)
  {
    power_btn.check();
    power_btn.time_state = now;
    if (power_btn.has_changed() && power_btn.has_been_pressed())
    {
      toggle_led = true;
    }
  }

  if (now - configuration_btn.time_state > buttons_rate_limit)
  {
    configuration_btn.check();
    configuration_btn.time_state = now;
    if (configuration_btn.has_changed() && configuration_btn.has_been_pressed())
    {
      toggle_led = true;
    }
  }

  if (now - increment_btn.time_state > buttons_rate_limit)
  {
    increment_btn.check();
    increment_btn.time_state = now;
    if (increment_btn.has_changed() && increment_btn.has_been_pressed())
    {
      toggle_led = true;
    }
  }

  if (now - decrement_btn.time_state > buttons_rate_limit)
  {
    decrement_btn.check();
    decrement_btn.time_state = now;
    if (decrement_btn.has_changed() && decrement_btn.has_been_pressed())
    {
      toggle_led = true;
    }
  }

  if (toggle_led) toggle_bit(PORTD, PD2);
}

#pragma endregion
#pragma region classes methods implementations

void Lcd::enable_pulse()
{
  set_bit(CONTR_LCD, LCD_ENABLE);
  _delay_us(1);
  reset_bit(CONTR_LCD, LCD_ENABLE);
  _delay_us(1);
  _delay_us(45);
}

void Lcd::set_data_to_lcd(const uint8_t data)
{
  set_most_significant_nibble(data);
  enable_pulse();
  set_less_significant_nibble(data);
  enable_pulse();
}

void Lcd::set_message_data_type_to_pin(const MessageType msg_type)
{
  switch (msg_type)
  {
  case MessageType::Instruction:
    reset_bit(CONTR_LCD, LCD_DATA_TYPE_PIN);
    break;
  case MessageType::Character:
    set_bit(CONTR_LCD, LCD_DATA_TYPE_PIN);
    break;
  }
}

void Lcd::send_message(const uint8_t data, const MessageType msg_type)
{
  set_message_data_type_to_pin(msg_type);
  set_data_to_lcd(data);

  const auto is_return_or_clean_instruction =
      msg_type == MessageType::Instruction && data < 4;

  if (is_return_or_clean_instruction) _delay_ms(2);
}

void Lcd::send_command(const uint8_t command_byte)
{
  send_message(command_byte, MessageType::Instruction);
}

void Lcd::send_character(const uint8_t character_byte)
{
  send_message(character_byte, MessageType::Character);
}

void Lcd::initialize_lcd()
{
  reset_bit(CONTR_LCD, LCD_DATA_TYPE_PIN);
  reset_bit(CONTR_LCD, LCD_ENABLE);

  _delay_ms(20);
  send_message(0x30, MessageType::Instruction);
  enable_pulse();
  _delay_ms(5);
  enable_pulse();
  _delay_ms(200);
  enable_pulse();
  send_command(0x20);
  enable_pulse();
  send_command(0x28);
  send_command(0x08);
  send_command(0x01);
  send_command(0x0F);
  send_command(0x80);
}

void Lcd::send_string(const String string)
{
  for (auto character : string)
  {
    send_character(character);
  }
}

void Lcd::clear() { send_command(0x01); }
void Lcd::turn_cursor_on() { send_command(0x0F); }
void Lcd::turn_cursor_off() { send_command(0x0C); }

void Lcd::move_cursor(const Line line, const uint8_t offset)
{
  switch (line)
  {
  case Line::Upper:
    return send_command(0x80 + offset);
  case Line::Lower:
    return send_command(0xC0 + offset);
  }
}

void Lcd::register_custom_characters()
{
  const uint8_t custom_chars_size = 1;
  const uint8_t custom_chars[custom_chars_size][8] = {
      // °
      {0b01100, 0b10010, 0b10010, 0b01100, 0b00000, 0b00000, 0b00000, 0b00000},
  };

  const uint8_t lcd_cgram_base_addr = 0x40;
  for (size_t i = 0; i < custom_chars_size; i++)
  {
    // posiciona o cursor no endereço onde o caractere vai ser gravado na
    // memória do LCD (cada caractere customizado tem um tamanho de 8 bytes)
    const auto char_byte_offset = i * 8;
    send_command(lcd_cgram_base_addr + char_byte_offset);

    // registra cada um dos 8 bytes desse caractere customizado
    const auto custom_character = custom_chars[i];
    for (size_t j = 0; j < 8; j++)
    {
      const auto custom_char_byte = custom_character[j];
      send_character(custom_char_byte);
    }
  }

  move_cursor(Line::Upper, 0);
}

void Lcd::setup()
{
  // Torna saídas todos os pinos das linhas do LCD
  set_bit(DATA_DIR_REG_LINHAS_LCD, LCD_DB7);
  set_bit(DATA_DIR_REG_LINHAS_LCD, LCD_DB6);
  set_bit(DATA_DIR_REG_LINHAS_LCD, LCD_DB5);
  set_bit(DATA_DIR_REG_LINHAS_LCD, LCD_DB4);
  // Torna saídas os pinos de controle do LCD
  set_bit(DATA_DIR_REG_CONTR_LCD, LCD_ENABLE);
  set_bit(DATA_DIR_REG_CONTR_LCD, LCD_DATA_TYPE_PIN);

  initialize_lcd();
  register_custom_characters();
}

#pragma endregion
