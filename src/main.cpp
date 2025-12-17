#include <Arduino.h>

#include <avr/io.h>       // definições do componente especificado
#include <avr/pgmspace.h> // para o uso do PROGMEM, gravação de dados na memória flash
#include <stdint.h>
#include <util/delay.h> // biblioteca para o uso das rotinas de _delay_ms e _delay_us()

#pragma region defs

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

#pragma endregion
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
#define set_most_significant_nibble(data) DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & data);

#define set_less_significant_nibble(data) DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & (data << 4));
#else
#define set_most_significant_nibble(trimmed_data_nibble)                                           \
  DADOS_LCD = (DADOS_LCD & 0xF0) | (data >> 4);

#define set_less_significant_nibble(data) DADOS_LCD = (DADOS_LCD & 0xF0) | (0x0F & data);
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

  PinManager(uint8_t pin, volatile uint8_t *port_input_reg, volatile uint8_t *port_data_reg,
             volatile uint8_t *data_dir_reg)
      : pin_(pin), port_input_reg_(port_input_reg), port_data_reg_(port_data_reg),
        data_dir_reg_(data_dir_reg)
  {
  }

  bool get_digital_level() const { return get_bit(*this->port_input_reg_, this->pin_) != 0; }

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
  bool has_changed() const { return this->current_state != this->previous_state; }
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

class Motor
{
private:
  PinManager pin_manager;

public:
  Motor(PinManager pin_manager) : pin_manager(pin_manager) {}

  void setup() { this->pin_manager.set_data_direction(PinManager::Output); }
  void turn_on() { this->pin_manager.set_digital_level(true); }
  void turn_off() { this->pin_manager.set_digital_level(false); }
};

class TemperatureManager
{
private:
  PinManager resistance_pin_manager;
  /**
   * É o pino de conversão analógico-digital que deve ser selecionado
   * para a leitura da temperatura. Por exemplo: o pino A0.
   */
  uint8_t channel_pin;
  volatile uint8_t *digital_input_disable_reg;
  volatile uint8_t temperature = 0;

  uint8_t estimate_temperature_from_tension(uint16_t tension) {}

public:
  TemperatureManager(PinManager resistance_pin_manager, uint8_t channel_pin,
                     volatile uint8_t *digital_input_disable_reg)
      : resistance_pin_manager(resistance_pin_manager),
        digital_input_disable_reg(digital_input_disable_reg), channel_pin(channel_pin)
  {
  }

  void setup()
  {
    this->resistance_pin_manager.set_data_direction(PinManager::Output);

    // DIDRx: Digital Input Disable Register x
    // Um único pino no ATmega328 pode ser uma saída ou uma entrada (digital ou
    // analógica); ele é multifuncional.
    //
    // A funcionalidade de entrada digital (um amplificador conectado no pino
    // pra realizar a leitura digital) pode criar um ruído que atrapalha a
    // leitura da entrada analógica. Por isso, desligamos o circuito que faz a
    // leitura digital no pino que será lido para evitar que ele gere ruído e
    // atrapalhe a leitura analógica.
    //
    // Observe que:
    // Quando um bit em DIDRx é:
    // * 0, então o buffer digital está ligado
    // * 1, então o buffer digital está desligado
    set_bit(*this->digital_input_disable_reg, this->channel_pin);
  }

  void turn_on() { this->resistance_pin_manager.set_digital_level(true); }
  void turn_off() { this->resistance_pin_manager.set_digital_level(false); }

  void begin()
  {
    // Reseta a seleção do pino de leitura (os últimos 4 bits)
    ADMUX &= 0b11110000;
    // Seleciona o bit de entrada desta instância
    ADMUX |= this->channel_pin;
    // Inicia a conversão
    ADCSRA |= (1 << ADSC);
  }

  void update_temperature(uint16_t tension)
  {
    this->temperature = this->estimate_temperature_from_tension(tension);
  }

  uint8_t get_temperature() { return this->temperature; }
};

class Stopwatch
{
private:
public:
};

class ProgramContext
{
public:
  enum Phase
  {
    Kneading = 0,
    Baking = 1,
    Raising = 2,
  };

  enum State
  {
    Idle,
    Working,
    Configurating,
  };

  bool is_working();
  bool is_idle();
  bool is_configurating();

  void begin_working();

  void begin_configuration();
  void make_idle();

  uint16_t get_phase_minutes(Phase phase);
  void increment_phase_minutes(Phase phase, int16_t step);

private:
  Phase current_phase = Phase::Kneading;
  State current_state = State::Idle;
  /**
   * Tempo de cada etapa do processo Normal *em minutos*:
   * 0: sova;
   * 1: assadura; e
   * 2: crescimento.
   */
  uint16_t phases_times[3] = {25, 90, 40};
};

#pragma endregion
#pragma region functions definitions

void move_to_next_phase_to_configure();
void handle_configuration();
void display_configuration_screen();
String pretty_print_minutes(uint16_t minutes);

#pragma endregion
#pragma region main

auto program = ProgramContext();

auto power_btn = Button(PinManager(PC2, &PINC, &PORTC, &DDRC));
auto configuration_btn = Button(PinManager(PC3, &PINC, &PORTC, &DDRC));
auto increment_btn = Button(PinManager(PC5, &PINC, &PORTC, &DDRC));
auto decrement_btn = Button(PinManager(PC4, &PINC, &PORTC, &DDRC));

auto motor = Motor(PinManager(PD2, &PIND, &PORTD, &DDRD));
auto temp_manager = TemperatureManager(PinManager(PD3, &PIND, &PORTD, &DDRD), A0, &DIDR0);

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

  motor.setup();

  // Configuração do conversor analógico-digital do microcontrolador do
  // Arduino (ATmega328).
  //
  // Esse trecho coloca o ADMUX num estado 01 (REF0:REF1) que diz que a tensão
  // usada pela porta AVCC (que alimenta o circuito analógico do
  // microcontrolador) será a mesma tensão usada como a tensão de referência
  // (valor máximo que o ADC pode medir).
  ADMUX &= ~((1 << REFS0) | (1 << REFS1));
  ADMUX |= (1 << REFS0);
  // * ADCSRx: ADC Control and Status Register x
  // * ADPSx: ADC Prescaler Select Bit x (é o x-nésimo bit menos significativo
  // do registrador ADCSRx)

  // Zera os bits do controlador & status B.
  // Esse registrador controla a fonte de gatilho, entrada e modo do ADC. Ao
  // zerar, estamos:
  // * desabilitando a inicialização automática do ADC
  // * deixando o ADC funcionar no seu modo simples de medição
  // * desligamos outras configurações avançadas do ADC
  ADCSRB = 0;
  // Configura individualmente os 3 primeiros bits do controlador & status A,
  // que definem o clock do conversor ADC (e outras funcionalidades do ADC).
  ADCSRA &= 0b11111000;
  // Essa configuração (0b111 = 128) divide o clock principal do
  // microcontrolador por 128 para formar o clock de trabalho do ADC, isso é,
  // declara que ADDCLK = CLK / 128.
  //
  // Para obter a precisão máxima de um ADC de 10 bits, o clock de conversão
  // deve estar entre 50kHz e 200kHz, e com esse cálculo, conseguimos chegar num
  // valor próximo, pois o arduino uno tem um clock de 16MHz, e 16000 (16 mega
  // hertz) / 128 = 125 (em kiloheartz).
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  // ADIE: ADC Interrupt Enable
  // Permite que uma interrupção (`ADC_vect) seja acionada sempre que o ADC
  // terminar de medir a tensão e tiver um valor pronto.
  ADCSRA |= (1 << ADIE);
  // ADEN: ADC Enable
  // Liga o ADC manualmente (a nível de hardware, tirando essa parte do circuito
  // do modo de baixo consumo de energia).
  ADCSRA |= (1 << ADEN);

  temp_manager.setup();
}

// rates in milliseconds
const uint8_t buttons_rate = 15;
const uint16_t press_time_for_long_press = 2000;
const uint16_t fast_incrementation_rate = 200;
const uint8_t display_refresh_rate = 50;

auto configuration_phase = ProgramContext::Phase::Kneading;

void loop()
{
  auto now = millis();

  ////////////////////////////////////////////////////////////////////////////////////
  // Entrando no modo de configuração
  //
  // - 1 clique em repouso: entra no modo de configuração;
  // - 1 clique curto no modo de configuração: altera a fase a ser configurada;
  // - 1 clique longo no modo de configuração: sai do modo de configuração.
  ////////////////////////////////////////////////////////////////////////////////////
  if (now - configuration_btn.time_state > buttons_rate)
  {
    static unsigned long configuration_btn_pressed_at = 0;
    configuration_btn.check();
    configuration_btn.time_state = now;

    if (configuration_btn.has_changed() && configuration_btn.has_been_pressed())
    {
      if (program.is_idle())
      {
        program.begin_configuration();
        configuration_phase = ProgramContext::Kneading;
      }
      else
      {
        configuration_btn_pressed_at = now;
      }
    }
    else if (configuration_btn.has_changed() && program.is_configurating())
    {
      const auto delta = now - configuration_btn_pressed_at;
      const auto is_long_press = delta >= press_time_for_long_press;
      is_long_press ? program.make_idle() : move_to_next_phase_to_configure();
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////
  // Configuração do tempo de cada fase.
  // Cliques longos aumentam/diminuem de 5 em 5 a cada meio segundo até serem soltos.
  ////////////////////////////////////////////////////////////////////////////////////
  if (program.is_configurating()) handle_configuration();

  if (program.is_idle() && now - power_btn.time_state > buttons_rate)
  {
    power_btn.check();
    power_btn.time_state = now;
    if (power_btn.has_changed() && power_btn.has_been_pressed())
    {
      program.begin_working();
    }
  }

  static unsigned long last_display_update = 0;
  if (now - last_display_update > display_refresh_rate)
  {
    last_display_update = now;
    if (program.is_configurating()) display_configuration_screen();
  }
}

ISR(ADC_vect)
{
  auto stable_tension = ADC;
  temp_manager.update_temperature(stable_tension);
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

  const auto is_return_or_clean_instruction = msg_type == MessageType::Instruction && data < 4;
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

bool ProgramContext::is_working() { return this->current_state == State::Working; }
bool ProgramContext::is_idle() { return this->current_state == State::Idle; }
bool ProgramContext::is_configurating() { return this->current_state == State::Configurating; }

void ProgramContext::begin_working()
{
  this->current_state = State::Working;
  this->current_phase = Phase::Kneading;
}

void ProgramContext::begin_configuration() { this->current_state = State::Configurating; }
void ProgramContext::make_idle() { this->current_state = State::Idle; }

uint16_t ProgramContext::get_phase_minutes(Phase phase) { return this->phases_times[phase]; }
void ProgramContext::increment_phase_minutes(Phase phase, int16_t step = 1)
{
  const auto new_value = this->phases_times[phase] + step;
  this->phases_times[phase] = new_value;
}

#pragma endregion
#pragma region functions implementations

void move_to_next_phase_to_configure()
{
  if (!program.is_configurating()) return;

  switch (configuration_phase)
  {
  case ProgramContext::Kneading:
    configuration_phase = ProgramContext::Raising;
    return;
  case ProgramContext::Raising:
    configuration_phase = ProgramContext::Baking;
    return;
  default:
    configuration_phase = ProgramContext::Kneading;
  }
}

void handle_configuration()
{
  const auto now = millis();

  static auto is_pressing_incrementation_buttons = false;
  static auto buttons_pressed_at = 0;
  static auto buttons_actions_time_state = 0;
  const auto current_phase_minutes = program.get_phase_minutes(configuration_phase);

  auto some_button_has_changed = false;
  auto some_button_has_been_pressed = false;
  /**
   * Se um dos botões foi pressionado, essa variável dita se foi um incremento ou decrement.
   * Caso contrário, qualquer informação nesta contida deve ser ignorada.
   */
  static auto should_increment = false;

  if (now - increment_btn.time_state > buttons_rate)
  {
    increment_btn.check();
    increment_btn.time_state = now;
    if (increment_btn.has_changed()) some_button_has_changed = true;
    if (increment_btn.has_been_pressed())
    {
      some_button_has_been_pressed = true;
      should_increment = true;
    }
  }
  // somente um dos dois botões será lido por vez e, caso ambos sejam pressionados
  // simultaneamente, o incremento tem preferência.
  else if (now - decrement_btn.time_state > buttons_rate)
  {
    decrement_btn.check();
    decrement_btn.time_state = now;
    if (decrement_btn.has_changed()) some_button_has_changed = true;
    if (decrement_btn.has_been_pressed())
    {
      some_button_has_been_pressed = true;
      should_increment = false;
    }
  }

  if (some_button_has_changed && some_button_has_been_pressed)
  {
    buttons_pressed_at = now;
    is_pressing_incrementation_buttons = true;
    buttons_actions_time_state = now;
    // realiza a primeira incrementação/decrementação assim que pressionado
    // as seguintes acontecerão a cada meio segundo
    program.increment_phase_minutes(configuration_phase, should_increment ? 1 : -1);
  }
  else if (some_button_has_changed)
  {
    is_pressing_incrementation_buttons = false;
  }
  else if (is_pressing_incrementation_buttons &&
           now - buttons_actions_time_state >= fast_incrementation_rate)
  {
    buttons_actions_time_state = now;
    const auto is_long_press = now - buttons_pressed_at >= press_time_for_long_press;
    auto incrementation = is_long_press ? 5 : 1;
    if (!should_increment) should_increment *= -1;
    program.increment_phase_minutes(configuration_phase, incrementation);
  }
}

void display_configuration_screen()
{
  Lcd::move_cursor(Lcd::Line::Upper, 0);
  Lcd::send_string(String("Config. "));

  switch (configuration_phase)
  {
  case ProgramContext::Kneading:
    Lcd::send_string("    sova");
    break;
  case ProgramContext::Raising:
    Lcd::send_string("  cresc.");
    break;
  case ProgramContext::Baking:
    Lcd::send_string("assadura");
    break;
  }

  Lcd::move_cursor(Lcd::Line::Lower, 0);
  Lcd::send_string(pretty_print_minutes(program.get_phase_minutes(configuration_phase)));
}

String pretty_print_minutes(uint16_t minutes)
{
  const uint8_t hours = minutes / 60;
  const uint8_t left_minutes = minutes % 60;

  char buffer[6];
  snprintf(buffer, sizeof(buffer), "%02d:%02d", hours, left_minutes);
  return String(buffer);
}

#pragma endregion