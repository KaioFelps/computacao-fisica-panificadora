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

  static void begin();
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

class Buzzer
{
private:
  const uint8_t pin;
  const uint16_t frequency;

  bool is_playing_ = false;
  unsigned long time_state = 0;
  uint16_t duration_ms = 0;

public:
  Buzzer(uint8_t pin, uint16_t frequency) : pin(pin), frequency(frequency) {}

  bool is_playing() const { return this->is_playing_; }

  void start(unsigned long now, uint16_t duration_ms)
  {
    tone(this->pin, this->frequency);
    this->is_playing_ = true;
    this->time_state = now;
    this->duration_ms = duration_ms;
  }

  void update(unsigned long now)
  {
    if (this->is_playing_ && now - this->time_state > this->duration_ms) this->stop();
  }

  void stop()
  {
    noTone(this->pin);
    this->is_playing_ = false;
  }

  void begin(PinManager pin_manager) const { pin_manager.set_data_direction(PinManager::Output); }
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

  void begin()
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

  void begin() { this->pin_manager.set_data_direction(PinManager::Output); }
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
  const uint8_t analogic_pin;
  const uint8_t analogic_channel;
  volatile uint8_t *digital_input_disable_reg;
  volatile uint8_t temperature = 0;

  uint16_t estimate_temperature_from_tension(uint16_t tension_mv)
  {
    const uint8_t curvature = 24;
    const uint16_t slope = 540;
    const uint8_t offset = 95;
    /**
     * Temp : Volts -> C°
     * Temp(tensao) = 2,4 * tensao^(2) + 54 * tensao + 9,5
     *
     * Como vamos receber milivolts (1V = 1000mV), o cálculo muda.
     * Para isso, dividimos todos os termos que utilizam a tensão por 1000^(k) onde
     * k é o expoente da tensão.
     *
     * Por exemplo, para o termo quadrático: 2,4 * tensao^(2), convertendo para milivolts
     * obtemos (2,4 * tensao^(2)) / 1000^(2).
     *
     * Para evitar cálculo com floats, todos os coeficientes são multiplicados por 10
     * e precisam ser divididos por 10 ao final do cálculo. Portanto,
     * (24 * tensao^(2)) / 1000^(2) ou (24 * tensao^(2)) / 1.000.000, e
     * temperatura = (termo quadrático + termo linear + constante) / 10.
     */
    const uint32_t quadratic_term =
        ((uint32_t)curvature * tension_mv * tension_mv) / (uint32_t)1000000;
    const uint16_t linear_term = (slope * (uint32_t)tension_mv) / 1000;
    // Somamos `5` pra simular um arredondamento de teto.
    const auto temperature = (quadratic_term + linear_term + offset + 5) / 10;
    return (uint16_t)temperature;
  }

public:
  TemperatureManager(PinManager resistance_pin_manager, uint8_t analogic_pin,
                     volatile uint8_t *digital_input_disable_reg)
      : resistance_pin_manager(resistance_pin_manager), analogic_pin(analogic_pin),
        analogic_channel(analogic_pin - 14), digital_input_disable_reg(digital_input_disable_reg)
  {
  }

  void begin()
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
    set_bit(*this->digital_input_disable_reg, this->analogic_channel);
  }

  void turn_on() { this->resistance_pin_manager.set_digital_level(true); }
  void turn_off() { this->resistance_pin_manager.set_digital_level(false); }

  void start_reading()
  {
    // Reseta a seleção do pino de leitura (os últimos 4 bits)
    ADMUX &= 0b11110000;
    // Seleciona o bit de entrada desta instância
    ADMUX |= this->analogic_channel;
    // Inicia a conversão
    ADCSRA |= (1 << ADSC);
  }

  void update_temperature(uint16_t adc_entry)
  {
    const auto adc_max_value = 1023;
    const auto max_milivoltage = 5000;
    const uint16_t tension = ((uint32_t)adc_entry * max_milivoltage) / adc_max_value;
    this->temperature = this->estimate_temperature_from_tension(tension);
  }

  uint8_t get_temperature() { return this->temperature; }
};

class Program
{
public:
  static const uint16_t MAX_TIME = 99 * 60 + 59;

  enum Phase
  {
    Kneading = 0,
    Baking = 1,
    Raising = 2,
  };

  enum State
  {
    Idle,
    Running,
    Configurating,
  };

  bool is_running();
  bool is_idle();
  bool is_configurating();

  void start();
  void stop();

  void begin_configuration();
  void make_idle();

  uint16_t get_phase_minutes(Phase phase) const;
  void increment_phase_minutes(Phase phase, int16_t step);

  Phase get_active_phase() const;
  void start_next_phase();

  /**
   * Checa se um ciclo foi completado e imediatamente reseta a flag.
   */
  bool check_cycle_has_finished();

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
  /**
   * Uma flag que indica se um ciclo terminou. Deve ser efêmera.
   */
  bool has_finished_a_cycle = false;
};

class Stopwatch
{
private:
  enum State
  {
    Running,
    Idle,
    Finished,
  };

  /**
   * O millis exato em que o cronômetro começou a contar (por ser iniciado ou
   * despausado).
   */
  unsigned long timer_begin_millis_print = 0;
  /**
   * A quantidade de tempo que o cronômetro deve marcar.
   */
  unsigned long goal = 0;
  uint16_t remaining_minutes = 0;
  State state = State::Idle;

  void clean()
  {
    this->timer_begin_millis_print = 0;
    this->goal = 0;
  }

  void finish()
  {
    this->clean();
    this->state = State::Finished;
  }

public:
  bool is_finished() const { return this->state == State::Finished; }

  bool is_running() const { return this->state == State::Running; }

  bool is_idle() const { return this->state == State::Idle; }

  uint16_t get_remaining_minutes() const { return this->remaining_minutes; }

  State get_state() const { return this->state; }

  void set_timer(uint16_t minutes) { this->goal = (unsigned long)minutes * 60 * 1000; }

  void start(unsigned long begin_millis_print)
  {
    this->state = State::Running;
    this->timer_begin_millis_print = begin_millis_print;
    this->remaining_minutes = 0;
  }

  void stop()
  {
    this->state = State::Idle;
    this->clean();
  }

  void calculate_current_time(unsigned long now)
  {
    if (this->state != State::Running) return;

    const auto run_millis = now - this->timer_begin_millis_print;
    const auto remaining_millis = run_millis > this->goal ? 0 : this->goal - run_millis;

    if (remaining_millis == 0) this->finish();

    const auto remaining_seconds = remaining_millis / 1000;
    // Arredondamento de teto para inteiros
    const auto remaining_minutes = (remaining_seconds + 59) / 60;
    this->remaining_minutes = ceil(remaining_minutes);
  }
};

#pragma endregion
#pragma region functions definitions

void update_lcd_display();
void move_to_next_phase_to_configure();
void handle_configuration();
void handle_configuration_mode_management();
void display_configuration_screen();
void append_temperature_to_lcd_string(char lcd_string[16]);
void prepend_formatted_minutes_to_lcd_string(char lcd_string[16], uint16_t minutes);

#pragma endregion
#pragma region main

auto program = Program();

auto power_btn = Button(PinManager(PC2, &PINC, &PORTC, &DDRC));
auto configuration_btn = Button(PinManager(PC3, &PINC, &PORTC, &DDRC));
auto increment_btn = Button(PinManager(PC5, &PINC, &PORTC, &DDRC));
auto decrement_btn = Button(PinManager(PC4, &PINC, &PORTC, &DDRC));

auto buzzer = Buzzer(A1, 300);
auto motor = Motor(PinManager(PD2, &PIND, &PORTD, &DDRD));
auto temp_manager = TemperatureManager(PinManager(PD3, &PIND, &PORTD, &DDRD), A0, &DIDR0);
auto stopwatch = Stopwatch();

void setup()
{
  Serial.begin(9600);
  Lcd::begin();
  Lcd::turn_cursor_off();

  buzzer.begin(PinManager(PC1, &PINC, &PORTC, &DDRC));

  power_btn.begin();
  configuration_btn.begin();
  increment_btn.begin();
  decrement_btn.begin();

  motor.begin();

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

  temp_manager.begin();
}

// taxas em milisegundos
const uint8_t buttons_rate = 15;
const uint16_t press_time_for_long_press = 500;
const uint16_t press_time_for_incrementation_long_press = 2000;
const uint16_t fast_incrementation_rate = 200;
const uint8_t display_refresh_rate = 50;
const uint8_t temperature_read_refresh_rate = 100;
const uint8_t stopwatch_check_refresh_rate = 250;
const uint16_t buzzer_duration = 500;

auto configuration_phase = Program::Phase::Kneading;

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
  if (now - configuration_btn.time_state > buttons_rate) handle_configuration_mode_management();

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
      program.start();
    }
  }

  if (program.is_running())
  {
    static unsigned long update_temperature_time_state = 0;
    if (now - update_temperature_time_state > temperature_read_refresh_rate)
    {
      update_temperature_time_state = now;
      temp_manager.start_reading();
    }

    if (stopwatch.is_idle())
    {
      const auto phase_minutes = program.get_phase_minutes(program.get_active_phase());
      Serial.print("Começando a contar minutos ");
      Serial.println(phase_minutes);
      stopwatch.set_timer(phase_minutes);
      stopwatch.start(now);
    }

    if (stopwatch.is_finished())
    {
      stopwatch.stop();
      program.start_next_phase();
      // TODO
      // handle_program_phase_begin();
    }

    static unsigned long stopwatch_check_time_state = 0;
    if (stopwatch.is_running() && now - stopwatch_check_time_state > stopwatch_check_refresh_rate)
    {
      stopwatch_check_time_state = now;
      stopwatch.calculate_current_time(now);
      Serial.println(stopwatch.get_remaining_minutes());
    }

    if (program.check_cycle_has_finished())
    {
      buzzer.start(now, buzzer_duration);
      Serial.println("tocar buzzer");
    }
  }

  buzzer.update(now);
  update_lcd_display();
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

void Lcd::begin()
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

bool Program::is_running() { return this->current_state == State::Running; }
bool Program::is_idle() { return this->current_state == State::Idle; }
bool Program::is_configurating() { return this->current_state == State::Configurating; }

void Program::start()
{
  this->current_state = State::Running;
  this->current_phase = Phase::Kneading;
}

Program::Phase Program::get_active_phase() const { return this->current_phase; }

void Program::begin_configuration() { this->current_state = State::Configurating; }
void Program::make_idle() { this->current_state = State::Idle; }

uint16_t Program::get_phase_minutes(Phase phase) const { return this->phases_times[phase]; }
void Program::increment_phase_minutes(Phase phase, int16_t step = 1)
{
  int16_t new_value = this->phases_times[static_cast<uint8_t>(phase)] + step;
  if (new_value < 0) new_value = 0;
  if ((uint16_t)new_value > MAX_TIME) new_value = MAX_TIME;
  this->phases_times[phase] = new_value;
}

void Program::start_next_phase()
{
  if (!this->is_running()) return;
  switch (this->current_phase)
  {
  case Phase::Kneading:
    this->current_phase = Phase::Raising;
    return;
  case Phase::Raising:
    this->current_phase = Phase::Baking;
    return;
  case Phase::Baking:
    this->stop();
    this->has_finished_a_cycle = true;
    return;
  }
}

bool Program::check_cycle_has_finished()
{
  const auto has_finished = this->has_finished_a_cycle;
  this->has_finished_a_cycle = false;
  return has_finished;
}

void Program::stop() { this->current_state = State::Idle; }

#pragma endregion
#pragma region functions implementations

void update_lcd_display()
{
  static unsigned long last_display_update = 0;
  const auto now = millis();
  if (now - last_display_update > display_refresh_rate)
  {
    last_display_update = now;
    if (program.is_configurating())
      display_configuration_screen();
    else if (program.is_idle())
    {
      Lcd::move_cursor(Lcd::Line::Upper, 0);
      Lcd::send_string(String("Panificadora    "));
      Lcd::move_cursor(Lcd::Line::Lower, 0);
      Lcd::send_string(String("BCC4004         "));
    }
    else if (program.is_running())
    {
      Lcd::move_cursor(Lcd::Line::Upper, 0);
      Lcd::send_string(String("Time  Phase Temp"));

      auto buffer = String("                  ");
      prepend_formatted_minutes_to_lcd_string(buffer.begin(), stopwatch.get_remaining_minutes());
      append_temperature_to_lcd_string(buffer.begin());

      char phase_char;
      switch (program.get_active_phase())
      {
      case Program::Phase::Kneading:
        phase_char = 'S';
        break;
      case Program::Phase::Raising:
        phase_char = 'C';
        break;
      case Program::Phase::Baking:
        phase_char = 'A';
        break;
      }

      buffer[8] = phase_char;

      Lcd::move_cursor(Lcd::Line::Lower, 0);
      Lcd::send_string(buffer);
    }
  }
}

void move_to_next_phase_to_configure()
{
  if (!program.is_configurating()) return;

  switch (configuration_phase)
  {
  case Program::Kneading:
    configuration_phase = Program::Raising;
    return;
  case Program::Raising:
    configuration_phase = Program::Baking;
    return;
  default:
    configuration_phase = Program::Kneading;
  }
}

void handle_configuration_mode_management()
{
  static unsigned long configuration_btn_pressed_at = 0;
  static bool has_just_entered_configuration_mode = false;
  const auto now = millis();

  configuration_btn.check();
  configuration_btn.time_state = now;

  if (configuration_btn.has_changed() && configuration_btn.has_been_pressed())
  {
    if (program.is_idle())
    {
      program.begin_configuration();
      configuration_phase = Program::Kneading;
      has_just_entered_configuration_mode = true;
    }
    else if (program.is_configurating())
    {
      configuration_btn_pressed_at = now;
    }
  }
  else if (configuration_btn.has_changed())
  {
    if (program.is_configurating() && !has_just_entered_configuration_mode)
    {
      const auto delta = now - configuration_btn_pressed_at;
      const auto is_long_press = delta >= press_time_for_long_press;
      is_long_press ? program.make_idle() : move_to_next_phase_to_configure();
    }

    has_just_entered_configuration_mode = false;
  }
}

void handle_configuration()
{
  const auto now = millis();
  if (now - increment_btn.time_state <= buttons_rate) return;

  static auto is_pressing_incrementation_buttons = false;
  static unsigned long buttons_pressed_at = 0;
  static unsigned long buttons_actions_time_state = 0;

  auto some_button_has_changed = false;
  auto some_button_has_been_pressed = false;
  /**
   * Se um dos botões foi pressionado, essa variável dita se foi um incremento ou decrement.
   * Caso contrário, qualquer informação nesta contida deve ser ignorada.
   */
  static auto should_increment = false;

  decrement_btn.check();
  decrement_btn.time_state = now;
  increment_btn.check();
  increment_btn.time_state = now;

  // caso ambos sejam pressionados simultaneamente, o incremento tem preferência.
  if (decrement_btn.has_changed() || increment_btn.has_changed()) some_button_has_changed = true;
  if (decrement_btn.has_been_pressed())
  {
    some_button_has_been_pressed = true;
    should_increment = false;
  }
  else if (increment_btn.has_been_pressed())
  {
    some_button_has_been_pressed = true;
    should_increment = true;
  }

  if (some_button_has_changed && some_button_has_been_pressed)
  {
    buttons_pressed_at = now;
    is_pressing_incrementation_buttons = true;
    // garante que haverá uma pausa até que comece a incrementar continuamente,
    // evitando incrementos acidentais
    buttons_actions_time_state = now + fast_incrementation_rate * 2;
    // realiza a primeira incrementação/decrementação assim que pressionado
    // as seguintes acontecerão a cada meio segundo
    program.increment_phase_minutes(configuration_phase, should_increment ? 1 : -1);
    return;
  }

  if (some_button_has_changed)
  {
    is_pressing_incrementation_buttons = false;
    return;
  }

  auto delta = (long long)now - (long long)buttons_actions_time_state;
  if (!is_pressing_incrementation_buttons || delta < fast_incrementation_rate)
  {
    return;
  }

  buttons_actions_time_state = now;
  const auto is_long_press = now - buttons_pressed_at >= press_time_for_incrementation_long_press;

  auto incrementation = is_long_press ? 5 : 1;
  if (!should_increment) incrementation *= -1;
  program.increment_phase_minutes(configuration_phase, incrementation);
}

void display_configuration_screen()
{
  Lcd::move_cursor(Lcd::Line::Upper, 0);
  Lcd::send_string(String("Config. "));

  switch (configuration_phase)
  {
  case Program::Kneading:
    Lcd::send_string("sova    ");
    break;
  case Program::Raising:
    Lcd::send_string("cresc.  ");
    break;
  case Program::Baking:
    Lcd::send_string("assadura");
    break;
  }

  auto buffer = String("                ");
  const auto minutes = program.get_phase_minutes(configuration_phase);
  prepend_formatted_minutes_to_lcd_string(buffer.begin(), minutes);

  Lcd::move_cursor(Lcd::Line::Lower, 0);
  Lcd::send_string(buffer);
}

void prepend_formatted_minutes_to_lcd_string(char lcd_string[16], uint16_t minutes)
{
  const uint8_t hours = minutes / 60;
  const uint8_t left_minutes = minutes % 60;

  char buffer[6];
  snprintf(buffer, sizeof(buffer), "%02d:%02d", hours, left_minutes);

  const auto buffer_copy_limit = sizeof(buffer) - 1; // ignora o caractere nulo
  for (size_t i = 0; i < buffer_copy_limit; i++)
  {
    lcd_string[i] = buffer[i];
  }
}

void append_temperature_to_lcd_string(char lcd_string[16])
{
  auto offset = 15;
  auto temperature = temp_manager.get_temperature();
  const auto degree_mark_character = 0;

  lcd_string[offset--] = degree_mark_character;

  while (temperature != 0)
  {
    lcd_string[offset--] = temperature % 10 + '0';
    temperature /= 10;
  }
}

#pragma endregion