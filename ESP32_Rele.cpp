#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <ctype.h>
#include <stdlib.h>

namespace {

// ======================================================
// CONFIGURACION GENERAL
// ======================================================

constexpr char BT_DEVICE_NAME[] = "ESP32-Rele-Pruebas";

constexpr uint8_t RELAY_PIN = 15;
constexpr uint8_t RELAY_ACTIVE_LEVEL = HIGH;
constexpr uint8_t RELAY_INACTIVE_LEVEL =
    (RELAY_ACTIVE_LEVEL == HIGH) ? LOW : HIGH;

// Botones conectados a GND usando INPUT_PULLUP.
constexpr uint8_t START_BUTTON_PIN = 25;
constexpr uint8_t STOP_BUTTON_PIN = 26;
constexpr uint8_t RESET_BUTTON_PIN = 27;

constexpr uint32_t DEFAULT_ON_MS = 1000;
constexpr uint32_t DEFAULT_OFF_MS = 1000;
constexpr uint32_t MIN_TIME_MS = 100;
constexpr uint32_t MAX_TIME_MS = 86400000UL;  // 24 horas

constexpr uint32_t COMMAND_IDLE_TIMEOUT_MS = 60;
constexpr uint32_t SERIAL_STATUS_INTERVAL_MS = 1000;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;

constexpr size_t COMMAND_BUFFER_SIZE = 96;

// ======================================================
// TIPOS
// ======================================================

enum class RunMode : uint8_t {
  STOPPED,
  CYCLE,
  PULSE,
  MANUAL_CLOSED
};

enum class CommandSource : uint8_t {
  NONE,
  BLUETOOTH,
  USB_SERIAL,
  MANUAL_BUTTON
};

struct ButtonState {
  uint8_t pin;
  bool lastReading = HIGH;
  bool stableState = HIGH;
  uint32_t lastChangeMs = 0;
};

struct InputState {
  String buffer;
  uint32_t lastByteMs = 0;
};

// ======================================================
// OBJETOS Y ESTADO GLOBAL
// ======================================================

BluetoothSerial SerialBT;
Preferences preferences;

uint32_t onTimeMs = DEFAULT_ON_MS;
uint32_t offTimeMs = DEFAULT_OFF_MS;

RunMode runMode = RunMode::STOPPED;
CommandSource activeSource = CommandSource::NONE;

bool relayClosed = false;
bool relayInitialized = false;
bool bluetoothWasConnected = false;

// Se reutiliza para el inicio del pulso y para el ultimo cambio de fase.
uint32_t modeTimestampMs = 0;
uint32_t pulseDurationMs = 0;
uint32_t lastStatusMs = 0;

InputState bluetoothInput;
InputState serialInput;

ButtonState startButton{START_BUTTON_PIN};
ButtonState stopButton{STOP_BUTTON_PIN};
ButtonState resetButton{RESET_BUTTON_PIN};

String lastReceivedCommand = "(sin comandos)";

// ======================================================
// UTILIDADES
// ======================================================

const char *runModeToString(RunMode mode) {
  switch (mode) {
    case RunMode::STOPPED:
      return "STOPPED";
    case RunMode::CYCLE:
      return "CYCLE";
    case RunMode::PULSE:
      return "PULSE";
    case RunMode::MANUAL_CLOSED:
      return "MANUAL_CLOSED";
    default:
      return "UNKNOWN";
  }
}

const char *sourceToString(CommandSource source) {
  switch (source) {
    case CommandSource::NONE:
      return "NONE";
    case CommandSource::BLUETOOTH:
      return "BLUETOOTH";
    case CommandSource::USB_SERIAL:
      return "USB_SERIAL";
    case CommandSource::MANUAL_BUTTON:
      return "MANUAL_BUTTON";
    default:
      return "UNKNOWN";
  }
}

String trimCopy(String value) {
  value.trim();
  return value;
}

String upperCopy(String value) {
  for (size_t i = 0; i < value.length(); ++i) {
    value[i] = static_cast<char>(
        toupper(static_cast<unsigned char>(value[i])));
  }

  return value;
}

// ======================================================
// SALIDAS DE DIAGNOSTICO
// ======================================================

void logLine(const String &message = "") {
  Serial.println(message);
}

void broadcast(const String &message) {
  Serial.print(message);

  if (SerialBT.hasClient()) {
    SerialBT.print(message);
  }
}

void broadcastLine(const String &message = "") {
  broadcast(message + "\r\n");
}

// ======================================================
// CONTROL DEL RELE
// ======================================================

bool isRelayOutputActive() {
  return digitalRead(RELAY_PIN) == RELAY_ACTIVE_LEVEL;
}

void setRelayClosed(bool closed) {
  const bool stateChanged = !relayInitialized || relayClosed != closed;

  relayClosed = closed;
  digitalWrite(RELAY_PIN, closed ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
  relayInitialized = true;

  if (stateChanged) {
    logLine(
        String("[RELE] ") +
        (closed ? "CERRADO" : "ABIERTO") +
        " | salida=" +
        (isRelayOutputActive() ? "ACTIVA" : "INACTIVA"));
  }
}

void initializeRelaySafe() {
  relayClosed = false;
  digitalWrite(RELAY_PIN, RELAY_INACTIVE_LEVEL);
  relayInitialized = true;
}

void stopAndOpenRelay(const String &reason, bool broadcastMessage = true) {
  runMode = RunMode::STOPPED;
  activeSource = CommandSource::NONE;
  setRelayClosed(false);

  const String message = "[STOP] " + reason + ". Rele ABIERTO.";

  if (broadcastMessage) {
    broadcastLine(message);
  } else {
    logLine(message);
  }
}

// ======================================================
// MEMORIA NO VOLATIL
// ======================================================

bool isValidTime(uint32_t value) {
  return value >= MIN_TIME_MS && value <= MAX_TIME_MS;
}

void saveTimings() {
  preferences.putULong("on_ms", onTimeMs);
  preferences.putULong("off_ms", offTimeMs);
}

void loadSavedTimings() {
  preferences.begin("relaycfg", false);

  onTimeMs = preferences.getULong("on_ms", DEFAULT_ON_MS);
  offTimeMs = preferences.getULong("off_ms", DEFAULT_OFF_MS);

  if (!isValidTime(onTimeMs)) {
    onTimeMs = DEFAULT_ON_MS;
  }

  if (!isValidTime(offTimeMs)) {
    offTimeMs = DEFAULT_OFF_MS;
  }
}

// ======================================================
// ACCIONES DE ALTO NIVEL
// ======================================================

void startCycle(CommandSource source) {
  runMode = RunMode::CYCLE;
  activeSource = source;
  modeTimestampMs = millis();

  setRelayClosed(true);

  broadcastLine(
      String("Ciclo iniciado por ") +
      sourceToString(source) +
      ". Fase CERRADO.");
}

void startPulse(uint32_t durationMs, CommandSource source) {
  runMode = RunMode::PULSE;
  activeSource = source;
  modeTimestampMs = millis();
  pulseDurationMs = durationMs;

  setRelayClosed(true);

  broadcastLine(
      String("Pulso iniciado por ") +
      sourceToString(source) +
      " durante " +
      String(durationMs) +
      " ms. Rele CERRADO.");
}

void holdRelayClosed(CommandSource source) {
  runMode = RunMode::MANUAL_CLOSED;
  activeSource = source;

  setRelayClosed(true);

  broadcastLine(
      String("Comando C ejecutado por ") +
      sourceToString(source) +
      ". Rele CERRADO fijo.");
}

void holdRelayOpen(CommandSource source) {
  runMode = RunMode::STOPPED;
  activeSource = CommandSource::NONE;

  setRelayClosed(false);

  broadcastLine(
      String("Comando A ejecutado por ") +
      sourceToString(source) +
      ". Rele ABIERTO fijo.");
}

void restartController(CommandSource source) {
  broadcastLine(
      String("[RESET] Reinicio solicitado por ") +
      sourceToString(source) +
      ". Abriendo rele antes de reiniciar.");

  runMode = RunMode::STOPPED;
  activeSource = CommandSource::NONE;
  setRelayClosed(false);

  delay(150);
  ESP.restart();
}

// ======================================================
// ESTADO Y AYUDA
// ======================================================

void sendStatus() {
  broadcastLine("Estado:");
  broadcastLine("  MODO=" + String(runModeToString(runMode)));
  broadcastLine("  FUENTE_ACTIVA=" + String(sourceToString(activeSource)));
  broadcastLine("  RELE_LOGICO=" + String(relayClosed ? "CERRADO" : "ABIERTO"));
  broadcastLine("  SALIDA_RELE=" + String(isRelayOutputActive() ? "ACTIVA" : "INACTIVA"));
  broadcastLine("  LOGICA_RELE=" + String(RELAY_ACTIVE_LEVEL == HIGH ? "ACTIVO_HIGH" : "ACTIVO_LOW"));
  broadcastLine("  ON=" + String(onTimeMs) + " ms");
  broadcastLine("  OFF=" + String(offTimeMs) + " ms");
  broadcastLine("  BT_CLIENTE=" + String(SerialBT.hasClient() ? "SI" : "NO"));
  broadcastLine("  PIN_RELE=GPIO" + String(RELAY_PIN));
  broadcastLine("  START_BUTTON=GPIO" + String(START_BUTTON_PIN) + " -> CERRADO FIJO");
  broadcastLine("  STOP_BUTTON=GPIO" + String(STOP_BUTTON_PIN) + " -> ABIERTO");
  broadcastLine("  RESET_BUTTON=GPIO" + String(RESET_BUTTON_PIN) + " -> RESET");
}

void sendHelp() {
  broadcastLine();
  broadcastLine("=== ESP32 RELE INDUSTRIAL ===");
  broadcastLine("Comandos disponibles:");
  broadcastLine("  C                  -> mantiene rele CERRADO fijo");
  broadcastLine("  A                  -> mantiene rele ABIERTO fijo");
  broadcastLine("  B                  -> inicia ciclo automatico");
  broadcastLine("  S                  -> stop inmediato / abre rele");
  broadcastLine("  ON=1200;OFF=800    -> configura ambos tiempos");
  broadcastLine("  ON=1200            -> configura tiempo cerrado");
  broadcastLine("  OFF=800            -> configura tiempo abierto");
  broadcastLine("  PULSE=1500 o P=1500 -> pulso cerrado unico");
  broadcastLine("  RESET              -> abre el rele y reinicia");
  broadcastLine("  STATUS             -> muestra el estado actual");
  broadcastLine("  ?                  -> muestra esta ayuda");
  broadcastLine();
  broadcastLine("Botones fisicos:");
  broadcastLine("  START -> rele CERRADO fijo");
  broadcastLine("  STOP  -> rele ABIERTO");
  broadcastLine("  RESET -> abre el rele y reinicia");
  broadcastLine();
  broadcastLine("Seguridad Bluetooth:");
  broadcastLine("  Si una accion iniciada por Bluetooth sigue activa y el cliente se desconecta,");
  broadcastLine("  el rele se abre. Las acciones iniciadas por Serial o botones no se modifican.");
  broadcastLine();

  sendStatus();
}

// ======================================================
// PARSEO DE TIEMPOS
// ======================================================

bool parseTimeValue(const String &text, uint32_t &result) {
  const String value = trimCopy(text);

  if (value.isEmpty() || value.length() >= 24) {
    return false;
  }

  char buffer[24] = {};
  value.toCharArray(buffer, sizeof(buffer));

  char *endPtr = nullptr;
  const unsigned long parsed = strtoul(buffer, &endPtr, 10);

  if (buffer[0] == '\0' || *endPtr != '\0' || !isValidTime(parsed)) {
    return false;
  }

  result = static_cast<uint32_t>(parsed);
  return true;
}

bool parseTimingToken(const String &token,
                      uint32_t &newOnMs,
                      uint32_t &newOffMs,
                      bool &onChanged,
                      bool &offChanged) {
  const String trimmedToken = trimCopy(token);
  const String upperToken = upperCopy(trimmedToken);

  uint32_t parsedValue = 0;

  if (upperToken.startsWith("ON=")) {
    if (!parseTimeValue(trimmedToken.substring(3), parsedValue)) {
      broadcastLine("Error: valor ON invalido.");
      return false;
    }

    newOnMs = parsedValue;
    onChanged = true;
    return true;
  }

  if (upperToken.startsWith("OFF=")) {
    if (!parseTimeValue(trimmedToken.substring(4), parsedValue)) {
      broadcastLine("Error: valor OFF invalido.");
      return false;
    }

    newOffMs = parsedValue;
    offChanged = true;
    return true;
  }

  broadcastLine("Error: comando no reconocido -> " + trimmedToken);
  broadcastLine("Use ? para ver instrucciones.");
  return false;
}

void processTimingCommand(const String &command) {
  uint32_t newOnMs = onTimeMs;
  uint32_t newOffMs = offTimeMs;
  bool onChanged = false;
  bool offChanged = false;

  int tokenStart = 0;

  while (tokenStart < static_cast<int>(command.length())) {
    int separator = command.indexOf(';', tokenStart);

    if (separator < 0) {
      separator = command.length();
    }

    const String token = command.substring(tokenStart, separator);

    if (!trimCopy(token).isEmpty() &&
        !parseTimingToken(token, newOnMs, newOffMs, onChanged, offChanged)) {
      return;
    }

    tokenStart = separator + 1;
  }

  if (!onChanged && !offChanged) {
    broadcastLine("Error: comando vacio o no reconocido.");
    return;
  }

  onTimeMs = newOnMs;
  offTimeMs = newOffMs;
  saveTimings();

  broadcastLine("Tiempos actualizados y guardados en memoria.");
  sendStatus();
}

// ======================================================
// PROCESAMIENTO DE COMANDOS
// ======================================================

void processCommand(String command, CommandSource source) {
  command = trimCopy(command);

  if (command.isEmpty()) {
    return;
  }

  lastReceivedCommand = String(sourceToString(source)) + ": " + command;
  logLine(String("[RX ") + sourceToString(source) + "] " + command);

  const String upperCommand = upperCopy(command);

  if (upperCommand == "C") {
    holdRelayClosed(source);
  } else if (upperCommand == "A") {
    holdRelayOpen(source);
  } else if (upperCommand == "B") {
    startCycle(source);
  } else if (upperCommand == "S") {
    stopAndOpenRelay(String("STOP aplicado por ") + sourceToString(source));
  } else if (upperCommand == "?") {
    sendHelp();
  } else if (upperCommand == "STATUS") {
    sendStatus();
  } else if (upperCommand == "RESET") {
    restartController(source);
  } else if (upperCommand.startsWith("PULSE=") || upperCommand.startsWith("P=")) {
    const int equalIndex = command.indexOf('=');
    uint32_t durationMs = 0;

    if (!parseTimeValue(command.substring(equalIndex + 1), durationMs)) {
      broadcastLine(
          "Error: valor de PULSE invalido. Use entre " +
          String(MIN_TIME_MS) +
          " y " +
          String(MAX_TIME_MS) +
          " ms.");
      return;
    }

    startPulse(durationMs, source);
  } else {
    processTimingCommand(command);
  }
}

// ======================================================
// ENTRADAS NO BLOQUEANTES
// ======================================================

void processInput(Stream &stream, InputState &input, CommandSource source) {
  while (stream.available()) {
    const char incoming = static_cast<char>(stream.read());
    input.lastByteMs = millis();

    if (incoming == '\r' || incoming == '\n') {
      if (!input.buffer.isEmpty()) {
        processCommand(input.buffer, source);
        input.buffer = "";
      }

      continue;
    }

    if (input.buffer.length() < COMMAND_BUFFER_SIZE) {
      input.buffer += incoming;
    } else {
      input.buffer = "";
      broadcastLine("Error: comando demasiado largo. Buffer descartado.");
    }
  }

  if (!input.buffer.isEmpty() &&
      static_cast<uint32_t>(millis() - input.lastByteMs) >=
          COMMAND_IDLE_TIMEOUT_MS) {
    processCommand(input.buffer, source);
    input.buffer = "";
  }
}

void processCommandInputs() {
  if (SerialBT.hasClient()) {
    processInput(SerialBT, bluetoothInput, CommandSource::BLUETOOTH);
  } else {
    bluetoothInput.buffer = "";
  }

  processInput(Serial, serialInput, CommandSource::USB_SERIAL);
}

// ======================================================
// MAQUINA DE ESTADOS
// ======================================================

void updateStateMachine() {
  const uint32_t now = millis();

  switch (runMode) {
    case RunMode::CYCLE: {
      const uint32_t phaseDurationMs = relayClosed ? onTimeMs : offTimeMs;

      if (static_cast<uint32_t>(now - modeTimestampMs) >= phaseDurationMs) {
        modeTimestampMs = now;
        setRelayClosed(!relayClosed);

        broadcastLine(
            String("Cambio de fase -> ") +
            (relayClosed ? "CERRADO" : "ABIERTO") +
            " | fuente=" +
            sourceToString(activeSource));
      }
      break;
    }

    case RunMode::PULSE:
      if (static_cast<uint32_t>(now - modeTimestampMs) >= pulseDurationMs) {
        stopAndOpenRelay("Pulso finalizado");
      }
      break;

    case RunMode::STOPPED:
    case RunMode::MANUAL_CLOSED:
      break;
  }
}

// ======================================================
// BOTONES FISICOS
// ======================================================

bool wasButtonPressed(ButtonState &button) {
  const bool reading = digitalRead(button.pin);
  const uint32_t now = millis();

  if (reading != button.lastReading) {
    button.lastReading = reading;
    button.lastChangeMs = now;
  }

  if (static_cast<uint32_t>(now - button.lastChangeMs) < BUTTON_DEBOUNCE_MS ||
      reading == button.stableState) {
    return false;
  }

  button.stableState = reading;
  return button.stableState == LOW;
}

void processButtons() {
  // RESET y STOP tienen prioridad sobre START.
  if (wasButtonPressed(resetButton)) {
    logLine("[BOTON] RESET presionado.");
    restartController(CommandSource::MANUAL_BUTTON);
    return;
  }

  if (wasButtonPressed(stopButton)) {
    logLine("[BOTON] STOP presionado.");
    stopAndOpenRelay("STOP por boton fisico");
    return;
  }

  if (wasButtonPressed(startButton)) {
    logLine("[BOTON] START/CERRADO presionado.");
    holdRelayClosed(CommandSource::MANUAL_BUTTON);
  }
}

// ======================================================
// SUPERVISION BLUETOOTH
// ======================================================

void handleBluetoothConnection() {
  const bool connected = SerialBT.hasClient();

  if (connected && !bluetoothWasConnected) {
    logLine(String("[BT] Cliente conectado. Nombre: ") + BT_DEVICE_NAME);
    broadcastLine("Dispositivo conectado por Bluetooth.");
    sendHelp();
  } else if (!connected && bluetoothWasConnected) {
    bluetoothInput.buffer = "";

    if (activeSource == CommandSource::BLUETOOTH) {
      stopAndOpenRelay(
          "Bluetooth desconectado: corte de seguridad porque la accion activa venia de Bluetooth",
          false);
    } else {
      logLine(
          "[BT] Cliente desconectado. El rele no cambia porque la accion activa no venia de Bluetooth.");
    }
  }

  bluetoothWasConnected = connected;
}

// ======================================================
// HEARTBEAT DE DIAGNOSTICO
// ======================================================

void printSerialHeartbeat() {
  const uint32_t now = millis();

  if (static_cast<uint32_t>(now - lastStatusMs) <
      SERIAL_STATUS_INTERVAL_MS) {
    return;
  }

  lastStatusMs = now;

  logLine(
      "[STATUS] BT=" +
      String(SerialBT.hasClient() ? "CONECTADO" : "DESCONECTADO") +
      " | MODO=" + String(runModeToString(runMode)) +
      " | FUENTE=" + String(sourceToString(activeSource)) +
      " | RELE=" + String(relayClosed ? "CERRADO" : "ABIERTO") +
      " | SALIDA=" + String(isRelayOutputActive() ? "ACTIVA" : "INACTIVA") +
      " | LOGICA=" + String(RELAY_ACTIVE_LEVEL == HIGH ? "HIGH" : "LOW") +
      " | ON=" + String(onTimeMs) + " ms" +
      " | OFF=" + String(offTimeMs) + " ms" +
      " | ULT_CMD=" + lastReceivedCommand);
}

// ======================================================
// INICIALIZACION
// ======================================================

void initializeHardware() {
  pinMode(RELAY_PIN, OUTPUT);
  initializeRelaySafe();

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
}

void initializeBluetooth() {
  if (SerialBT.begin(BT_DEVICE_NAME)) {
    return;
  }

  logLine("[ERROR] No se pudo iniciar Bluetooth.");
  initializeRelaySafe();

  while (true) {
    delay(1000);
  }
}

}  // namespace

// ======================================================
// SETUP Y LOOP PRINCIPAL
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  initializeHardware();
  loadSavedTimings();
  initializeBluetooth();

  logLine("ESP32 listo.");
  logLine(String("[BT] Nombre: ") + BT_DEVICE_NAME);
  logLine("[SERIAL] Monitor listo a 115200 baudios.");
  logLine("[SEGURIDAD] Rele inicializado en estado ABIERTO.");
  logLine(
      String("[SEGURIDAD] Logica del rele: ACTIVO ") +
      (RELAY_ACTIVE_LEVEL == HIGH ? "HIGH." : "LOW."));

  sendHelp();
}

void loop() {
  handleBluetoothConnection();
  processCommandInputs();
  processButtons();
  updateStateMachine();
  printSerialHeartbeat();

  delay(1);
}
