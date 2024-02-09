//  токен доступа на gitlab: glpat-rn2fPZdNUwZh9yVov6bY
//     G01X-123456Y-123456Z-123456M06F24000;
//     0123456789012345678901234567890123456    i в массиве
//     0         1         2         3         десятки
//     1234567890123456789012345678901234567     штук ()
//     0        1         2         3            десятки

#include "SPI.h"        // библиотека для работы с периферийными устройствами
#include "SD.h"         // библиотека для работы с sd картами
File ncFile;            // создаем объект класса File для работы с файлами на флешке
char data[37];          // строка с данными из .nc файла

// пины для sd карты (объявлены в присоединенной библиотеке):
// MISO: 50
// MOSI: 51
// SCK:  52
// cs:   (8)53
// pin 10 для работы с sd картой обязательно должен быть определен как OUTPUT. (Он же - пин SS). Для чего - не знаю.

const int8_t pinStepX1 = 2;    // на шилде это драйвер "X"
const int8_t pinStepX2 = 3;    // -/- Y
const int8_t pinStepY1 = 4;    // -/- Z
const int8_t pinStepY2 = 12;   // -/- A
const int8_t pinStepZ = 9;     // на шилде это пин Limit X axis (концевик)
const int8_t pinDirX1 = 5;
const int8_t pinDirX2 = 6;
const int8_t pinDirY1 = 7;
const int8_t pinDirY2 = 13;
const int8_t pinDirZ = 11;     // на шилде это пин Limit Z axis
const int8_t pinEn = 8;

const int8_t chipSelect = 53;

uint8_t microStep = 8;                    // микрошаг шаговых двигателей
uint8_t gSpeed = 0;                       // G-команда, 1 - подача / 2 - ускоренное перемещение
uint16_t fSpeed = 180;                    // F-команда, подача в миллиметрах в минуту

String fileName = "do_now.ncm";

// пины, отвечающие за ручное управление:
#define pinStart 44             // тумблер старт программы
#define pinTuneMachine 49       // тумблер выйти в ноль координат станка
#define pinSetG54 46            // тумблер установить референтную точку G54; ноль заготовки
#define pinGoToG54 43           // тумблер перейти к референтной точке G54
#define ToolTouchDetected 39    // пин слушает касание инструментом датчика
#define pinSetToolSensor 40     // тумблер задать координаты точки в которой будет происходить смена инструмента
#define pinAutoSetTool 42       // кнопка автоматически подводит кончик инструмента к датчику касания инструмента

#define pinStepByStep 28        // тумблеры скоростей на пульте управления
#define pinFiSpeed 26
#define pinSeSpeed 24
#define pinThSpeed 22
#define pinToLeft 31            // кнопки движения на пульте управления
#define pinToRight 33
#define pinToForward 27
#define pinToBack 29
#define pinToTop 25
#define pinToBottom 23

// концевики - контроль рабочей зоны станка
#define pinEnTuning 30          // пин, включение режима ограничения выхода за рабочую зону
#define pinLimitSwitchX 47      // пин, куда подключается концевик подстроечный
#define pinLimitSwitchY 48
#define pinLimitSwitchZ 38

// выход в ноль координат ЗАГОТОВКИ, поиск центра заготовки
#define pinOutRect 41           // пин, старт процедуры поиска центра заготовки - прямоугольник, снаружи
#define pinTouchProbe 36        // пин, снимает показания с датчика 3D Touch Probe

#define left LOW                           // маска направлления движения по осям HIGH / LOW
#define right HIGH
#define forward HIGH
#define back LOW
#define top HIGH
#define bottom LOW

#define leftFlag false                     // маска флагов направлений движения (true/false)
#define rightFlag true
#define forwardFlag true
#define backFlag false
#define topFlag true
#define bottomFlag false

bool xDir = false;                         // флаг хранит направление движения данной оси в данном кадре true - вправо, false - влево
bool yDir = false;                         // true - вперед, false - назад
bool zDir = false;                         // true - вверх, false - вниз

const uint32_t zDistance = 65600;          // длина оси Z
const uint16_t spacerHeight = 4000;        // высота проставки в шагах для установки референтной точки. 4000 шагов - это 5мм.

int32_t xStepsFrame = 0;                   // количество шагов, сколько планируется выполнить в данном кадре
int32_t yStepsFrame = 0;
int32_t zStepsFrame = 0;                   // значение по z умножается на 2, потому что на этой оси стоит однозаходный винт

uint32_t xStepsFrameABS = 0;               // количество шагов по модулю в кадре (без учета знака, т.е. направления)
uint32_t yStepsFrameABS = 0;
uint32_t zStepsFrameABS = 0;

uint32_t xStepsDone = 0;               // выполненное количество шагов в текущем кадре
uint32_t yStepsDone = 0;
uint32_t zStepsDone = 0;

int32_t xPlaneToTime = 0;            // количество шагов, сколько планируется должно было бы быть выполнено от начала программы до данного момента
int32_t yPlaneToTime = 0;            // считается на основе чисел, полученных с sd карты и прошедших через парсер
int32_t zPlaneToTime = 0;            // к слову сказать, количество шагов по z умножается на 2, потому что по z стоит однозаходный винт

uint32_t xDoneToTime = 0;          // количество сделанных шагов за всё время от начала выполнения программы до данного момента
uint32_t yDoneToTime = 0;
uint32_t zDoneToTime = 0;

char tempNumChar[9];                     // массив для хранения чисел, преобразованных в строки функцией intToChar();

uint32_t frameCounter = 0;              // счетчик выполненных к данному моменту кадров. Выводится на компьютер для визуального контроля прогресса обработки


// описываем классы:


class SpeedControl {
  public:
  const float qEquilateral = 1.414;           // коэффициент увеличения длительности такта при движении по 2 осям X и Y (равносторонний треугольник)
  const float qNotEquiLateral = 1.153;        // коэффициент при движении по 2 осям с участием оси Z
  const float qThreeAxis = 1.499;             // коэффициент для 3-х осей
  const uint16_t durLow1AxAccel = 172;        // длительность сигнала низкого уровня на ускоренном переменщении при движении только по одной из осей
  const uint16_t durLow2AxAccel = durLow1AxAccel * qEquilateral;       // длительность сигнала низкого уровня на ускоренном перемещении по двум осям (без участия оси Z)
  const uint16_t durLow2AxZAccel = durLow1AxAccel * qNotEquiLateral;   // длительность сигнала низкого уровня на ускоренном перемещении по двум осям (c участием оси Z)
  const uint16_t durLow3AxAccel = durLow1AxAccel * qThreeAxis;         // длительность сигнала низкого уровня на ускоренном перемещении по трем осям одновременно
  const uint8_t durHighLevel = 15;            // длительность высокого уровня сигнала на двигателе
  const uint16_t pulsPerMm = 400;             // импульсов в одном миллиметре
  const uint16_t maxSpeed = 187;              // максимальная допустимая скорость движения для данного станка. измеряется в микросекундах (длина полного такта)
  //                                             187 микросекунд - это длина такта соответствующая скорости движения 800 мм/с по оси X или Y
  uint16_t durLow1Ax;                         // длительность сигнала низкого уровня на двигателе при движении только по одной оси (кроме Z)
  uint16_t durLow1AxZ;                        // длительность сигнала низкого уровня при движении только по оси Z
  uint16_t durLow2Ax;                         // длительность низкого уровня при движении 2 осей БЕЗ Z (только по X и Y)
  uint16_t durLow2AxZ;                        // длительность низкого уровня при движении 2 осей с участием оси Z
  uint16_t durLow3Ax;                         // длительность низкого уровня при движении 3 осей


  // метод устанавливает значения переменных при смене скорости движения по осям
  void setSpeed() {
    uint32_t pulsPerMin;                        // импульсов в минуту
    uint32_t pulsPerSek;                        // импульсов в секунду
  //                                               такт - это сумма длительностей высокого и низкого уровней сигнала на двигателе
    uint16_t beatDur1Ax;                        // длительность (Duration) такта (Beat) в микросекундах (1/1000.000 доля секунды) при движении только по одной оси (кроме Z)
    uint16_t beatDur1AxZ;                       // длительность такта только по оси Z
    uint16_t beatDur2Ax;                        // длительность такта при движении 2 осей БЕЗ Z (по X и Y) (на z другой винт)
    uint16_t beatDur2AxZ;                       // длительность такта при движении 2 осей с участием оси Z
    uint16_t beatDur3Ax;                        // длительность такта при движении 3 осей
//                                                 вычисляем:
    pulsPerMin = (long)pulsPerMm * fSpeed;      // сигналов в минуту
    pulsPerSek = pulsPerMin / 60;               // сигналов в секунду

    beatDur1Ax = ((1.0 / pulsPerSek) * (long)1000000.0);   // длительность такта при движении только по одной из осей (кроме Z)
    // дальнейшие расчеты строятся от beatDur1Ax, поэтому проверяем, нет ли превышения максимально допустимой скорости для данного станка
    if (beatDur1Ax < maxSpeed) {
      beatDur1Ax = maxSpeed;                    // принудительно ограничиваем полученную из nc-файла скорость максимальной скоростью из константы
    }
    beatDur1AxZ = beatDur1Ax / 2;               // длительность такта при движении только по оси Z
    beatDur2Ax = beatDur1Ax * qEquilateral;     // две оси без Z
    beatDur2AxZ = beatDur1Ax * qNotEquiLateral;   // две оси + Z
    beatDur3Ax = beatDur1Ax * qThreeAxis;       // три оси
//                                                  уровни низкого сигнала для отправки на двигатель
    durLow1Ax = beatDur1Ax - durHighLevel;      // 1 ось -Z
    durLow1AxZ = beatDur1AxZ - durHighLevel;    // 1 ось +Z
    durLow2Ax = beatDur2Ax - durHighLevel;      // 2 оси -Z
    durLow2AxZ = beatDur2AxZ - durHighLevel;    // 2 оси +Z
    durLow3Ax = beatDur3Ax - durHighLevel;      // 3 оси
  }

  // метод возвращает скорость вращения моторов при движении только по одной оси
  uint16_t getSpeed(char axis) {
    if (axis != 'z') {                  // запрашиваемая скорость для осей X и Y
      if (gSpeed == 1) {                // на подаче
        return durLow1Ax;
      } else if (gSpeed == 0) {         // на ускоренном
        return durLow1AxAccel;
      }
    } else if (axis == 'z') {           // запрашиваемая скорость для оси Z
      if (gSpeed == 1) {                // на подаче
        return durLow1AxZ;
      } else if (gSpeed == 0) {         // на ускоренном
        return durLow1AxAccel;
      }
    }
  }

  // метод возвращает скорость вращения моторов при одновременном движении по двум осям
  uint16_t getSpeed(char axis1, char axis2) {
    if (axis1 != 'z' && axis2 != 'z') {                  // запрашиваемая скорость для осей X и Y (без участия оси Z)
      if (gSpeed == 1) {                // на подаче
        return durLow2Ax;
      } else if (gSpeed == 0) {         // на ускоренном
        return durLow2AxAccel;
      }
    } else if (axis1 == 'z' || axis2 == 'z') {           // запрашиваемая скорость для осей XZ или YZ
      if (gSpeed == 1) {                // на подаче
        return durLow2AxZ;
      } else if (gSpeed == 0) {         // на ускоренном
        return durLow2AxZAccel;
      }
    }
  }

  // метод возвращает скорость вращения моторов при одновременном движении по трем осям
  uint16_t getSpeed(char axis1, char axis2, char axis3) {
    if (gSpeed == 1) {                // на подаче
      return durLow3Ax;
    } else if (gSpeed == 0) {         // на ускоренном
      return durLow3AxAccel;
    }
  }
};

SpeedControl speedSetting;           // имплементим класс контроля скорости


class PositionTracking {            // класс отвечает за отслеживание координат при любом перемещении станка
  private:

  int32_t absStepsOfAxisX = 0;      // абсолютное положение шпинделя по оси в шагах от начала координат станка
  int32_t absStepsOfAxisY = 0;
  int32_t absStepsOfAxisZ = 0;

  public:

  void resetPositionX() {             // когда во время настройки (выход в ноль) ось пришла в нулевую позицию, обнуляем переменную, хранящую абсолютные координаты
    absStepsOfAxisX = 0;
  }

  void resetPositionY() {
    absStepsOfAxisY = 0;
  }

  void resetPositionZ() {
    absStepsOfAxisZ = zDistance;      // Полный ход оси Z
  }

  void budgeX() {
    if (xDir) {
      absStepsOfAxisX++;
    } else {
      absStepsOfAxisX--;
    }
  }


  void budgeY() {
    if (yDir) {
      absStepsOfAxisY++;
    } else {
      absStepsOfAxisY--;
    }
  }


  void budgeZ() {
    if (zDir) {
      absStepsOfAxisZ++;
    } else {
      absStepsOfAxisZ--;
    }
  }


  int32_t getPositionX() {          // возвращает текущую позицию по координате X в шагах от начала координат
    return absStepsOfAxisX;
  }

  int32_t getPositionY() {          // возвращает текущую позицию по координате Y в шагах от начала координат
    return absStepsOfAxisY;
  }

  int32_t getPositionZ() {          // возвращает текущую позицию по координате Z в шагах от начала координат
    return absStepsOfAxisZ;
  }



  void printPosition() {
    Serial.print("X: ");
    Serial.print(absStepsOfAxisX * 0.0025, 3);
    Serial.print("; Y: " + String(absStepsOfAxisY * 0.0025, 3));
    Serial.println("; Z: " + String(absStepsOfAxisZ * 0.005, 3));
  }

};


PositionTracking machinePosition;   // имплементим класс слежения за перемещениями по координатам в абсолютной системе коодинат станка


// класс управляет автоматическими перемещениями по осям типа движения к референтной точке или к точке смены инструмента
class AutomaticMove {
  public:
  void moveX(uint16_t delayMoveHigh, uint16_t delayMoveLow) {      // метод движения по оси X
    digitalWrite(pinStepX1, HIGH);
    digitalWrite(pinStepX2, HIGH);
    delayMicroseconds(delayMoveHigh);
    digitalWrite(pinStepX1, LOW);
    digitalWrite(pinStepX2, LOW);
    delayMicroseconds(delayMoveLow);
    machinePosition.budgeX();                           // отслеживаем передвижение в абсолютной системе координат
  }

  void moveY(uint16_t delayMoveHigh, uint16_t delayMoveLow) {      // метод движения по оси Y
    digitalWrite(pinStepY1, HIGH);
    digitalWrite(pinStepY2, HIGH);
    delayMicroseconds(delayMoveHigh);
    digitalWrite(pinStepY1, LOW);
    digitalWrite(pinStepY2, LOW);
    delayMicroseconds(delayMoveLow);
    machinePosition.budgeY();                           // отслеживаем передвижение в абсолютной системе координат
  }

  void moveZ(uint16_t delayMoveHigh, uint16_t delayMoveLow) {      // метод движения по оси Z
    digitalWrite(pinStepZ, HIGH);
    delayMicroseconds(delayMoveHigh);
    digitalWrite(pinStepZ, LOW);
    delayMicroseconds(delayMoveLow);
    machinePosition.budgeZ();                           // отслеживаем передвижение в абсолютной системе координат
  }
};


AutomaticMove aMove;                // имплементим класс автоматического перемещения


class LimitSwitchTuning {           // класс для настройки нулевой точки станка по концевым выключателям
  uint8_t tuneSpeedHigh = 15;       // скорость движения во время выхода в нулевые позиции
  uint16_t tuneSpeedLow = 150;

  public:

  void tuneMachine() {                  // метод выводит станок в нулевую позицию по всем координатам
    digitalWrite(pinDirX1, left);         // устанавливаем направление движения осей по направлению к концевикам подстроечным
    digitalWrite(pinDirX2, left);
    digitalWrite(pinDirY1, back);
    digitalWrite(pinDirY2, back);
    digitalWrite(pinDirZ, top);
    tuningMoveZ();                        // выводим оси в нулевые координаты
    tuningMoveX();
    tuningMoveY();
    machinePosition.resetPositionZ();     // К слову сказать, обнуление координаты Z предполагает присвоение значения не 0, а максимальное
                                          // значение! (верхняя точка)
  }

  void tuningMoveX() {
    while (true) {
      if (digitalRead(pinLimitSwitchX)) {
        machinePosition.resetPositionX();
        Serial.println("finish tuning X axis.");
        break;
      } else {
        digitalWrite(pinStepX1, HIGH);
        digitalWrite(pinStepX2, HIGH);
        delayMicroseconds(tuneSpeedHigh);
        digitalWrite(pinStepX1, LOW);
        digitalWrite(pinStepX2, LOW);
        delayMicroseconds(tuneSpeedLow);
        // Serial.println("move tuning X axis");
      }
    }
  }

  void tuningMoveY() {
    while (true) {
      if (digitalRead(pinLimitSwitchY)) {
        machinePosition.resetPositionY();
        Serial.println("finish tuning Y axis.");
        break;
      } else {
        digitalWrite(pinStepY1, HIGH);
        digitalWrite(pinStepY2, HIGH);
        delayMicroseconds(tuneSpeedHigh);
        digitalWrite(pinStepY1, LOW);
        digitalWrite(pinStepY2, LOW);
        delayMicroseconds(tuneSpeedLow);
        // Serial.println("move tuning Y axis");
      }
    }
  }

  void tuningMoveZ() {
    while (true) {
      if (digitalRead(pinLimitSwitchZ)) {
        machinePosition.resetPositionZ();
        Serial.println("finish tuning Z axis.");
        break;
      } else {
        digitalWrite(pinStepZ, HIGH);
        delayMicroseconds(tuneSpeedHigh);
        digitalWrite(pinStepZ, LOW);
        delayMicroseconds(tuneSpeedLow);
        // Serial.println("move tuning Y axis");
      }
    }
  }
};

LimitSwitchTuning limitSwitchTuning;    // имплементим объект вывода станка в нулевые позиции по всем осям


class synFrameProc {                // класс распределяет шаги в кадре
  public:
  uint32_t timeLastMoveZ = 0;       // время последнего выполнения шага по оси Z. Сравниваем с текущим таймером в микросекундах и принимаем
                                    // решение, не превышена ли скорость.
  const uint32_t minPauseZ = 167;  // минимальная пауза между шагами по оси Z в микросекундах

  void sfpFrameProcessing() {         // метод распределяет шаги в кадре
    if (xStepsFrameABS != 0 && yStepsFrameABS == 0 && zStepsFrameABS == 0) {          // движение только по оси X
      timingMoveX();                // устанавливаем скорость движения и управляем движением по этой оси
    } else if (xStepsFrameABS == 0 && yStepsFrameABS != 0 && zStepsFrameABS == 0) {   // движение только по оси Y
      timingMoveY();                // устанавливаем скорость движения и управляем движением по этой оси
    } else if (xStepsFrameABS == 0 && yStepsFrameABS == 0 && zStepsFrameABS != 0) {   // движение только по оси Z
      timingMoveZ();                // устанавливаем скорость движения и управляем движением по этой оси
    } else if (xStepsFrameABS != 0 && yStepsFrameABS != 0 && zStepsFrameABS == 0) {   // движение по двум осям X и Y
      timingMoveXY();
    } else if (xStepsFrameABS != 0 && yStepsFrameABS == 0 && zStepsFrameABS != 0) {   // движение по двум осям X и Z
      timingMoveXZ();
    } else if (xStepsFrameABS == 0 && yStepsFrameABS != 0 && zStepsFrameABS != 0) {   // движение по двум осям Y и Z
      timingMoveYZ();
    } else if (xStepsFrameABS != 0 && yStepsFrameABS != 0 && zStepsFrameABS != 0) {   // все три оси в движении X, Y и Z
      timingMoveXYZ();
    }
  }


  void timingMoveX() {              // управляем движением по оси X в случае, если движение есть только по одной оси X
    for (uint32_t i = 0; i < xStepsFrameABS; i++) {
      sfpMoveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));              // крутим двигатель по одной только оси X на количество шагов в кадре
    }
  }


  void timingMoveY() {              // управляем движением по оси Y в случае, если движение есть только по одной оси Y
    for (uint32_t i = 0; i < yStepsFrameABS; i++) {
      sfpMoveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));              // крутим двигатель по одной только оси Y на количество шагов в кадре
    }
  }


  void timingMoveZ() {              // управляем движением по оси Z в случае, если движение есть только по одной оси Z
    for (uint32_t i = 0; i < zStepsFrameABS; i++) {
      sfpMoveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));              // крутим двигатель по одной только оси Z на количество шагов в кадре
    }
  }


  void timingMoveXY() {             // управляем движением по двум осям X и Y
    sfpMoveXY(speedSetting.durHighLevel, speedSetting.getSpeed('x', 'y'));               // делаем первые шаги по осям
    // если мастер ось - X (метод masterAxis вернет 1)
    if (masterAxis(xStepsFrameABS, yStepsFrameABS, zStepsFrameABS) == 1) {      // вычисляем мастер-ось (ось, где больше всего шагов в данном кадре)
      // i в for равно единице, а не нулю потому что от запланированного количества шагов в кадре мы строчкой выше
      // уже сделали по одному шагу по всем задействованным в данном кадре осям
      for (uint32_t i = 1; i < xStepsFrameABS; i++) {   // количество тактов в кадре равно количеству шагов в кадре по оси с наибольшим количеством шагов
        if (xStepsDone * 1.0 / xStepsFrameABS < yStepsDone * 1.0 / yStepsFrameABS) {  // коэффициент по мастер-оси меньше второй оси?
          sfpMoveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));          // догоняем мастер ось движением только по одной мастер-оси
        } else {                                                                // а если коэффициенты мастер-оси и второй оси равны, или коэффициент мастер-оси больше
          sfpMoveXY(speedSetting.durHighLevel, speedSetting.getSpeed('x', 'y'));         // двикоем сразу две оси
        }
      }
    } else {
      for (uint32_t i = 1; i < yStepsFrameABS; i++) {                           // количество тактов в кадре равно количеству шагов в кадре по оси с наибольшим количеством шагов
        if (yStepsDone * 1.0 / yStepsFrameABS < xStepsDone * 1.0 / xStepsFrameABS) {  // коэффициент по мастер-оси меньше второй оси?
          sfpMoveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));          // догоняем мастер ось движением только по одной мастер-оси
        } else {                                                                // а если коэффициенты мастер-оси и второй оси равны, или коэффициент мастер-оси больше
          sfpMoveXY(speedSetting.durHighLevel, speedSetting.getSpeed('x', 'y'));         // двикоем сразу две оси
        }
      }
    }
  }


  void timingMoveXZ() {             // управляем движением по двум осям X и Z
    sfpMoveXZ(speedSetting.durHighLevel, speedSetting.getSpeed('x', 'z'));              // делаем первые шаги по осям
    // если мастер ось - X (метод masterAxis вернет 1)
    if (masterAxis(xStepsFrameABS, yStepsFrameABS, zStepsFrameABS) == 1) {      // вычисляем мастер-ось (ось, где больше всего шагов в данном кадре)
      // i в for равно единице, а не нулю потому что от запланированного количества шагов в кадре мы строчкой выше
      // уже сделали по одному шагу по всем задействованным в данном кадре осям
      for (uint32_t i = 1; i < xStepsFrameABS; i++) {                           // количество тактов в кадре равно количеству шагов в кадре по оси с наибольшим количеством шагов
        if (xStepsDone * 1.0 / xStepsFrameABS < zStepsDone * 1.0 / zStepsFrameABS) {    // коэффициент по мастер-оси меньше второй оси?
          sfpMoveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));          // догоняем мастер ось движением только по одной мастер-оси
        } else {                                                                // а если коэффициенты мастер-оси и второй оси равны, или коэффициент мастер-оси больше
          sfpMoveXZ(speedSetting.durHighLevel, speedSetting.getSpeed('x', 'z'));        // двигаем сразу две оси
        }
      }
    } else {
      for (uint32_t i = 1; i < zStepsFrameABS; i++) {                           // количество тактов в кадре равно количеству шагов в кадре по оси с наибольшим количеством шагов
        if (zStepsDone * 1.0 / zStepsFrameABS < xStepsDone * 1.0 / xStepsFrameABS) {    // коэффициент по мастер-оси меньше второй оси?
          sfpMoveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));          // догоняем мастер ось движением только по одной мастер-оси
        } else {                                                                // а если коэффициенты мастер-оси и второй оси равны, или коэффициент мастер-оси больше
          sfpMoveXZ(speedSetting.durHighLevel, speedSetting.getSpeed('x', 'z'));        // двигаем сразу две оси
        }
      }
    }
  }


  void timingMoveYZ() {             // управляем движением по двум осям Y и Z
    sfpMoveYZ(speedSetting.durHighLevel, speedSetting.getSpeed('y', 'z'));              // делаем первые шаги по осям
    // если мастер ось - Y (метод masterAxis вернет 2)
    if (masterAxis(xStepsFrameABS, yStepsFrameABS, zStepsFrameABS) == 2) {      // вычисляем мастер-ось (ось, где больше всего шагов в данном кадре)
      // i в for равно единице, а не нулю потому что от запланированного количества шагов в кадре мы строчкой выше
      // уже сделали по одному шагу по всем задействованным в данном кадре осям
      for (uint32_t i = 1; i < yStepsFrameABS; i++) {                           // количество тактов в кадре равно количеству шагов в кадре по оси с наибольшим количеством шагов
        if (yStepsDone * 1.0 / yStepsFrameABS < zStepsDone * 1.0 / zStepsFrameABS) {    // коэффициент по мастер-оси меньше второй оси?
          sfpMoveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));          // догоняем мастер ось движением только по одной мастер-оси
        } else {                                                                // а если коэффициенты мастер-оси и второй оси равны, или коэффициент мастер-оси больше
          sfpMoveYZ(speedSetting.durHighLevel, speedSetting.getSpeed('y', 'z'));        // двигаем сразу две оси
        }
      }
    } else {
      for (uint32_t i = 1; i < zStepsFrameABS; i++) {                           // количество тактов в кадре равно количеству шагов в кадре по оси с наибольшим количеством шагов
        if (zStepsDone * 1.0 / zStepsFrameABS < yStepsDone * 1.0 / yStepsFrameABS) {    // коэффициент по мастер-оси меньше второй оси?
          sfpMoveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));          // догоняем мастер ось движением только по одной мастер-оси
        } else {                                                                // а если коэффициенты мастер-оси и второй оси равны, или коэффициент мастер-оси больше
          sfpMoveYZ(speedSetting.durHighLevel, speedSetting.getSpeed('y', 'z'));        // двигаем сразу две оси
        }
      }
    }
  }


  void timingMoveXYZ() {
    bool isNeedMoveX = false;                   // флаги, где будет храниться информация о том, надо ли двигаться по данной оси
    bool isNeedMoveY = false;
    bool isNeedMoveZ = false;

    sfpMoveXYZ(speedSetting.durHighLevel, speedSetting.getSpeed('x', 'y', 'z'));           // делаем первые шаги по осям
    // вычисляем мастер-ось
    // если мастер ось - X (метод masterAxis вернет 1)
    if (masterAxis(xStepsFrameABS, yStepsFrameABS, zStepsFrameABS) == 1) {                // мастер-ось X
      isNeedMoveX = true;   // так как мастер-ось всегда двигается в каждом такте, то безо всяких условий в самом начале просто отмечаем, что эту ось нужно двигать
      for (uint32_t i = 1; i < xStepsFrameABS; i++) {     // количество тактов в кадре равно количеству шагов по мастер-оси
        // сначала анализируем одну пару осей, одна из которых мастер-ось
        if (xStepsDone * 1.0 / xStepsFrameABS < yStepsDone * 1.0 / yStepsFrameABS) {      // коэффициент по мастер-оси меньше второй оси?
          isNeedMoveY = false;                                                            // в этой паре осей, в этом такте крутим только мастер-ось
        } else {                                                                          // коэффициент по мастер-оси равен, или больше второй оси?
          isNeedMoveY = true;                                                             // в этой паре осей, в этом такте крутим обе оси
        }
        // теперь анализируем вторую пару осей, одна из которых мастер-ось
        if (xStepsDone * 1.0 / xStepsFrameABS < zStepsDone * 1.0 / zStepsFrameABS) {      // коэффициент по мастер-оси меньше второй оси?
          isNeedMoveZ = false;                                                            // в этой паре осей, в этом такте крутим только мастер-ось
        } else {                                                                          // коэффициент по мастер-оси равен, или больше второй оси?
          isNeedMoveZ = true;                                                             // в этой паре осей, в этом такте крутим обе оси
        }
        // информация о том, какие оси в данном такте необходимо двигать, а какие должны остаться неподвижны, собрана.
        // Передаем эту информацию в метод, который на основе этой информации запустит нужные оси в движение
        execMove(isNeedMoveX, isNeedMoveY, isNeedMoveZ);
      }
    } else if (masterAxis(xStepsFrameABS, yStepsFrameABS, zStepsFrameABS) == 2) {         // мастер-ось Y
      isNeedMoveY = true;   // так как мастер-ось всегда двигается в каждом такте, то безо всяких условий в самом начале просто отмечаем, что эту ось нужно двигать

      for (uint32_t i = 1; i < yStepsFrameABS; i++) {     // количество тактов в кадре равно количеству шагов по мастер-оси
        // сначала анализируем одну пару осей, одна из которых мастер-ось
        if (yStepsDone * 1.0 / yStepsFrameABS < xStepsDone * 1.0 / xStepsFrameABS) {      // коэффициент по мастер-оси меньше второй оси?
          isNeedMoveX = false;                                                            // в этой паре осей, в этом такте крутим только мастер-ось
        } else {                                                                          // коэффициент по мастер-оси равен, или больше второй оси?
          isNeedMoveX = true;                                                             // в этой паре осей, в этом такте крутим обе оси
        }
        // теперь анализируем вторую пару осей, одна из которых мастер-ось
        if (yStepsDone * 1.0 / yStepsFrameABS < zStepsDone * 1.0 / zStepsFrameABS) {      // коэффициент по мастер-оси меньше второй оси?
          isNeedMoveZ = false;                                                            // в этой паре осей, в этом такте крутим только мастер-ось
        } else {                                                                          // коэффициент по мастер-оси равен, или больше второй оси?
          isNeedMoveZ = true;                                                             // в этой паре осей, в этом такте крутим обе оси
        }
        // информация о том, какие оси в данном такте необходимо двигать, а какие должны остаться неподвижны, собрана.
        // Передаем эту информацию в метод, который на основе этой информации запустит нужные оси в движение
        execMove(isNeedMoveX, isNeedMoveY, isNeedMoveZ);
      }
    } else if (masterAxis(xStepsFrameABS, yStepsFrameABS, zStepsFrameABS) == 3) {      // мастер-ось Z
      isNeedMoveZ = true;   // так как мастер-ось всегда двигается в каждом такте, то безо всяких условий в самом начале просто отмечаем, что эту ось нужно двигать

      for (uint32_t i = 1; i < zStepsFrameABS; i++) {     // количество тактов в кадре равно количеству шагов по мастер-оси
        // сначала анализируем одну пару осей, одна из которых мастер-ось
        if (zStepsDone * 1.0 / zStepsFrameABS < yStepsDone * 1.0 / yStepsFrameABS) {      // коэффициент по мастер-оси меньше второй оси?
          isNeedMoveY = false;                                                            // в этой паре осей, в этом такте крутим только мастер-ось
        } else {                                                                          // коэффициент по мастер-оси равен, или больше второй оси?
          isNeedMoveY = true;                                                             // в этой паре осей, в этом такте крутим обе оси
        }
        // теперь анализируем вторую пару осей, одна из которых мастер-ось
        if (zStepsDone * 1.0 / zStepsFrameABS < xStepsDone * 1.0 / xStepsFrameABS) {      // коэффициент по мастер-оси меньше второй оси?
          isNeedMoveX = false;                                                            // в этой паре осей, в этом такте крутим только мастер-ось
        } else {                                                                          // коэффициент по мастер-оси равен, или больше второй оси?
          isNeedMoveX = true;                                                             // в этой паре осей, в этом такте крутим обе оси
        }
        // информация о том, какие оси в данном такте необходимо двигать, а какие должны остаться неподвижны, собрана.
        // Передаем эту информацию в метод, который на основе этой информации запустит нужные оси в движение
        execMove(isNeedMoveX, isNeedMoveY, isNeedMoveZ);
      }
    }
  }


  // метод вычисляет мастер-ось
  uint8_t masterAxis(uint32_t xStepsFrameABS, uint32_t yStepsFrameABS, uint32_t zStepsFrameABS) {
    uint8_t masterAxis = 0;                     // здесь храним текущее значение о том, какая ось мастер (в том числе и промежуточные значения вычислений)
    /* 1 - X master
     * 2 - Y master
     * 3 - Z master
     */
    if (xStepsFrameABS >= yStepsFrameABS) {
      masterAxis = 1;                           // промежуточное значение: мастер ось - X
      if (xStepsFrameABS >= zStepsFrameABS) {   // сравниваем промежуточное значение с осью Z
        masterAxis = 1;                         // финальное значение: мастер ось - X
      } else {
        masterAxis = 3;                         // финальное значение: мастер ось - Z
      }
    } else {
      masterAxis = 2;                           // промежуточное значение: мастер ось - Y
      if (yStepsFrameABS >= zStepsFrameABS) {   // сравниваем промежуточное значение с осью Z
        masterAxis = 2;                         // финальное значение: мастер ось - Y
      } else {
        masterAxis = 3;                         // финальное значение: мастер ось - Z
      }
    }
    return masterAxis;
  }


  // метод, при одновременном движении всех трех осей, приводит в движение оси, необходимость движения которых указана в параметрах
  void execMove(bool isNeedMoveX, bool isNeedMoveY, bool isNeedMoveZ) {
    if (isNeedMoveX && isNeedMoveY && isNeedMoveZ) {            // если нужно крутить все оси
      sfpMoveXYZ(speedSetting.durHighLevel, speedSetting.getSpeed('x', 'y', 'z'));
    } else if (isNeedMoveX && !isNeedMoveY && !isNeedMoveZ) {   // крутить надо только ось X
      sfpMoveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
    } else if (!isNeedMoveX && isNeedMoveY && !isNeedMoveZ) {   // крутить надо только ось Y
      sfpMoveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
    } else if (!isNeedMoveX && !isNeedMoveY && isNeedMoveZ) {   // крутить надо только ось Z
      sfpMoveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));
    } else if (isNeedMoveX && isNeedMoveY && !isNeedMoveZ) {   // крутить надо две оси: X и Y
      sfpMoveXY(speedSetting.durHighLevel, speedSetting.getSpeed('x', 'y'));
    } else if (isNeedMoveX && !isNeedMoveY && isNeedMoveZ) {   // крутить надо две оси: X и Z
      sfpMoveXZ(speedSetting.durHighLevel, speedSetting.getSpeed('x', 'z'));
    } else if (!isNeedMoveX && isNeedMoveY && isNeedMoveZ) {   // крутить надо две оси: Y и Z
      sfpMoveYZ(speedSetting.durHighLevel, speedSetting.getSpeed('y', 'z'));
    }
  }


  void sfpMoveX(uint16_t moveSpeedHigh, uint16_t moveSpeedLow) {     // метод вращает двигатель только по одной оси X
    digitalWrite(pinStepX1, HIGH);
    digitalWrite(pinStepX2, HIGH);
    delayMicroseconds(moveSpeedHigh);
    digitalWrite(pinStepX1, LOW);
    digitalWrite(pinStepX2, LOW);
    delayMicroseconds(moveSpeedLow);
    xStepsDone++;                  // считаем количество фактически выполненных шагов в текущем кадре
    xDoneToTime++;                 // считаем количества шагов от начала программы и до текущего момента
    machinePosition.budgeX();      // отслеживаем перемещение в абсолютной системе координат станка
    /*
    Serial.print("step X: ");
    Serial.println(xStepsDone);
    */
  }


  void sfpMoveY(uint16_t moveSpeedHigh, uint16_t moveSpeedLow) {     // метод вращает двигатель только по одной оси Y
    digitalWrite(pinStepY1, HIGH);
    digitalWrite(pinStepY2, HIGH);
    delayMicroseconds(moveSpeedHigh);
    digitalWrite(pinStepY1, LOW);
    digitalWrite(pinStepY2, LOW);
    delayMicroseconds(moveSpeedLow);
    yStepsDone++;                  // считаем количество фактически выполненных шагов в текущем кадре
    yDoneToTime++;                 // считаем количества шагов от начала программы и до текущего момента
    machinePosition.budgeY();      // отслеживаем перемещение в абсолютной системе координат станка
    /*
    Serial.print("step Y: ");
    Serial.println(yStepsDone);
    */
  }


  void sfpMoveZ(uint16_t moveSpeedHigh, uint16_t moveSpeedLow) {     // метод вращает двигатель только по одной оси Z

    while (micros() - timeLastMoveZ < minPauseZ) {            // если прошлый шаг по этой оси был выполнен меньше, чем минимально необходимое
                                                              // время (т.е. превышена скорость)
      if (micros() - timeLastMoveZ < 0) {                     // проверяем, не обнулился ли таймер
        delay(1);                                               // елси обнулился - эта разность будет меньше нуля. Ждем одну миллисекунду
        break;                                                  // выходим из цикла ожидания
      }
      delayMicroseconds(1);                                   // таймер не превышен. Просто ждем 1 микросекунду и снова проверяем время
    }

    digitalWrite(pinStepZ, HIGH);
    delayMicroseconds(moveSpeedHigh);
    digitalWrite(pinStepZ, LOW);
    delayMicroseconds(moveSpeedLow / 2);      // moveSpeedLow / 2 потому что винт по Z однозаходный и чтобы достичь скорости движения других осей, нужно крутиться в два раза быстрее
    zStepsDone++;                  // считаем количество фактически выполненных шагов в текущем кадре
    zDoneToTime++;                 // считаем количества шагов от начала программы и до текущего момента
    machinePosition.budgeZ();      // отслеживаем перемещение в абсолютной системе координат станка
    timeLastMoveZ = micros();      // фиксируем время, когда был сделан этот шаг
    /*
    Serial.print("step Z: ");
    Serial.println(zStepsDone);
    */
  }


  void sfpMoveXY(uint16_t moveSpeedHigh, uint16_t moveSpeedLow) {     // метод вращает двигатель по двум осям: X и Y
    digitalWrite(pinStepX1, HIGH);
    digitalWrite(pinStepX2, HIGH);
    digitalWrite(pinStepY1, HIGH);
    digitalWrite(pinStepY2, HIGH);
    delayMicroseconds(moveSpeedHigh);
    digitalWrite(pinStepX1, LOW);
    digitalWrite(pinStepX2, LOW);
    digitalWrite(pinStepY1, LOW);
    digitalWrite(pinStepY2, LOW);
    delayMicroseconds(moveSpeedLow);
    xStepsDone++;                  // считаем количество фактически выполненных шагов в текущем кадре
    yStepsDone++;
    xDoneToTime++;                 // считаем количества шагов от начала программы и до текущего момента
    yDoneToTime++;
    machinePosition.budgeX();      // отслеживаем перемещение в абсолютной системе координат станка
    machinePosition.budgeY();
    /*
    Serial.print("step X: ");
    Serial.print(xStepsDone);
    Serial.print("; step Y: ");
    Serial.println(yStepsDone);
    */
  }


  void sfpMoveXZ(uint16_t moveSpeedHigh, uint16_t moveSpeedLow) {     // метод вращает двигатель по двум осям: X и Z
    digitalWrite(pinStepX1, HIGH);
    digitalWrite(pinStepX2, HIGH);
    digitalWrite(pinStepZ, HIGH);
    delayMicroseconds(moveSpeedHigh);
    digitalWrite(pinStepX1, LOW);
    digitalWrite(pinStepX2, LOW);
    digitalWrite(pinStepZ, LOW);
    delayMicroseconds(moveSpeedLow);
    xStepsDone++;                  // считаем количество фактически выполненных шагов в текущем кадре
    zStepsDone++;
    xDoneToTime++;                 // считаем количества шагов от начала программы и до текущего момента
    zDoneToTime++;
    machinePosition.budgeX();      // отслеживаем перемещение в абсолютной системе координат станка
    machinePosition.budgeZ();
    /*
    Serial.print("step X: ");
    Serial.print(xStepsDone);
    Serial.print("; step Z: ");
    Serial.println(zStepsDone);
    */
  }


  void sfpMoveYZ(uint16_t moveSpeedHigh, uint16_t moveSpeedLow) {     // метод вращает двигатель по двум осям: Y и Z
    digitalWrite(pinStepY1, HIGH);
    digitalWrite(pinStepY2, HIGH);
    digitalWrite(pinStepZ, HIGH);
    delayMicroseconds(moveSpeedHigh);
    digitalWrite(pinStepY1, LOW);
    digitalWrite(pinStepY2, LOW);
    digitalWrite(pinStepZ, LOW);
    delayMicroseconds(moveSpeedLow);
    yStepsDone++;                  // считаем количество фактически выполненных шагов в текущем кадре
    zStepsDone++;
    yDoneToTime++;                 // считаем количества шагов от начала программы и до текущего момента
    zDoneToTime++;
    machinePosition.budgeY();      // отслеживаем перемещение в абсолютной системе координат станка
    machinePosition.budgeZ();
    /*
    Serial.print("step Y: ");
    Serial.print(yStepsDone);
    Serial.print("; step Z: ");
    Serial.println(zStepsDone);
    */
  }


  void sfpMoveXYZ(uint16_t moveSpeedHigh, uint16_t moveSpeedLow) {     // метод вращает двигатель по трем осям: X, Y и Z
    digitalWrite(pinStepX1, HIGH);
    digitalWrite(pinStepX2, HIGH);
    digitalWrite(pinStepY1, HIGH);
    digitalWrite(pinStepY2, HIGH);
    digitalWrite(pinStepZ, HIGH);
    delayMicroseconds(moveSpeedHigh);
    digitalWrite(pinStepX1, LOW);
    digitalWrite(pinStepX2, LOW);
    digitalWrite(pinStepY1, LOW);
    digitalWrite(pinStepY2, LOW);
    digitalWrite(pinStepZ, LOW);
    delayMicroseconds(moveSpeedLow);
    xStepsDone++;                  // считаем количество фактически выполненных шагов в текущем кадре
    yStepsDone++;
    zStepsDone++;
    xDoneToTime++;                 // считаем количества шагов от начала программы и до текущего момента
    yDoneToTime++;
    zDoneToTime++;
    machinePosition.budgeX();      // отслеживаем перемещение в абсолютной системе координат станка
    machinePosition.budgeY();
    machinePosition.budgeZ();
    /*
    Serial.print("step X: ");
    Serial.print(xStepsDone);
    Serial.print("; step Y: ");
    Serial.print(yStepsDone);
    Serial.print("; step Z: ");
    Serial.println(zStepsDone);
    */
  }
};


class ManualControl {     // класс ручного управления перемещением по осям
  public:
  uint16_t stepsInWeaving = 4;        // при микрошаге 1/16 в сотке помещается 8 шагов
                                      // при микрошаге 1/8  в сотке 4 шага
                                      // при микрошаке 2/4 в сотке помещается 2 шага
  bool stepByStep = false;            // переменные хранят состояние тумблеров скорости на пульте управления
  bool fiSpeed = false;
  bool seSpeed = false;
  bool thSpeed = false;

  //fsval расшифровывается как first speed value
  // скорости (значения задержки) вращения моторов в ручном режиме
  // при микрошаге 1/8
  uint16_t curSpeed = 0;                // текущая скорость - отправляется на двигатели (изменяется методами разгона и торможения)
  const uint16_t firstSpeedManual = 8000;   // пауза низкого уровня сигнала (она и формирует скорость движения)
  const uint16_t secondSpeedManual = 200;
  const uint16_t thirdSpeedManual = 100;
  const uint16_t highsval = 8;          // пауза для высокого уровня сигнала (для всех скоростей одинаковая)

  // метод возвращает скорость вращения двигателей по осям
  uint16_t getSpeedRotation(uint8_t needSpeed) {
    // needSpeed - параметр определяет насколько быстро нужно двигаться (первая, вторая, третья скорости)
    // curMicroStep - параметр определяет какое сейчас установлено деление микрошага (1/4, 1/18, 1/16)

    if (needSpeed == 1) {             // запрашивается 1-ая скорость
      return firstSpeedManual;
    } else if (needSpeed == 2) {      // запрашивается 2-ая скорость
      return secondSpeedManual;
    } else if (needSpeed == 3) {        // запрашивается 3-я скорость
      return thirdSpeedManual;
    }
  }


  void toLeftHandler() {        // обработчик нажатия кнопки "Влево"

    digitalWrite(pinDirX1, left);   // устанавливаем значение пинов, соответствующее данному направлению
    digitalWrite(pinDirX2, left);
    xDir = leftFlag;

    if (stepByStep && !fiSpeed && !seSpeed && !thSpeed) {         // если включен тумблер "по сотке" и больше никаеие тумблеры не включены
      oneWeaving(pinToLeft);                                      // делаем движение на одну сотку
    } else if (fiSpeed && !seSpeed && !thSpeed) {                 // включена первая скорость
      axisMovement(getSpeedRotation(1), pinToLeft);               // вращаем мотор, пока нажата кнопка
      // Serial.println("first speed");
    } else if (seSpeed && !thSpeed) {                             // включена вторая скорость
      axisMovement(getSpeedRotation(2), pinToLeft);                             // вращаем мотор, пока нажата кнопка
      // Serial.println("second speed");
    } else if (thSpeed) {                                         // включена третья скорость
      axisMovement(getSpeedRotation(3), pinToLeft);                             // вращаем мотор, пока нажата кнопка
      // Serial.println("third speed");
    }
  }


  void toRightHandler() {        // обработчик нажатия кнопки "Вправо"

    digitalWrite(pinDirX1, right);   // устанавливаем значение пинов, соответствующее данному направлению
    digitalWrite(pinDirX2, right);
    xDir = rightFlag;

    if (stepByStep && !fiSpeed && !seSpeed && !thSpeed) {         // если включен тумблер "по сотке" и больше никаеие тумблеры не включены
      oneWeaving(pinToRight);                                  // делаем движение на одну сотку
    } else if (fiSpeed && !seSpeed && !thSpeed) {                 // включена первая скорость, а другие тумблеры кроме stepByStep выключены?
      axisMovement(getSpeedRotation(1), pinToRight);                            // вращаем мотор, пока нажата кнопка
    } else if (seSpeed && !thSpeed) {                             // включена вторая скорость, а другие тумблеры кроме stepByStep выключены?
      axisMovement(getSpeedRotation(2), pinToRight);                            // вращаем мотор, пока нажата кнопка
    } else if (thSpeed) {                                         // включена третья скорость, а другие тумблеры кроме stepByStep выключены?
      axisMovement(getSpeedRotation(3), pinToRight);                            // вращаем мотор, пока нажата кнопка
    }
  }


  void toForwardHandler() {         // обработчик нажатия кнопки "Вперед"

    digitalWrite(pinDirY1, back);   // устанавливаем значение пинов, соответствующее данному направлению
    digitalWrite(pinDirY2, back);
    yDir = backFlag;

    if (stepByStep && !fiSpeed && !seSpeed && !thSpeed) {         // если включен тумблер "по сотке" и больше никаеие тумблеры не включены
      oneWeaving(pinToForward);                                   // делаем движение на одну сотку
    } else if (fiSpeed && !seSpeed && !thSpeed) {                 // включена первая скорость, а другие тумблеры кроме stepByStep выключены?
      axisMovement(getSpeedRotation(1), pinToForward);                          // вращаем мотор, пока нажата кнопка
    } else if (seSpeed && !thSpeed) {                             // включена вторая скорость, а другие тумблеры кроме stepByStep выключены?
      axisMovement(getSpeedRotation(2), pinToForward);                          // вращаем мотор, пока нажата кнопка
    } else if (thSpeed) {                                         // включена третья скорость, а другие тумблеры кроме stepByStep выключены?
      axisMovement(getSpeedRotation(3), pinToForward);                          // вращаем мотор, пока нажата кнопка
    }
  }


  void toBackHandler() {            // обработчик нажатия кнопки "Назад"

    digitalWrite(pinDirY1, forward);   // устанавливаем значение пинов, соответствующее данному направлению
    digitalWrite(pinDirY2, forward);
    yDir = forwardFlag;

    if (stepByStep && !fiSpeed && !seSpeed && !thSpeed) {         // если включен тумблер "по сотке" и больше никаеие тумблеры не включены
      oneWeaving(pinToBack);                                      // делаем движение на одну сотку
    } else if (fiSpeed && !seSpeed && !thSpeed) {                 // включена первая скорость, а другие тумблеры кроме stepByStep выключены?
      axisMovement(getSpeedRotation(1), pinToBack);                             // вращаем мотор, пока нажата кнопка
    } else if (seSpeed && !thSpeed) {                             // включена вторая скорость, а другие тумблеры кроме stepByStep выключены?
      axisMovement(getSpeedRotation(2), pinToBack);                             // вращаем мотор, пока нажата кнопка
    } else if (thSpeed) {                                         // включена третья скорость, а другие тумблеры кроме stepByStep выключены?
      axisMovement(getSpeedRotation(3), pinToBack);                             // вращаем мотор, пока нажата кнопка
    }
  }


  void toTopHandler() {            // обработчик нажатия кнопки "Вверх"

    digitalWrite(pinDirZ, top);   // устанавливаем значение пина, соответствующее данному направлению
    zDir = topFlag;

    if (stepByStep && !fiSpeed && !seSpeed && !thSpeed) {         // если включен тумблер "по сотке" и больше никаеие тумблеры не включены
      oneWeaving(pinToTop);                                       // делаем движение на одну сотку
    } else if (fiSpeed && !seSpeed && !thSpeed) {                 // включена первая скорость, а другие тумблеры кроме stepByStep выключены?
      axisMovement(getSpeedRotation(1), pinToTop);                              // вращаем мотор, пока нажата кнопка
    } else if (seSpeed && !thSpeed) {                             // включена вторая скорость, а другие тумблеры кроме stepByStep выключены?
      // getSpeedRotation умножаем на 4, потому что ось Z отдельно от других работает с микрошагом в 4 раза меньше (ограничение в максимальной частоте двигателя ДШИ 200
      axisMovement(getSpeedRotation(2), pinToTop);                              // вращаем мотор, пока нажата кнопка
    } else if (thSpeed) {                                         // включена третья скорость, а другие тумблеры кроме stepByStep выключены?
      axisMovement(getSpeedRotation(3), pinToTop);                              // вращаем мотор, пока нажата кнопка
    }
  }


  void toBottomHandler() {            // обработчик нажатия кнопки "Вниз"

    digitalWrite(pinDirZ, bottom);   // устанавливаем значение пина, соответствующее данному направлению
    zDir = bottomFlag;

    if (stepByStep && !fiSpeed && !seSpeed && !thSpeed) {         // если включен тумблер "по сотке" и больше никаеие тумблеры не включены
      oneWeaving(pinToBottom);                                    // делаем движение на одну сотку
    } else if (fiSpeed && !seSpeed && !thSpeed) {                 // включена первая скорость, а другие тумблеры кроме stepByStep выключены?
      axisMovement(getSpeedRotation(1), pinToBottom);                           // вращаем мотор, пока нажата кнопка
    } else if (seSpeed && !thSpeed) {                             // включена вторая скорость, а другие тумблеры кроме stepByStep выключены?
      axisMovement(getSpeedRotation(2), pinToBottom);                           // вращаем мотор, пока нажата кнопка
    } else if (thSpeed) {                                         // включена третья скорость, а другие тумблеры кроме stepByStep выключены?
      axisMovement(getSpeedRotation(3), pinToBottom);                           // вращаем мотор, пока нажата кнопка
    }
  }


  void axisMovement (uint16_t rotationSpeed, uint8_t pressedButton) {     // метод вращает мотор пока нажата кнопка
    uint8_t qAccel = 60;                                 // число микросекунд прибавляемое на каждом следующем шаге при разгоне
    uint16_t minSpeed = 2000;                           // скорость, с которой начинается разгон
    uint32_t widthXAxis = 132000;                       // ширина рабочей части оси X в шагах
    uint32_t lengthYAxis = 98000;                       // длина рабочей части оси Y в шагах
    uint8_t breakDist = 237;                            // длина тормозного пути в шагах (зависит от коэффициента замедления)
    uint8_t stopOutWorkArea = digitalRead(pinEnTuning); // проверяем, включено ли ограничение выхода за пределы рабочего пространства
    Serial.print("stopOutWorkArea: ");
    Serial.println(stopOutWorkArea);
    while (digitalRead(pressedButton)) {                // отлавливаем конец нажатия кнопки
      if (stopOutWorkArea && pressedButton == pinToLeft && machinePosition.getPositionX() <= 0 + breakDist) {
        Serial.println("out of range! axis X to left");
        break;
      }
      if (stopOutWorkArea && pressedButton == pinToRight && machinePosition.getPositionX() >= widthXAxis - breakDist) {
        Serial.println("out of range! axis X to right");
        break;
      }
      if (stopOutWorkArea && pressedButton == pinToForward && machinePosition.getPositionY() <= 0 + breakDist) {
        Serial.println("out of range! axis Y to forward");
        break;
      }
      if (stopOutWorkArea && pressedButton == pinToBack && machinePosition.getPositionY() >= lengthYAxis - breakDist) {
        Serial.println("out of range! axis Y to back");
        break;
      }
      if (stopOutWorkArea && pressedButton == pinToTop && machinePosition.getPositionZ() >= zDistance - breakDist) {
        Serial.println("out of range! axis Z to top");
        break;
      }
      if (stopOutWorkArea && pressedButton == pinToBottom && machinePosition.getPositionZ() <= 0 + breakDist) {
        Serial.println("out of range! axis Z to bottom");
        break;
      }
      // создаем разгон
      if (rotationSpeed < firstSpeedManual) {          // если это не первая скорость
        if (curSpeed == 0) {                            // если это еще самое начало движения
          curSpeed = minSpeed;                          // инициализируем текущую скорость как минимальную
        }
        if (curSpeed > rotationSpeed) {                 // разгоняемся до нужной скорости уменьшая длину такта
          curSpeed = curSpeed - (curSpeed / qAccel);
        }

      } else {                                          // если это первая
        curSpeed = rotationSpeed;                       // без разгона двигаемся с данной скорости
      }
      distributeAxis(curSpeed, pressedButton);          // вращаем двигатели (этот метод и определяет, по каким осям двигатели будут вращаться, и вращает их)                                   // сбрасываем значение текущей скорости
    }
    // теперь кнопка отпущена и мы начинаем замедление
    if (digitalRead(pinEnTuning) && pressedButton == pinToLeft && machinePosition.getPositionX() >= 0 + breakDist) {
      slowdown(rotationSpeed, minSpeed, qAccel, pressedButton);
    }
    if (digitalRead(pinEnTuning) && pressedButton == pinToRight && machinePosition.getPositionX() <= widthXAxis - breakDist) {
      slowdown(rotationSpeed, minSpeed, qAccel, pressedButton);
    }
    if (digitalRead(pinEnTuning) && pressedButton == pinToForward && machinePosition.getPositionY() >= 0 + breakDist) {
      slowdown(rotationSpeed, minSpeed, qAccel, pressedButton);
    }
    if (digitalRead(pinEnTuning) && pressedButton == pinToBack && machinePosition.getPositionY() <= lengthYAxis - breakDist) {
      slowdown(rotationSpeed, minSpeed, qAccel, pressedButton);
    }
    if (digitalRead(pinEnTuning) && pressedButton == pinToTop && machinePosition.getPositionZ() <= zDistance - breakDist) {
      slowdown(rotationSpeed, minSpeed, qAccel, pressedButton);
    }
    if (digitalRead(pinEnTuning) && pressedButton == pinToBottom && machinePosition.getPositionZ() >= 0 + breakDist) {
      slowdown(rotationSpeed, minSpeed, qAccel, pressedButton);
    }
    curSpeed = 0;                                       // обнуляем текущую скорость
  }

  // метод замедляет движение по оси в ручном режиме
  void slowdown(uint16_t rotationSpeed, uint16_t minSpeed, uint8_t qAccel, uint8_t pressedButton) {
    if (rotationSpeed < firstSpeedManual) {             // если это не первая скорость (на первой нет ускорения и замедления)
      while (curSpeed < minSpeed) {
        if (curSpeed >= qAccel) {                       // чтобы не прибавляло ноль и не зависало
          curSpeed = curSpeed + (curSpeed / qAccel);    // замедляемся
        } else {
          curSpeed += 10;                               // увеличиваем curSpeed на 10, чтобы при следующих делениях curSpeed был меньше qAccel
        }
        distributeAxis(curSpeed, pressedButton);          // вращаем двигатели (этот метод и определяет, по каким осям двигатели будут вращаться, и вращает их)
      }
    }
  }

  // метод совершает движение по оси длиной в одну сотую миллиметра (на первой скорости - самой медленной)
  void oneWeaving (uint8_t pressedButton) {
    for (uint16_t i = 0; i < stepsInWeaving; i++) {    // делаем столько шагов, сколько есть в сотке с учетом выставленного микрошага
      distributeAxis(getSpeedRotation(1), pressedButton);            // вращаем двигатель на первой скорости (самой медленной)
    }

    while (true) {                      // ждем, когда оператор отпустит кнопку, чтобы обеспечить одну сотку расстояния за одно нажатие
      if (!digitalRead(pressedButton)) {
        break;
      }
    }
  }


  void distributeAxis(uint16_t rotationSpeed, uint8_t pressedButton) {      // метод определяет, по какой из осей сейчас будут вращаться двигатели
    // Serial.println(rotationSpeed);
    if (pressedButton == pinToLeft || pressedButton == pinToRight) {                // нажата одна из кнопок: "Влево" или "Вправо"
      stepXManual(rotationSpeed, highsval);                                         // делаем шаг по оси X
    } else if (pressedButton == pinToForward || pressedButton == pinToBack) {       // нажата одна из кнопок: "Вперед" или "Назад"
      stepYManual(rotationSpeed, highsval);                                         // делаем шаг по оси Y
    } else if (pressedButton == pinToTop || pressedButton == pinToBottom) {         // нажата одна из кнопок: "Вверх" или "Вниз"
      stepZManual(rotationSpeed, highsval);                                         // делаем шаг по оси Z
    }
  }


  void quizTumblerState() {     // метод опрашивает состояние кругового переключателя скоростей ручного перемещения
    stepByStep = false;       // обнуляем переменные скоростей, чтобы ложно положительные показания не лезли из прошлых итераций
    fiSpeed = false;
    seSpeed = false;
    thSpeed = false;
    if (digitalRead(pinStepByStep)) {stepByStep = true;}    // опрашиваем пины тумблеров
    if (digitalRead(pinFiSpeed)) {fiSpeed = true;}
    if (digitalRead(pinSeSpeed)) {seSpeed = true;}
    if (digitalRead(pinThSpeed)) {thSpeed = true;}
  }


  void stepXManual(uint16_t speedDelay, uint16_t highSignalDelay) {                      // метод движения по X в ручном режиме
    PORTE |= 1 << PORTE4;                 // подаем высокий уровень сигнала на первый мотор
    PORTE |= 1 << PORTE5;                 // подаем высокий уровень сигнала на второй мотор
    delayMicroseconds(highSignalDelay);
    PORTE &= ~(1 << PORTE4);              // подаем низкий уровень сигнала на первый мотор
    PORTE &= ~(1 << PORTE5);              // подаем низкий уровень сигнала на второй мотор
    delayMicroseconds(speedDelay);
    machinePosition.budgeX();                           // отслеживаем передвижение в абсолютной системе координат
  }


  void stepYManual(uint16_t speedDelay, uint16_t highSignalDelay) {                      // метод движения по Y в ручном режиме
    PORTG |= 1 << PORTG5;                 // подаем высокий уровень сигнала на первый мотор
    PORTB |= 1 << PORTB6;                 // подаем высокий уровень сигнала на второй мотор
    delayMicroseconds(highSignalDelay);
    PORTG &= ~(1 << PORTG5);              // подаем низкий уровень сигнала на первый мотор
    PORTB &= ~(1 << PORTB6);              // подаем низкий уровень сигнала на второй мотор
    delayMicroseconds(speedDelay);
    machinePosition.budgeY();                           // отслеживаем передвижение в абсолютной системе координат
  }


  void stepZManual(uint16_t speedDelay, uint16_t highSignalDelay) {                      // метод движения по Z в ручном режиме
    PORTH |= 1 << PORTH6;                 // подаем высокий уровень сигнала на мотор Z
    delayMicroseconds(highSignalDelay);
    PORTH &= ~(1 << PORTH6);              // подаем низкий уровень сигнала на мотор Z
    delayMicroseconds(speedDelay);
    machinePosition.budgeZ();                           // отслеживаем передвижение в абсолютной системе координат
  }


  void isOnManual() {            // метод проверяет
    if (digitalRead(pinToLeft)) {              // нажата кнопка "Влево"?
      // после нажатия кнопки ручного перемещения опрашиваем состояние всех положений рукоятки скорости
      //(выясняем, на какой скорости сейчас надо вигаться)
      quizTumblerState();
      toLeftHandler();                              // активируем обработчик кнопки "Влево"
    } else if (digitalRead(pinToRight)) {           // нажата кнопка "Вправо"?
      // после нажатия кнопки ручного перемещения опрашиваем состояние всех положений рукоятки скорости
      //(выясняем, на какой скорости сейчас надо вигаться)
      quizTumblerState();
      toRightHandler();                             // активируем обработчик кнопки "Вправо"
    } else if (digitalRead(pinToForward)) {         // нажата кнопка "Вперед"?
      // после нажатия кнопки ручного перемещения опрашиваем состояние всех положений рукоятки скорости
      //(выясняем, на какой скорости сейчас надо вигаться)
      quizTumblerState();
      toForwardHandler();                           // активируем обработчик кнопки "Вперед"
    } else if (digitalRead(pinToBack)) {            // нажата кнопка "Назад"?
      // после нажатия кнопки ручного перемещения опрашиваем состояние всех положений рукоятки скорости
      //(выясняем, на какой скорости сейчас надо вигаться)
      quizTumblerState();
      toBackHandler();                              // активируем обработчик кнопки "Назад"
    } else if (digitalRead(pinToTop)) {             // нажата кнопка "Вверх"?
      // после нажатия кнопки ручного перемещения опрашиваем состояние всех положений рукоятки скорости
      //(выясняем, на какой скорости сейчас надо вигаться)
      quizTumblerState();
      toTopHandler();                               // активируем обработчик кнопки "Вверх"
    } else if (digitalRead(pinToBottom)) {          // нажата кнопка "Вниз"?
      // после нажатия кнопки ручного перемещения опрашиваем состояние всех положений рукоятки скорости
      //(выясняем, на какой скорости сейчас надо вигаться)
      quizTumblerState();
      toBottomHandler();                            // активируем обработчик кнопки "Вниз"
    }
  }
};


ManualControl mControl;             // имплементим класс ManualControl


class ToolChangePoint {
  public:
  int32_t changePointX = 0;              // координаты точки смены инструмента
  int32_t changePointY = 0;
  int32_t changePointZ = 0;
  int32_t endTool = 0;                   // координаты шпинделя по Z в момент, когда нижняя часть фрезы находится в точке смены инструмента

  bool toolLenComp() {
    // сделать эту функцию и посмотреть в каких еще процессах она задействована
  }

  // метод устанавливает координаты точки смены инструмента
  void setToolSensorPoint() {
    // поиск координат датчика касания инструмента начинается включением тумблера pinSetToolSensor - пин 40
    // для инициализации датчика надо вручную подвести шпиндель над датчиком и нажать тумблер pinSetToolSensor - движение в этой
    // функции это вниз до касания датчика. Записываются все три координаты - это и есть точка сверки длин инструментов
    // После касания инструментом датчика, поднимаем инструмент на пару миллиметров Z
    moveToSensorPoint();                    // опускаем инструмент до датчика касания инструмента
  }

  // метод опускает шпиндель до касания с датчиком инструмента
  void moveToSensorPoint() {
    gSpeed = 1;                             // двигаемся на подаче
    fSpeed = 100;                           // скорость 100 мм/с
    speedSetting.setSpeed();                // рассчитываем параметры железа для этой скорости
    zDir = bottomFlag;                      // устанавливаем флаг направления движения "Вниз"
    digitalWrite(pinDirZ, bottom);          // устанавливаем значение пина, соответствующее данному направлению
    // датчик работает на размыкание при нажатии: двигаем шпиндель вниз пока не разомкнет контакты ToolTouchDetected
    while (digitalRead(ToolTouchDetected)) {
        aMove.moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));
    }
    // всё, инструмент коснулся датчика, записываем все три координаты этого местоположения
    changePointX = machinePosition.getPositionX();
    changePointY = machinePosition.getPositionY();
    changePointZ = machinePosition.getPositionZ();
    Serial.print("Tool touch sensor position X: ");
    Serial.println(changePointX);
    Serial.print("Tool touch sensor position Y: ");
    Serial.println(changePointY);
    Serial.print("Tool touch sensor position Z: ");
    Serial.println(changePointZ);
    // теперь приподнимаем инструмент над датчиком на 5мм (4000 шагов)
    int zStepsInMm      = 800;                          // количество шагов в 1мм по оси Z
    int lengthMoveAway  = 5;                            // после касания датчика подняться на lengthMoveAway миллиметров
    int finalDistance   = zStepsInMm * lengthMoveAway;  // шпиндель поднимется на finalDistance миллиметров
    gSpeed = 0;                                         // на ускоренном ходу
    zDir = topFlag;                         // устанавливаем флаг направления движения "Вверх"
    digitalWrite(pinDirZ, top);             // устанавливаем значение пина, соответствующее данному направлению
    for (int i = 0; i < finalDistance; i++) {
        aMove.moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));
    }
    Serial.println("Tool touch sensor initialized!");
  }

  // метод поднимает шпиндель в самый верх (если шпиндель уже не находится в самом верху)
  void toRiseSpindle() {
    if (machinePosition.getPositionZ() < zDistance) {           // если текущее положение шпинделя ниже верхней точки
      zDir = topFlag;                                             // устанавливаем флаг направления движения "Вверх"
      digitalWrite(pinDirZ, top);                                 // устанавливаем значение пина, соответствующее данному направлению
      while (machinePosition.getPositionZ() < zDistance) {        // поднимаем шпиндель пока текущее положение ниже верхней точки рабочего диапазона оси Z
        aMove.moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));    // функция движения по Z на ускоренном ходу
      }
      Serial.println("Spindle rised.");
    } else {
      Serial.println("Spindle already at the highest point.");
    }
  }

  // метод отодвигает стол в самую далекую координату, чтобы освободить место для установки новой фрезы
  void moveAlongTable() {
    if (machinePosition.getPositionY() > 0) {         // если стол еще не в самой дальней точке (координата самой далекой точки: Y = 0)
      yDir = backFlag;
      digitalWrite(pinDirY1, back);
      digitalWrite(pinDirY2, back);
      while(machinePosition.getPositionY() > 0) {     // пока текущая позиция по оси Y больше нуля, двигаем стол
        aMove.moveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
      }
      Serial.println("table moved back");
    } else {                                          // стол уже в самой дальней точке - двигать уже ничего не надо и мы об этом сообщаем
      Serial.println("The table does not need to be moved");
    }
  }

  // метод перемещает шпиндель до точки смены инструмента по оси X
  void goToChangePointX() {
    // выясняем с какой стороны от точки смены инструмента сейчас находится шпиндель и задаем направление движения лево/право
    if (machinePosition.getPositionX() < changePointX) {              // если шпиндель в данный момент находится слева от точки смены инструмента
      xDir = rightFlag;                                                 // устанавливаем флаг направления "вправо"
      digitalWrite(pinDirX1, right);                                    // устанавливае значения пинов направления "вправо"
      digitalWrite(pinDirX2, right);
      while(machinePosition.getPositionX() < changePointX) {            // двигаемся вправо пока шпиндель не придет в точку смены инструмента
        aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
      }
      Serial.println("Spindle moved to right.");
    } else if (machinePosition.getPositionX() > changePointX) {         // если шпиндель в данный момент находится справа от точи смены инструмента
      xDir = leftFlag;                                                  // устанавливаем флаг направления "влево"
      digitalWrite(pinDirX1, left);                                     // устанавливае значения пинов направления "влево"
      digitalWrite(pinDirX2, left);
      while(machinePosition.getPositionX() > changePointX) {            // двигаемся влево пока шпиндель не придет в точку смены инструмента
        aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
      }
    } else {                                                            // а если шпиндель уже в точе смены по оси X, просто сообщаем об этом и ничего не делаем
      Serial.println("Spindle already at the change point on X axis.");
    }
  }
};

ToolChangePoint changeP;                // имплементим точку смены инструмента


class ReferentPoint {
  private:
  int32_t rPointG54X = 0;            // референтные точки G54 на каждой из осей. В шагах от начала координат станка
  int32_t rPointG54Y = 0;
  int32_t rPointG54Z = 0;

  public:

  void setReferentialPointG54() {    // устанавливаем референтную точку
    // Предполагается, что референтная точка устанавливается через параллельку толщиной 5мм.
    // Следовательно, реальная референтная точка будет на 5мм (4000 шагов
    // при микрошаге 1/8) ниже, чем текущее положение шпинделя (только для оси Z!!!)
    rPointG54X = machinePosition.getPositionX();
    rPointG54Y = machinePosition.getPositionY();
    rPointG54Z = machinePosition.getPositionZ() - spacerHeight;
  }

  int32_t getRPointG54X() {          // методы возвращают значения референтных точек
    return rPointG54X;
  }
  int32_t getRPointG54Y() {
    return rPointG54Y;
  }
  int32_t getRPointG54Z() {
    return rPointG54Z;
  }

  void goToRPoint() {               // метод ведет шпиндель к референтной точке
    if (rPointG54Z + spacerHeight - changeP.toolLenComp() > zDistance) {     // если инструмент слишком длинный
      // rPointG54Z: координата Z реф. точки;
      // spacerHeight: недоход до реф. точки;
      // changeP.toolLenComp(): поправка на длину инструмента
      Serial.println("The spindle cannot be moved to the reference point: the tool is too long.");
      return;
    } else if (rPointG54Z + spacerHeight - changeP.toolLenComp() < 0) {      // если инструмент слишком короткий
      Serial.println("The spindle cannot be moved to the reference point: the tool is too short.");
      return;
    }

    gSpeed = 0;                                               // декларируем, что перемещения сейчас будут на ускоренном ходу

    toRiseSpindle();    // сначала надо поднять шпиндель в самый верх (если референтная точка находится ниже максимальной высоты оси Z)
    goToRPointOnX();    // теперь, когда шпиндель вверху, перемещаемся по оси X до референтной точки (если в этом есть необходимость)
    goToRPointOnY();    // перемещаемся по оси Y до референтной точки (если в этом есть необходимость)
    lowerToRPoint();    // опускаем шпиндель по оси Z до референтной точки
                        // (если референтная точка ниже максимальной высоты оси Z)
                        // недоход до реф. точки составит величину spacerHeight (высота параллельки для выставления реф. точки)
                        // изначально эта величина планируется 5мм (4000 шагов)
  }

  void toRiseSpindle() {    // метод поднимает шпиндель в самый верх по оси Z
    if (machinePosition.getPositionZ() < zDistance) {           // если текущее положение шпинделя ниже верхней точки
      zDir = topFlag;                                                // устанавливаем флаг направления движения "Вверх"
      digitalWrite(pinDirZ, top);                                 // устанавливаем значение пина, соответствующее данному направлению
      while (machinePosition.getPositionZ() < zDistance) {        // поднимаем шпиндель пока текущее положение ниже верхней точки рабочего диапазона оси Z
        aMove.moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));    // функция движения по Z на ускоренном ходу
      }
      Serial.println("Spindle rised.");
    } else {
      Serial.println("Spindle already at the highest point.");
    }
  }

  // метод подводит шпиндель к референтной точке по оси X
  void goToRPointOnX() {
    int32_t deltaX = rPointG54X - machinePosition.getPositionX();    // расстояние, разница между текущим положением и референтной точкой. Может иметь отрицательное значение!
    if (deltaX != 0) {                                             // определяем, не находимся ли мы в данный момент в референтной точке по оси X
      // теперь определяем направление движения
      if (deltaX > 0) {
        xDir = rightFlag;                                               // устанавливаем флаг направления движения "вправо"
        digitalWrite(pinDirX1, right);                             // значения пинов устанавливаем соответственно данному направлению
        digitalWrite(pinDirX2, right);
        while (machinePosition.getPositionX() < rPointG54X) {    // вращаем двигатели до тех пор, пока координаты референтной точки и текущего положения по данной оси не станут равны.
          aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
        }
      } else if (deltaX < 0) {
        xDir = leftFlag;                                              // устанавливаем флаг направления движения "влево"
        digitalWrite(pinDirX1, left);                              // значения пинов устанавливаем соответственно данному направлению
        digitalWrite(pinDirX2, left);
        while (machinePosition.getPositionX() > rPointG54X) {    // вращаем двигатели до тех пор, пока координаты референтной точки и текущего положения по данной оси не станут равны.
          aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
        }
      }
      Serial.println("Spindle moved to RPoint on X axis");
    } else {
      Serial.println("Spindle dont need to move to RPoint on X axis");
    }
  }

  // метод подводит шпиндель к референтной точке по оси Y
  void goToRPointOnY() {
    Serial.println("Go to g54");
    int32_t deltaY = rPointG54Y - machinePosition.getPositionY();    // расстояние, разница между текущим положением и референтной точкой. Может иметь отрицательное значение!
    if (deltaY != 0) {                                             // определяем, не находимся ли мы в данный момент в референтной точке по оси Y
      // теперь определяем направление движения
      if (deltaY > 0) {
        yDir = forwardFlag;                                               // устанавливаем флаг направления движения "вперед"
        digitalWrite(pinDirY1, forward);                           // значения пинов устанавливаем соответственно данному направлению
        digitalWrite(pinDirY2, forward);
        while (machinePosition.getPositionY() < rPointG54Y) {    // вращаем двигатели до тех пор, пока координаты референтной точки и текущего положения по данной оси не станут равны.
          aMove.moveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
        }
      } else if (deltaY < 0) {
        yDir = backFlag;                                              // устанавливаем флаг направления движения "назад"
        digitalWrite(pinDirY1, back);                              // значения пинов устанавливаем соответственно данному направлению
        digitalWrite(pinDirY2, back);
        while (machinePosition.getPositionY() > rPointG54Y) {    // вращаем двигатели до тех пор, пока координаты референтной точки и текущего положения по данной оси не станут равны.
          aMove.moveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
        }
      }
      Serial.println("Spindle moved to RPoint on Y axis");
    } else {
      Serial.println("Spindle dont need to move to RPoint on Y axis");
    }
  }

  // метод опускает шпиндель от самой верхней точки оси Z до референтной точки
  void lowerToRPoint() {
    if (machinePosition.getPositionZ()
        - spacerHeight
        + changeP.toolLenComp() > rPointG54Z) {  // определяем, не находимся ли мы в данный момент в референтной точке по оси Z
      zDir = bottomFlag;                                                        // устанавливаем флаг направления движения "вперед"
      digitalWrite(pinDirZ, bottom);                                            // устанавливаем значение пина, соответствующее данному направлению
      while (machinePosition.getPositionZ()
             - spacerHeight
             + changeP.toolLenComp() > rPointG54Z) {    // вращаем двигатели до тех пор, пока координаты
                                                        // референтной точки и текущего положения по данной оси не станут равны.
        aMove.moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));              // функция движения по Z на ускоренном ходу
      }
      Serial.println("Spindle lowered to RPoint.");
    } else {
      Serial.println("Spindle dont need to lower to RPoint");
    }
  }
};


ReferentPoint refPoint;               // имплементим класс "Референтная точка"


void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.setTimeout(3);

  pinMode(SS, OUTPUT);                // надо для работы с sd модулем

  pinMode(pinStart, INPUT);
  pinMode(pinTuneMachine, INPUT);
  pinMode(pinSetG54, INPUT);
  pinMode(pinGoToG54, INPUT);

  pinMode(pinStepX1, OUTPUT);
  pinMode(pinStepX2, OUTPUT);
  pinMode(pinStepY1, OUTPUT);
  pinMode(pinStepY2, OUTPUT);
  pinMode(pinStepZ, OUTPUT);
  pinMode(pinDirX1, OUTPUT);
  pinMode(pinDirX2, OUTPUT);
  pinMode(pinDirY1, OUTPUT);
  pinMode(pinDirY2, OUTPUT);
  pinMode(pinDirZ, OUTPUT);
  pinMode(pinEn, OUTPUT);

  pinMode(pinStepByStep, INPUT);        // пины тумблеров скоростей ручного управления
  pinMode(pinFiSpeed, INPUT);
  pinMode(pinSeSpeed, INPUT);
  pinMode(pinThSpeed, INPUT);
  pinMode(pinToLeft, INPUT);            // кнопки движения на пульте управления
  pinMode(pinToRight, INPUT);
  pinMode(pinToForward, INPUT);
  pinMode(pinToBack, INPUT);
  pinMode(pinToTop, INPUT);
  pinMode(pinToBottom, INPUT);

  pinMode(pinEnTuning, INPUT);          // пин в новой редакции включает блокировку выхода осей за пределы рабочей зоны
  // поиск центра заготовки:
  pinMode(pinOutRect, INPUT);           // пин, включает автоматический поиск центра заготовки - прямоугольник, снаружи
  pinMode(pinTouchProbe, INPUT);        // пин, снимает показания с датчика 3D Touch Probe

  pinMode(pinSetToolSensor, INPUT);     // тумблер, запускает процесс инициализации датчика касания инструмента
  pinMode(ToolTouchDetected, INPUT);    // пин слушает касание инструментом датчика

  digitalWrite(pinEn, HIGH);             // держим шаговые моторы в вЫключенном состоянии

  digitalWrite(pinStepX1, LOW);
  digitalWrite(pinStepX2, LOW);
  digitalWrite(pinStepY1, LOW);
  digitalWrite(pinStepY2, LOW);
  digitalWrite(pinStepZ, LOW);

  digitalWrite(pinDirX1, LOW);
  digitalWrite(pinDirX2, LOW);
  digitalWrite(pinDirY1, LOW);
  digitalWrite(pinDirY2, LOW);
  digitalWrite(pinDirZ, LOW);
}

bool isNumber(char is_number) {   // функция парсинга: определяет, текущий символ цифра, или не цифра
  bool result = false;
  for (uint8_t i = 0; i < 10; i++) {    // сравниваем присланное число со всеми цифрами от 0 до 9
    char str_i[2];                      //
    itoa(i,str_i,DEC);
    if (is_number == str_i[0]) {
      result = true;
    }
  }
  return result;
}

int32_t absNum(int32_t num) {        // возвращает значение числа шагов в кадре по модулю (без учета знака +\-)
  if (num >= 0) {
    return num;
  } else {
    return -num;
  }
}

void read_line_sd() {
  if (!ncFile) {
    // инициализируем карту: инициализируем библиотеку, присваиваем контакт для сигнала
    while (!SD.begin(SPI_HALF_SPEED, chipSelect)) {
      // если карта неисправна, есть ошибка в подключении модуля - выводим сообщение
      Serial.println("sd_card initialization failed");
    }
    ncFile = SD.open(fileName);     // открываем файл с sd карты для чтения
    if (ncFile) {                   // проверяем, как открылся файл
      Serial.println("file is open successfully.");
    } else {
      Serial.println("file open failed.");
    }
  }
  uint8_t i = 0;                  // счетчик индекса символа во временной строке
  while (true) {                  // поштучно читаем символы из файла
    char curChar = ncFile.read();   // читаем очередной символ
    if (curChar != '\n') {          // если этот символ не перенос строки
      data[i] = curChar;   // записываем это символ под индексом [i] во временную строку
      data[i+1] = '\0';    // в следующую ячейку строки сразу же записываем
                           // символ конца строки. Если надо, этот символ потом перепишется цифрой
    } else {               // дошли до конца строки
      break;               // заканчиваем читать и записывать эту строку
    }
    i++;                   // переходим к след. символу этой строки
  }

}

void lineParsing() {                // функция парсит строку
  uint8_t lenData = strlen(data);     // длина строки, какую сейчас предстоит распарсить

  xStepsFrame = 0;                    // обнуляем значения количества шагов, чтобы не складывалось со значениями из прошого кадра
  yStepsFrame = 0;                    // обнулять надо, этого требует алгоритм заполнения этой группы переменных
  zStepsFrame = 0;
  xStepsFrameABS = 0;
  yStepsFrameABS = 0;
  zStepsFrameABS = 0;
  uint8_t i = 0;
  while (i < lenData) {               // перебираем строку посимвольно

    if (data[i] == 'X') {               // в строке нашли координату X
      bool minusX = false;                // переменная для хранения сведений о знаке текущей координаты
      while (true) {
        i++;                              // переходим к символу следующему за буквой
        if (isNumber(data[i])) {          // ищем все цифры координаты X
          char str_data_i[2];               // переменная для хранения текущего проверяемого символа
          str_data_i[0] = data[i];
          str_data_i[1] = '\0';             // создаем полноценную строку добавлением символа конца строки (без этого atol() не работает)
          xStepsFrame = xStepsFrame * 10 + atol(str_data_i);    // алгоритм добавления по одной цифре справа
        } else if (data[i] == '-') {        // в текущей координате обнаружен знак минус
          minusX = true;
          continue;
        } else {
          xStepsFrameABS = xStepsFrame;     // пока еще не добавили знак минуса количеству шагов по координате,
                                            // даже если она и отрицательная, записываем значение ABS
          if (minusX) {
            xStepsFrame = -xStepsFrame;
          }
          break;                            // Выходим из цикла поиска всех цифр за буквой
        }
      }
    }

    if (data[i] == 'Y') {        // в строке нашли координату Y
      bool minusY = false;              // переменная для хранения сведений о знаке текущей координаты
      while (true) {
        i++;                              // переходим к символу следующему за буквой
        if (isNumber(data[i])) {          // ищем все цифры координаты Y
          char str_data_i[2];               // переменная для хранения текущего проверяемого символа
          str_data_i[0] = data[i];
          str_data_i[1] = '\0';             // создаем полноценную строку добавлением символа конца строки (без этого atol() не работает)
          yStepsFrame = yStepsFrame * 10 + atol(str_data_i);    // алгоритм добавления по одной цифре справа
        } else if (data[i] == '-') {        // в текущей координате обнаружен знак минус
          minusY = true;
          continue;
        } else {
          yStepsFrameABS = yStepsFrame;     // пока еще не добавили знак минуса количеству шагов по координате,
                                            // даже если она и отрицательная, записываем значение ABS
          if (minusY) {
            yStepsFrame = -yStepsFrame;
          }
          break;                            // Выходим из цикла поиска всех цифр за буквой
        }
      }
    }

    if (data[i] == 'Z') {        // в строке нашли координату Z
      bool minusZ = false;              // переменная для хранения сведений о знаке текущей координаты
      while (true) {
        i++;                              // переходим к символу следующему за буквой
        if (isNumber(data[i])) {          // ищем все цифры координаты Z
          char str_data_i[2];               // переменная для хранения текущего проверяемого символа
          str_data_i[0] = data[i];
          str_data_i[1] = '\0';             // создаем полноценную строку добавлением символа конца строки (без этого atol() не работает)
          zStepsFrame = zStepsFrame * 10 + atol(str_data_i);    // алгоритм добавления по одной цифре справа
        } else if (data[i] == '-') {      // в текущей координате обнаружен знак минус
          minusZ = true;
          continue;
        } else {
          // zStepsFrame = zStepsFrame * 2;    // умножаем полученное с sd карты значение на 2, потому что на оси z стоит однозаходный винт
                                               // умножение отменено после редактирования компьютерного постпроцессора
                                               // теперь количество шагов на контроллер изначально приходит верное
          zStepsFrameABS = zStepsFrame;     // пока еще не добавили знак минуса количеству шагов по координате,
                                            // даже если она и отрицательная, записываем значение ABS
          if (minusZ) {
            zStepsFrame = -zStepsFrame;
          }
          break;                            // Выходим из цикла поиска всех цифр за буквой
        }
      }
    }

    if (data[i] == 'G') {        // в строке нашли команду G
      uint8_t gTemp = 0;                    // временная переменная нужна для алгоритма добавления разрядов по одной цифре справа (чтобы первоначальное значение всегда равнялось нулю)
      while (true) {                        // выясняем номер команды G (считываем число из строки data по одной цифре)
        i++;                                // переходим к символу следующему за буквой
        if (isNumber(data[i])) {            // ищем все цифры в номере команды G
          char str_data_i[2];               // переменная для хранения текущего проверяемого символа
          str_data_i[0] = data[i];
          str_data_i[1] = '\0';             // создаем полноценную строку добавлением символа конца строки (без этого atol() не работает)
          gTemp = gTemp * 10 + atol(str_data_i);
        } else {                            // Всё число записано во временную переменную полностью и пришло время узнаь, что же это за команда G
          if (gTemp == 0 || gTemp == 1) {   // Если это команда G: ускоренного перемещения || скорости на подаче
            gSpeed = gTemp;                   // Всё число записано во временную переменную полностью и пришло время его перенести в постоянную глобальную переменную
          }
          break;                            // Выходим из цикла поиска всех цифр за буквой
        }
      }
    }

    if (data[i] == 'F') {        // в строке нашли команду F - скорость подачи в мм/мин
      uint16_t fSpeedTemp = 0;              // временная переменная нужна для алгоритма добавления разрядов по
                                            // одной цифре справа (чтобы первоначальное значение всегда равнялось нулю)
      while (true) {                        // выясняем скорость подачи (считываем число из строки data по одной цифре)
        i++;                                  // переходим к символу следующему за буквой
        if (isNumber(data[i])) {                 // отыскиваем все цифры, следующие за буквой F
          char str_data_i[2];                     // переменная для хранения текущего проверяемого символа
          str_data_i[0] = data[i];
          str_data_i[1] = '\0';             // создаем полноценную строку добавлением символа конца строки (без этого atol() не работает)
          fSpeedTemp = fSpeedTemp * 10 + atol(str_data_i);
        } else {
          fSpeed = fSpeedTemp;              // Всё число записано во временную переменную полностью и пришло время его перенести в постоянную глобальную переменную
          speedSetting.setSpeed();          // вычисляем параметры движения для новой скорости
          break;                            // Выходим из цикла поиска всех цифр за буквой
        }
      }
    }

    if (data[i] == 'T') {         // в строке указана смена инструмента
      uint16_t toolNumTemp = 0;             // временная переменная нужна для алгоритма добавления разрядов по 
                                            // одной цифре справа (чтобы первоначальное значение всегда равнялось нулю)
      while (true) {                        // выясняем номер инструмента, какой сейчас будет установлен
        i++;                                  // переходим к символу следующему за буквой
        if (isNumber(data[i])) {              // отыскиваем все цифры, следующие за буквой T
          char str_data_i[2];                   // переменная для хранения текущего проверяемого символа
          str_data_i[0] = data[i];
          str_data_i[1] = '\0';                 // создаем полноценную строку добавлением символа конца строки (без этого atol() не работает)
          toolNumTemp = toolNumTemp * 10 + atol(str_data_i);
        } else {
          // Номер инструмента полностью записан.
          // Переходим к процедуре физической замены инструмента.
          // Для этого сначала нужно поднять шпиндель в самый верх и
          // отодвинуть стол, чтобы была возможность достать прежнюю фрезу
          // и вставить новую.
          changeP.toRiseSpindle();          // поднимаем шпиндель
          changeP.moveAlongTable();         // отодвигаем стол
          // Теперь можно останавливать шпиндель и менять фрезу.
          // А пока меняется фреза, ожидаем нажатия кнопок:
          //   2. все кнопки ручного перемещения по всем осям
          //   3. кнопка начать движение к точке смены инструмента
          //   4. перейти к референтной точке
          while (true) {
            // 2. проверяем нажата ли одна из кнопок ручного перемещения
            mControl.isOnManual();
            // 4. передвигаем шпиндель к референтной точке
            if (digitalRead(pinGoToG54)) {
              refPoint.goToRPoint();
            }
          }
          break;                            // Выходим из цикла поиска всех цифр за буквой
        }
      }
    }
    i++;                                    //переходим к следующему символу
  }
}


void defineDirection() {        // функция определения направления движения всех координат
  if (xStepsFrame != 0) {         // есть движение по данной оси
    if (xStepsFrame > 0) {          // значение координаты выше нуля
      digitalWrite(pinDirX1, right);   // устанавливаем значение пинов, соответствующее данному направлению
      digitalWrite(pinDirX2, right);
      xDir = rightFlag;                    // устанавливаем флаг направления движения в данном кадре (нужно для системы слежения за позицией в абсолютной системе координат)
    } else {                          // значение координаты ниже нуля
      digitalWrite(pinDirX1, left);   // устанавливаем значение пинов, соответствующее данному направлению
      digitalWrite(pinDirX2, left);
      xDir = leftFlag;
    }
  }
  if (yStepsFrame != 0) {         // есть движение по данной оси
    if (yStepsFrame > 0) {          // значение координаты выше нуля
      digitalWrite(pinDirY1, forward);   // устанавливаем значение пинов, соответствующее данному направлению
      digitalWrite(pinDirY2, forward);
      yDir = forwardFlag;                    // устанавливаем флаг направления движения в данном кадре (нужно для системы слежения за позицией в абсолютной системе координат)
    } else {                          // значение координаты ниже нуля
      digitalWrite(pinDirY1, back);   // устанавливаем значение пинов, соответствующее данному направлению
      digitalWrite(pinDirY2, back);
      yDir = backFlag;
    }
  }
  if (zStepsFrame != 0) {         // есть движение по данной оси
    if (zStepsFrame > 0) {          // значение координаты выше нуля
      digitalWrite(pinDirZ, top);   // устанавливаем значение пинов, соответствующее данному направлению
      zDir = topFlag;                  // устанавливаем флаг направления движения в данном кадре (нужно для системы слежения за позицией в абсолютной системе координат)
    } else {                          // значение координаты ниже нуля
      digitalWrite(pinDirZ, bottom);   // устанавливаем значение пинов, соответствующее данному направлению
      zDir = bottomFlag;
    }
  }
}

void intToChar(int32_t num) {
  tempNumChar[8] = '\0';              
  for (int i = 8; i >= 0; i--) {
    if (i == 7) {
      tempNumChar[i] = (num % 10) + 48;
      continue;
    }
    if (i == 6) {
      tempNumChar[i] = ((num % 100) / 10) + 48;
      continue;
    }
    if (i == 5) {
      tempNumChar[i] = ((num % 1000) / 100) + 48;
      continue;
    }
    if (i == 4) {
      tempNumChar[i] = ((num % 10000) / 1000) + 48;
      continue;
    }
    if (i == 3) {
      tempNumChar[i] = ((num % 100000) / 10000) + 48;
      continue;
    }
    if (i == 2) {
      tempNumChar[i] = ((num % 1000000) / 100000) + 48;
      continue;
    }
    if (i == 1) {
      tempNumChar[i] = ((num % 10000000) / 1000000) + 48;
      continue;
    }
    if (i == 0) {
      tempNumChar[i] = ((num % 100000000) / 10000000) + 48;
      continue;
    }
  }
}

void countFrames() {            // функция считает количество выполненных кадров
  frameCounter++;               // считаем номера кадров
  // Serial.println("frame: " + String(frameCounter));
  if (frameCounter % 1000 == 0) {
    // Serial.println("frame: " + String(frameCounter));
  }
}

void report() {
  /*
  // считаем запланированное количество шагов, сколько должно было бы быть выполнено от начала программы до текущего момента:
  xPlaneToTime = xPlaneToTime + xStepsFrameABS;
  yPlaneToTime = yPlaneToTime + yStepsFrameABS;
  zPlaneToTime = zPlaneToTime + zStepsFrameABS;

  // массивы символов для вывода в консоль типа 00000328
  char fXStepsFrameABS[9];  // запланированные шаги в пределах одного кадра
  char fYStepsFrameABS[9];
  char fZStepsFrameABS[9];
  char fXStepsDone[9];    // выполненные шаги в пределах одного кадра
  char fYStepsDone[9];
  char fZStepsDone[9];
  char fXPlaneToTime[9];   // запланированные шаги от начала программы до данного момента
  char fYPlaneToTime[9];
  char fZPlaneToTime[9];
  char fXDoneToTime[9];  // выполненные шаги от начала программы до данного момента
  char fYDoneToTime[9];
  char fZDoneToTime[9];

  intToChar(xStepsFrameABS);            // преобразуем запланированные шаги в пределах одного кадра из числа в строку типа 00000328 и записываем в массивы выше
  for (int i = 0; i < 9; i++) {fXStepsFrameABS[i] = tempNumChar[i];}  // копируем временный массив tempNumChar[] в нужных массив
  intToChar(yStepsFrameABS);
  for (int i = 0; i < 9; i++) {fYStepsFrameABS[i] = tempNumChar[i];}
  intToChar(zStepsFrameABS);
  for (int i = 0; i < 9; i++) {fZStepsFrameABS[i] = tempNumChar[i];}

  intToChar(xStepsDone);              // преобразуем выполненные шаги в пределах одного кадра из числа в строку типа 00000328 и записываем в нужный массив
  for (int i = 0; i < 9; i++) {fXStepsDone[i] = tempNumChar[i];}
  intToChar(yStepsDone);
  for (int i = 0; i < 9; i++) {fYStepsDone[i] = tempNumChar[i];}
  intToChar(zStepsDone);
  for (int i = 0; i < 9; i++) {fZStepsDone[i] = tempNumChar[i];}

  intToChar(xPlaneToTime);  // преобразуем запланированные шаги от начала программы до настоящего момента из числа в строку типа 00000328 и записываем в нужный массив
  for (int i = 0; i < 9; i++) {fXPlaneToTime[i] = tempNumChar[i];}
  intToChar(yPlaneToTime);
  for (int i = 0; i < 9; i++) {fYPlaneToTime[i] = tempNumChar[i];}
  intToChar(zPlaneToTime);
  for (int i = 0; i < 9; i++) {fZPlaneToTime[i] = tempNumChar[i];}

  intToChar(xDoneToTime);  // преобразуем выполненные шаги от начала программы до настоящего момента из числа в строку типа 00000328 и записываем в нужный массив
  for (int i = 0; i < 9; i++) {fXDoneToTime[i] = tempNumChar[i];}
  intToChar(yDoneToTime);
  for (int i = 0; i < 9; i++) {fYDoneToTime[i] = tempNumChar[i];}
  intToChar(zDoneToTime);
  for (int i = 0; i < 9; i++) {fZDoneToTime[i] = tempNumChar[i];}
  
  Serial.println("Farame steps planned: X" + String(fXStepsFrameABS) + " Y" + String(fYStepsFrameABS) + " Z" + String(fZStepsFrameABS));
  Serial.println("Farame steps done   : X" + String(fXStepsDone) + " Y" + String(fYStepsDone) + " Z" + String(fZStepsDone));
  Serial.println("To time steps planned: X" + String(fXPlaneToTime) + " Y" + String(fYPlaneToTime) + " Z" + String(fZPlaneToTime));
  Serial.println("To time steps done   : X" + String(fXDoneToTime) + " Y" + String(fYDoneToTime) + " Z" + String(fZDoneToTime));
  */
}

void startProgram() {
  if (digitalRead(pinStart)) {  // если был нажат тумблер "Старт программы"
    synFrameProc sfp;             // имплементим класс распределения шагов// uint32_t oldZPosition = machinePosition.getPositionZ();
    while (true) {
      read_line_sd();             // читаем строку из файла на флешке
      lineParsing();              // парсим строку
      defineDirection();          // устанавливаем направление движения по всем осям
      sfp.sfpFrameProcessing();   // отрабатываем шаги
      report();                   // мониторим результаты работы программы на экране компа
      xStepsDone = 0;             // обнуляем количество выполненных шагов в текущем кадре
      yStepsDone = 0;
      zStepsDone = 0;
    }
  }
}

class CenterFinder {
public:
  // функция начинает автоматический поиск центра заготовки прямоугольной формы
  void startCenterOutRect() {
    startPositionX = machinePosition.getPositionX();            // в начале поиски центра инициализируем начальные координаты по X и Y
    startPositionY = machinePosition.getPositionY();
    searchTop();                  // находим верх заготовки
    searchLeftWall();             // ищем левый край заготовки
    goToStartPositionX();         // по оси X возвращаемся к точке, с которой начали измерения
    searchRightWall();            // ищем правую грань
    goToStartPositionX();         // по оси X возвращаемся к точке, с которой начали измерения
    searchBackWall();             // ищем дальнюю стенку заготовки (дальнюю от оператора, когда он стоит спереди станка)
    goToStartPositionY();         // по оси Y возвращаемся к точке, с которой начали измерения
    searchFrontWall();            // ищем переднюю стенку заготовки (ближе к оператору)
    goToStartPositionY();         // по оси Y возвращаемся к точке, с которой начали измерения
    goToCenterX();                // идем в центр заготовки по оси X
    goToCenterY();                // идем в центр заготовки по оси Y
    keepSpacerHeight();           // привести шпиндель на высоту проставки (предположительно 5 мм)
  }

private:
  const uint16_t stepsInMmXY = 400;               // шагов в 1мм по осям X и Y
  const uint16_t stepsInMmZ = 800;                // при 1/8 шага в 1мм движения по оси Z 800 импульсов
  uint16_t bumpCounter = 0;                        // счетчик столкновений с датчиками и концевиками (сколько допустимо шагов в состоянии 
                                                  // столкновения пока станок будет заблокирован)
  uint16_t allowedBumps = 200;                // допустимое количество шагов двигателя в состоянии столкновения с датчиками и концевиками
  uint32_t lowestPoint;                           // плоскость, на которой будет срабатывать логика, что край заготовки пройден
  uint32_t safetyPlane;                           // плоскость, на которую будет подниматься шпиндель между проходами (абс. сист. коорд.)
  uint32_t startPositionX;                        // точка на оси X с которой начались измерения
  uint32_t startPositionY;                        // точка на оси Y с которой начались измерения
  uint32_t topWorkpiece;                          // координата верха заготовки (по оси Z)
  uint32_t leftWallPosition;                      // координата левой стенки заготовки (по оси X)
  uint32_t rightWallPosition;                     // координата правой стенки заготовки (по оси X)
  uint32_t backWallPosition;                      // координата задней стенки заготовки (ось Y)
  uint32_t forwardWallPosition;                   // координата передней стенки заготовки (ось Y)
  uint32_t centerPositionX;                       // координата центра заготовки по оси X
  uint32_t centerPositionY;                       // координата центра заготовки по оси Y
  const uint32_t measurementDistance = stepsInMmXY * 8;  // расстояние между измерениями в ШАГАХ
  const uint16_t liftingHeight = stepsInMmZ * 4;        // высота подъема в ШАГАХ между проходами
  const uint32_t retraction = stepsInMmXY * 2;          // расстояние отвода датчика от стенки после касания

  // инициализация станка для автоматического движения шпинделя вниз
  void initToDown() {
    gSpeed = 1;               // устанавливаем режим движения: на подаче
    fSpeed = 200;             // устанавливаем скорость движения 200 мм/мин
    speedSetting.setSpeed();  // вычисляем параметры движения для новой скорости
    zDir = bottomFlag;        // устанавливаем флаг направления движения "вниз"
    digitalWrite(pinDirZ, bottom);    // устанавливаем значение пина, соответствующее данному направлению
  }

  // функция впервые ищет (инициализирует) верх заготовки
  void searchTop() {      // touch the top of the workpiece
    initToDown();
    while (digitalRead(pinTouchProbe)) {    // вращаем двигатели 
      aMove.moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));              // функция автоматического движения по Z
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                    // обнуляем счетчик столкновений с датчиками
    topWorkpiece = machinePosition.getPositionZ();      // сохраняем координату верха заготовки
    Serial.print("topWorkpiece at start: ");
    Serial.println(topWorkpiece);
    safetyPlane = liftingHeight + topWorkpiece;         // создаем координаты плоскости безопасности для перемещений над верхом заготовки
    Serial.print("safetyPlane: ");
    Serial.println(safetyPlane);
    lowestPoint = topWorkpiece - (stepsInMmZ * 5);      // плоскость, на которой будет срабатывать логика, что край заготовки пройден
  }

  // функция опускает шпиндель вниз до касания, а когда касания не просиходит на расстоянии вплоть до -5мм от поверхности заготовки
  // присылает false, что означает - край заготовки уже пройден
  bool downToTouch() {
    initToDown();
    // опускаем шпиндель пока или датчик не коснется верха заготовки, или шпиндель опустится до минимальной точки (точки срабатывания)
    while (digitalRead(pinTouchProbe) && machinePosition.getPositionZ() >= lowestPoint) {
      aMove.moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));              // функция автоматического движения по Z
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
    if (machinePosition.getPositionZ() > lowestPoint) {
      // шпиндель не достиг нижней точки срабатывания
      return false;
    } else {
      // шпиндель дошел до края заготовки и опустился до точки срабатывания (ниже уровня верха заготовки)
      return true;
    }
  }

  // функция поднимает шпиндель на плоскость безопасности
  void upToSafetyPlane() {
    gSpeed = 0;                                     // устанавливаем режим движения: ускоренное
    zDir = topFlag;                                 // устанавливаем флаг направления движения "вверх"
    digitalWrite(pinDirZ, top);                     // устанавливаем значение пина, соответствующее данному направлению
    // поднимаем шпиндель, пока текущая позиция ниже необходимой
    while (machinePosition.getPositionZ() < safetyPlane) {
      aMove.moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));              // функция автоматического движения по Z
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
  }

  // функция двигает шпиндель влево на расстояние, заданное в шагах
  void moveLeft(uint32_t moveDistance) {
    upToSafetyPlane();                   // поднимаем шпиндель на плоскость безопасности
    gSpeed = 0;                                     // устанавливаем режим движения: ускоренное
    xDir = leftFlag;                                // устанавливаем флаг направления движения "влево"
    digitalWrite(pinDirX1, left);                   // устанавливаем направление движения влево
    digitalWrite(pinDirX2, left);
    for (uint32_t i = 0; i < moveDistance; i++) {
      aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
  }

  // функция двигает шпиндель вправо на расстояние, заданное в шагах
  void moveRight(uint32_t moveDistance) {
    upToSafetyPlane();                   // поднимаем шпиндель на плоскость безопасности
    gSpeed = 0;                                     // устанавливаем режим движения: ускоренное
    xDir = rightFlag;                               // устанавливаем флаг направления движения "вправо"
    digitalWrite(pinDirX1, right);                   // устанавливаем направление движения вправо
    digitalWrite(pinDirX2, right);
    for (uint32_t i = 0; i < moveDistance; i++) {
      aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
  }

  // метод двигает шпиндель вдаль от оператора на расстояние, заданное в шагах
  void moveBack(uint32_t moveDistance) {
    upToSafetyPlane();                   // поднимаем шпиндель на плоскость безопасности
    gSpeed = 0;                          // устанавливаем режим движения: ускоренное
    yDir = backFlag;                     // устанавливаем флаг направления движения "назад"
    digitalWrite(pinDirY1, back);        // устанавливаем направление движения назад (вдаль от оператора)
    digitalWrite(pinDirY2, back);
    for (uint32_t i = 0; i < moveDistance; i++) {
      aMove.moveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
  }

  // метод двигает шпиндель в сторону оператора на расстояние, заданное в шагах
  void moveFront(uint32_t moveDistance) {
    upToSafetyPlane();                   // поднимаем шпиндель на плоскость безопасности
    gSpeed = 0;                          // устанавливаем режим движения: ускоренное
    yDir = forwardFlag;                     // устанавливаем флаг направления движения "вперед"
    digitalWrite(pinDirY1, forward);        // устанавливаем направление движения ваперед (к оператору)
    digitalWrite(pinDirY2, forward);
    for (uint32_t i = 0; i < moveDistance; i++) {
      aMove.moveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
  }

  // функция касается левой стенки и отводит шпиндель на 2мм влево
  void touchLeftWall() {
    gSpeed = 1;                   // устанавливаем режим движения: на подаче
    fSpeed = 200;                 // устанавливаем скорость движения 150 мм/мин
    speedSetting.setSpeed();      // вычисляем параметры движения для новой скорости
    xDir = rightFlag;             // устанавливаем флаг направления движения "вправо"
    digitalWrite(pinDirX1, right);                   // устанавливаем направление движения вправо
    digitalWrite(pinDirX2, right);
    // двигаемся вправо до касания левой грани
    while (digitalRead(pinTouchProbe)) {
      aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    leftWallPosition = machinePosition.getPositionX();    // коснулись левой грани заготовки - записываем позицию по X
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
    // теперь отъезжаем на 2 мм от грани, чтобы движение вверх не царапало шарик датчика
    gSpeed = 0;                   // устанавливаем режим движения: ускоренный
    xDir = leftFlag;              // устанавливем флаг направления движения "влево"
    digitalWrite(pinDirX1, left);                   // устанавливаем направление движения влево
    digitalWrite(pinDirX2, left);
    // отъезжаем отграни на расстояние retraction (отвод)
    for (uint32_t i = 0; i < retraction; i++) {
      aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
  }

  // функция касается правой стенки и отводит шпиндель на 2мм вправо
  void touchRightWall() {
    gSpeed = 1;                   // устанавливаем режим движения: на подаче
    fSpeed = 200;                 // устанавливаем скорость движения 150 мм/мин
    speedSetting.setSpeed();      // вычисляем параметры движения для новой скорости
    xDir = leftFlag;              // устанавливаем флаг направления движения "влево"
    digitalWrite(pinDirX1, left);                   // устанавливаем направление движения влево
    digitalWrite(pinDirX2, left);
    // двигаемся влево до касания правой грани
    while (digitalRead(pinTouchProbe)) {
      aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
    rightWallPosition = machinePosition.getPositionX();    // коснулись левой грани заготовки - записываем позицию по X
    bumpCounter = 0;                                       // обнуляем счетчик столкновений с датчиками
    // теперь отъезжаем на 2 мм от грани, чтобы движение вверх не царапало шарик датчика
    gSpeed = 0;                   // устанавливаем режим движения: ускоренный
    xDir = rightFlag;              // устанавливем флаг направления движения "вправо"
    digitalWrite(pinDirX1, right);                   // устанавливаем направление движения вправо
    digitalWrite(pinDirX2, right);
    // отъезжаем от грани на расстояние retraction (отвод)
    for (uint32_t i = 0; i < retraction; i++) {
      aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
  }

  // функция касается задней стенки и отводит шпиндель на 2мм назад
  void touchBackWall() {
    gSpeed = 1;                   // устанавливаем режим движения: на подаче
    fSpeed = 200;                 // устанавливаем скорость движения 150 мм/мин
    speedSetting.setSpeed();      // вычисляем параметры движения для новой скорости
    yDir = forwardFlag;           // устанавливаем флаг направления движения "вперед"
    digitalWrite(pinDirY1, forward);      // устанавливаем направление движения вперед
    digitalWrite(pinDirY2, forward);
    // двигаемся назад до касания задней грани
    while (digitalRead(pinTouchProbe)) {
      aMove.moveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    backWallPosition = machinePosition.getPositionY();    // коснулись задней грани заготовки - записываем позицию по Y
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
    // теперь отъезжаем на 2 мм от грани, чтобы движение вверх не царапало шарик датчика
    gSpeed = 0;                           // устанавливаем режим движения: ускоренный
    yDir = backFlag;                      // устанавливем флаг направления движения "назад"
    digitalWrite(pinDirY1, back);         // устанавливаем направление движения назад
    digitalWrite(pinDirY2, back);
    // отъезжаем от грани на расстояние retraction (отвод)
    for (uint32_t i = 0; i < retraction; i++) {
      aMove.moveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
  }

  // функция касается передней стенки и отводит шпиндель на 2мм назад
  void touchForwardWall() {
    gSpeed = 1;                   // устанавливаем режим движения: на подаче
    fSpeed = 200;                 // устанавливаем скорость движения 150 мм/мин
    speedSetting.setSpeed();      // вычисляем параметры движения для новой скорости
    yDir = backFlag;              // устанавливаем флаг направления движения "назад"
    digitalWrite(pinDirY1, back);           // устанавливаем направление движения назад
    digitalWrite(pinDirY2, back);
    // двигаемся назад до касания задней грани
    while (digitalRead(pinTouchProbe)) {
      aMove.moveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    forwardWallPosition = machinePosition.getPositionY();    // коснулись задней грани заготовки - записываем позицию по Y
    bumpCounter = 0;                                         // обнуляем счетчик столкновений с датчиками
    // теперь отъезжаем на 2 мм от грани, чтобы движение вверх не царапало шарик датчика
    gSpeed = 0;                             // устанавливаем режим движения: ускоренный
    yDir = forwardFlag;                     // устанавливем флаг направления движения "назад"
    digitalWrite(pinDirY1, forward);        // устанавливаем направление движения назад
    digitalWrite(pinDirY2, forward);
    // отъезжаем от грани на расстояние retraction (отвод)
    for (uint32_t i = 0; i < retraction; i++) {
      aMove.moveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
  }

  // функция ищет левую грань заготовки
  void searchLeftWall() {
    upToSafetyPlane();                   // поднимаем шпиндель на плоскость безопасности
    while (true) {
      moveLeft(measurementDistance);                  // сдвигаем шпиндель влево на одно расстояние между измерениями
      if (downToTouch()) {
        touchLeftWall();                        // касаемся левой стенки
        upToSafetyPlane();                     // поднимаем шпиндель на плоскость безопасности
        break;
      }
    }
    
  }

  // функция ищет правую грань заготовки
  void searchRightWall() {
    upToSafetyPlane();                   // поднимаем шпиндель на плоскость безопасности
    while (true) {
      moveRight(measurementDistance);                // сдвигаем шпиндель вправо на одно расстояние между измерениями
      if (downToTouch()) {
        touchRightWall();                      // касаемся правой стенки
        upToSafetyPlane();                     // поднимаем шпиндель на плоскость безопасности
        break;
      }
    }
  }

  // метод ищет заднюю (дальнюю) стенку заготовки
  void searchBackWall() {
    upToSafetyPlane();                // поднимаемся на плоскость безопасности
    while (true) {
      moveBack(measurementDistance);  // проходим несколько миллиметров
      if (downToTouch()) {
        touchBackWall();
        upToSafetyPlane();
        break;
      }
    }
  }

  // метод ищет переднюю (ближнюю) стенку заготовки
  void searchFrontWall() {
    upToSafetyPlane();                // поднимаемся на плоскость безопасности
    while (true) {
      moveFront(measurementDistance); // проходим несколько миллиметров
      if (downToTouch()) {
        touchForwardWall();
        upToSafetyPlane();
        break;
      }
    }
  }

  // по оси X возвращаемся в точку, с которой начинались измерения
  void goToStartPositionX() {
    upToSafetyPlane();                   // поднимаем шпиндель на плоскость безопасности
    uint32_t curPositionX = machinePosition.getPositionX();          // ищем, где мы находимся в данный момент в абсолютной системе координат
    if (curPositionX < startPositionX) {    // если мы находимся левее стартовой точки
      xDir = rightFlag;                     // устанавливаем флаг направления движения "вправо"
      digitalWrite(pinDirX1, right);        // устанавливаем направление движения вправо
      digitalWrite(pinDirX2, right);
    } else {
      xDir = leftFlag;                      // устанавливаем флаг направления движения "влево"
      digitalWrite(pinDirX1, left);         // устанавливаем направление движения влево
      digitalWrite(pinDirX2, left);
    }
    gSpeed = 0;                             // устанавливаем режим движения: ускоренное
    int32_t moveDistanceX = startPositionX - curPositionX;        // находим расстояние, какое надо пройти, чтобы вернуться в начальную точку
    moveDistanceX = abs(moveDistanceX);
    // двигаемся нужное количество шагов до начальной точки измерений
    for (uint32_t i = 0; i < moveDistanceX; i++) {
      aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
  }

  // по оси Y возвращаемся в точку, с которой начинались измерения
  void goToStartPositionY() {
    upToSafetyPlane();                   // поднимаем шпиндель на плоскость безопасности
    uint32_t curPositionY = machinePosition.getPositionY();          // ищем, где мы находимся в данный момент в абсолютной системе координат
    if (curPositionY < startPositionY) {    // если мы находимся дальше стартовой точки (дальше от оператора)
      yDir = forwardFlag;                     // устанавливаем флаг направления движения "вперед"
      digitalWrite(pinDirY1, forward);        // устанавливаем направление движения вперед
      digitalWrite(pinDirY2, forward);
    } else {
      yDir = backFlag;                      // устанавливаем флаг направления движения "назад" (от оператора)
      digitalWrite(pinDirY1, back);         // устанавливаем направление движения назад
      digitalWrite(pinDirY2, back);
    }
    gSpeed = 0;                             // устанавливаем режим движения: ускоренное
    int32_t moveDistanceY = startPositionY - curPositionY;        // находим расстояние, какое надо пройти, чтобы вернуться в начальную точку
    moveDistanceY = abs(moveDistanceY);
    // двигаемся нужное количество шагов до начальной точки измерений
    for (uint32_t i = 0; i < moveDistanceY; i++) {
      aMove.moveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
  }

  // двигаемся к центру заготовки по оси X
  void goToCenterX() {
    upToSafetyPlane();                   // поднимаем шпиндель на плоскость безопасности
    centerPositionX = leftWallPosition + ((rightWallPosition - leftWallPosition) / 2);
    uint32_t curPositionX = machinePosition.getPositionX();          // ищем, где мы находимся в данный момент в абсолютной системе координат
    if (curPositionX < centerPositionX) {                           // текущее положение левее центральной точки
      xDir = rightFlag;                                 // устанавливаем флаг направления движения "вправо"
      digitalWrite(pinDirX1, right);                    // устанавливаем направление движения вправо
      digitalWrite(pinDirX2, right);
    } else {
      xDir = leftFlag;                                  // устанавливаем флаг направления движения "влево"
      digitalWrite(pinDirX1, left);                     // устанавливаем направление движения влево
      digitalWrite(pinDirX2, left);
    }
    gSpeed = 0;                                     // устанавливаем режим движения: ускоренное
    int32_t moveDistanceX = centerPositionX - curPositionX;        // находим расстояние, какое надо пройти, чтобы попасть в центральную точку
    moveDistanceX = abs(moveDistanceX);
    // двигаемся нужное количество шагов до центральной точки по оси X
    for (uint32_t i = 0; i < moveDistanceX; i++) {
      aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
  }

  // двигаемся к центру заготовки по оси Y
  void goToCenterY() {
    upToSafetyPlane();                   // поднимаем шпиндель на плоскость безопасности
    centerPositionY = backWallPosition + ((forwardWallPosition - backWallPosition) / 2);
    uint32_t curPositionY = machinePosition.getPositionY();           // ищем, где мы находимся в данный момент в абсолютной системе координат
    if (curPositionY < centerPositionY) {                             // текущее положение левее центральной точки
      yDir = forwardFlag;                                 // устанавливаем флаг направления движения "вперед"
      digitalWrite(pinDirY1, forward);                    // устанавливаем направление движения вперед
      digitalWrite(pinDirY2, forward);
    } else {
      yDir = backFlag;                                    // устанавливаем флаг направления движения "влево"
      digitalWrite(pinDirY1, back);                       // устанавливаем направление движения влево
      digitalWrite(pinDirY2, back);
    }
    gSpeed = 0;                                     // устанавливаем режим движения: ускоренное
    int32_t moveDistanceY = centerPositionY - curPositionY;        // находим расстояние, какое надо пройти, чтобы попасть в центральную точку
    moveDistanceY = abs(moveDistanceY);
    Serial.print("moveDistanceY: ");
    Serial.println(moveDistanceY);
    // двигаемся нужное количество шагов до центральной точки по оси Y
    for (uint32_t i = 0; i < moveDistanceY; i++) {
      aMove.moveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
      collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
    }
    bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
  }

  keepSpacerHeight() {
    uint32_t curPositionZ = machinePosition.getPositionZ();     // находим текущее положение по Z
    uint32_t spacerHeightPoint = topWorkpiece + spacerHeight;   // координаты точки на высоте проставки от поверхности
    int32_t moveDistanceZ = spacerHeightPoint - curPositionZ;  // расстояние от текущего местоположения до spacerHeightPoint по оси Z
    Serial.print("curPositionZ: ");
    Serial.println(curPositionZ);
    Serial.print("topWorkpiece: ");
    Serial.println(topWorkpiece);
    Serial.print("spacerHeightPoint: ");
    Serial.println(spacerHeightPoint);
    Serial.print("moveDistanceZ: ");
    Serial.println(moveDistanceZ);
    if (moveDistanceZ > 0) {
      // текущее местоположение ниже spacerHeightPoint: едем вверх
      gSpeed = 0;                                     // устанавливаем режим движения: ускоренное
      zDir = topFlag;                                 // устанавливаем флаг направления движения "вверх"
      digitalWrite(pinDirZ, top);                     // устанавливаем значение пина, соответствующее данному направлению
      // поднимаем шпиндель, пока текущая позиция ниже необходимой
      while (machinePosition.getPositionZ() < spacerHeightPoint) {
        aMove.moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));              // функция автоматического движения по Z
        collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
      }
      bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
    } else if (moveDistanceZ < 0) {
      // текущее местоположение выше spacerHeightPoint: едем вверх
      gSpeed = 0;                                     // устанавливаем режим движения: ускоренное
      zDir = bottomFlag;                                 // устанавливаем флаг направления движения "вниз"
      digitalWrite(pinDirZ, bottom);                     // устанавливаем значение пина, соответствующее данному направлению
      // опускаем шпиндель, пока текущая позиция выше необходимой
      while (machinePosition.getPositionZ() > spacerHeightPoint) {
        aMove.moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));              // функция автоматического движения по Z
        collisionCheck();             // проверяем нет ли столкновений с концевиками и датчиком касания
      }
      bumpCounter = 0;                                      // обнуляем счетчик столкновений с датчиками
    }
  }

  void collisionCheck() {
    // проверяем столкновения с датчиком касания:
    if (!digitalRead(pinTouchProbe)) {
      bumpCounter++;
      if (bumpCounter > allowedBumps) {
        while (true) {
          Serial.println("Error! There was a collision with the touch sensor!");
        }
      }
    }
    // проверяем столкновения с концевиками:
    if (digitalRead(pinLimitSwitchX)) {
      bumpCounter++;
      if (bumpCounter > allowedBumps) {
        while (true) {
          Serial.println("Error! There was a collision with the pinLimitSwitchX sensor!");
        }
      }
    }
    if (digitalRead(pinLimitSwitchY)) {
      bumpCounter++;
      if (bumpCounter > allowedBumps) {
        while (true) {
          Serial.println("Error! There was a collision with the pinLimitSwitchY sensor!");
        }
      }
    }
    if (digitalRead(pinLimitSwitchZ)) {
      bumpCounter++;
      if (bumpCounter > allowedBumps) {
        while (true) {
          Serial.println("Error! There was a collision with the pinLimitSwitchZ sensor!");
        }
      }
    }
  }
};

CenterFinder centerFinder;

void loop() {
  delay(1500);                  // даем возможность запуститься блокам питания перед считыванием состояния пульта упавления
  digitalWrite(pinEn, LOW);     // включаем двигатели
  while (true) {                // основной цикл
    mControl.isOnManual();      // проверяем нажата ли одна из кнопок ручного перемещения
    if (digitalRead(pinTuneMachine)) {      // нажат тумблер выхода в ноль координат станка
      limitSwitchTuning.tuneMachine();      // выводим оси станка в нулевые позиции
    } else if (digitalRead(pinStart)) {     // нажат тумблер "Старт программы"
      startProgram();                       // начинается отработка программы с флешки
    } else if (digitalRead(pinSetG54)) {
      refPoint.setReferentialPointG54();    // устанавливаем референтную точку
    } else if (digitalRead(pinGoToG54)) {
      refPoint.goToRPoint();                // передвигаем шпиндель к референтной точке
    }else if (digitalRead(pinSetToolSensor)) {
      changeP.setToolSensorPoint();         // начинаем процесс инициализации датчика касания инструмента
    }else if (digitalRead(pinOutRect)) {
      centerFinder.startCenterOutRect();    // начинаем автоматический поиск центра заготовки
    }
  }
}
