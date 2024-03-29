//  токен доступа на gitlab: glpat-rn2fPZdNUwZh9yVov6bY
//     G01X-123456Y-123456Z-123456M06F24000;
//     0123456789012345678901234567890123456    i в массиве
//     0         1         2         3         десятки
//     1234567890123456789012345678901234567     штук ()
//     0        1         2         3            десятки

#include "SPI.h"        // библиотека для работы с периферийными устройствами
#include "SD.h"         // библиотека для работы с sd картами
File ncFile;            // создаем объект класса File для работы с файлами на флешке

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

// пины смены инструмента
#define pinGoChangePoint 45     // кнопка идем к точке смены инструмента по X
#define ToolTouchDetected 37    // пин слушает касание инструментом датчика
#define pinSetToolSensor 40     // тумблер задать координаты точки в которой будет происходить смена инструмента
#define pinAutoSetTool 42       // кнопка продолжить программу после замены инструмента

#define pinStepByStep 28        // тумблеры скоростей на пульте управления
#define pinFiSpeed 26
#define pinSeSpeed 24
#define pinThSpeed 22
const uint8_t pinToLeft    = 31;   // кнопки движения на пульте управления
const uint8_t pinToRight   = 33;
const uint8_t pinToForward = 27;
const uint8_t pinToBack    = 29;
const uint8_t pinToTop     = 25;
const uint8_t pinToBottom  = 23;

// концевики - контроль рабочей зоны станка
#define pinEnTuning 30          // пин, включение режима ограничения выхода за рабочую зону
#define pinLimitSwitchX 47      // пин, куда подключается концевик подстроечный
#define pinLimitSwitchY 48
#define pinLimitSwitchZ 38

// выход в ноль координат ЗАГОТОВКИ, поиск центра заготовки
const uint8_t pinTouchProbe = 36;           // пин, снимает показания с датчика 3D Touch Probe
const uint8_t startSearchG54Rectangle = 39; // тумблер, старт поиска нулевой точки заготовки в полуавтоматическом режиме

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

const uint32_t widthXAxis = 132000;        // ширина оси X в шагах
const uint32_t lengthYAxis = 98000;        // длина оси Y в шагах
const uint32_t zDistance = 65600;          // длина оси Z
const uint16_t spacerHeight = 4000;        // высота проставки в шагах для установки референтной точки. 4000 шагов - это 5мм.

int32_t rPointG54X = 0;                    // референтные точки G54 на каждой из осей. В шагах от начала координат станка
int32_t rPointG54Y = 0;
int32_t rPointG54Z = 0;

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

bool minusX = false;            // переменная для хранения сведений о знаке текущей координаты
bool minusY = false;
bool minusZ = false;

bool ovfX = false;                  // флаг переполнения таймера 3
bool ovfY = false;                  // флаг переполнения таймера 4
bool ovfZ = false;                  // флаг переполнения таймера 5

char tempNumChar[9];                     // массив для хранения чисел, преобразованных в строки функцией intToChar();

uint32_t frameCounter = 0;              // счетчик выполненных к данному моменту кадров. Выводится на компьютер для визуального контроля прогресса обработки

// коды ошибок:
const uint8_t OVER_LOW_LIMIT = 1;       // шпиндель заходит за нижнюю границу по оси Z
const uint8_t OVER_HIGH_LIMIT = 2;      // шпиндель заходит за верхнюю границу по Z
const uint8_t TOO_SHORT_TOOL = 7;       // инструмент в шпинделе слишком короткий
const uint8_t TOO_LONG_TOOL = 8;        // инструмент в шпинделе слишком длинный

const uint8_t GENERAL_ERROR = 255;

// порядковые номера цифр в таблице ASCII
const uint8_t ZERO  = 48;
const uint8_t ONE   = 49;
const uint8_t TWO   = 50;
const uint8_t THREE = 51;
const uint8_t FOUR  = 52;
const uint8_t FIVE  = 53;
const uint8_t SIX   = 54;
const uint8_t SEVEN = 55;
const uint8_t EIGHT = 56;
const uint8_t NINE  = 57;


// константы направления движения для автоматических перемещений
const char A_RIGHT      = 'r';
const char A_LEFT       = 'l';
const char A_FORWARD    = 'f';      // к оператору
const char A_BACK       = 'b';      // от оператора
const char A_UP         = 't';
const char A_DOWN       = 'd';
// константы переключения между подачей и ускорунным перемещением
const int ACCELERATED   = 0;
const int AT_FEED       = 1;

// описываем классы:


class SpeedControl {
  public:
  const uint16_t pulsPerMm = 400;             // импульсов в одном миллиметре
  const uint8_t durHighLevel = 40;             // длительность высокого уровня сигнала на двигателе
  const float qEquilateral = 1.414;           // коэффициент увеличения длительности такта при движении по 2 осям X и Y (равносторонний треугольник)
  const float qNotEquiLateral = 1.153;        // коэффициент при движении по 2 осям с участием оси Z
  const float qThreeAxis = 1.499;             // коэффициент для 3-х осей
  // максимальная допустимая скорость движения для данного станка. измеряется в микросекундах (длина полного такта)
  const uint16_t maxSpeed = 230;
  // длительность сигнала низкого уровня на ускоренном перемещении при движении только по одной из осей
  const uint16_t durLow1AxAccel = maxSpeed - durHighLevel;
  // длительность сигнала низкого уровня на ускоренном перемещении по двум осям (без участия оси Z)
  const uint16_t durLow2AxAccel = (durLow1AxAccel * qEquilateral) - durHighLevel;
  // длительность сигнала низкого уровня на ускоренном перемещении по двум осям (c участием оси Z)
  const uint16_t durLow2AxZAccel = (durLow1AxAccel * qNotEquiLateral) - durHighLevel;
  // длительность сигнала низкого уровня на ускоренном перемещении по трем осям одновременно
  const uint16_t durLow3AxAccel = (durLow1AxAccel * qThreeAxis) - durHighLevel;
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
    if (beatDur1AxZ < maxSpeed) {               // проверяем, нет ли превышения скорости
        beatDur1AxZ = maxSpeed;
    }
    beatDur2Ax = beatDur1Ax * qEquilateral;     // две оси без Z
    beatDur2AxZ = beatDur1Ax * qNotEquiLateral;   // две оси + Z
    beatDur3Ax = beatDur1Ax * qThreeAxis;       // три оси
        //  уровни низкого сигнала для отправки на двигатель
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
  private:
  // данные регулирующие ускорение и замедление
  // длина такта - это сумма времени высокого и низкого сигналов (в микросекундах)
  uint16_t minBeat = 180;       // минимальная длина такта (в микросекундах) - масимальная скорость
  uint16_t curBeat = 0;         // длительность текущего шага
  uint16_t maxBeat = 2600;      // максимальная длина такта в микросекундах - минимальная скорость
  uint16_t durHigh = speedSetting.durHighLevel;     // высокий уровень сигнала хранится в объекте speedSetting
  uint16_t aqAccel = 77;        // коэффициента ускорения и замедления
  uint16_t brakingDistance = 250;  // длина пути торможения и разгон (в шагах)
  uint32_t pathLength = 0;      // общая длина пути
  uint32_t pathMiddle = 0;      // половина пути (чтобы знать, где начинать тормозить)

  public:


  void moveX(uint16_t delayMoveHigh, uint16_t delayMoveLow) {      // метод движения по оси X
    PORTE |= 1 << PORTE4;                 // подаем высокий уровень сигнала на первый мотор
    PORTE |= 1 << PORTE5;                 // подаем высокий уровень сигнала на второй мотор
    delayMicroseconds(delayMoveHigh);
    PORTE &= ~(1 << PORTE4);              // подаем низкий уровень сигнала на первый мотор
    PORTE &= ~(1 << PORTE5);              // подаем низкий уровень сигнала на второй мотор
    delayMicroseconds(delayMoveLow);
    machinePosition.budgeX();             // отслеживаем передвижение в абсолютной системе координат
  }

  void moveY(uint16_t delayMoveHigh, uint16_t delayMoveLow) {      // метод движения по оси Y
    PORTG |= 1 << PORTG5;                 // подаем высокий уровень сигнала на первый мотор
    PORTB |= 1 << PORTB6;                 // подаем высокий уровень сигнала на второй мотор
    delayMicroseconds(delayMoveHigh);
    PORTG &= ~(1 << PORTG5);              // подаем низкий уровень сигнала на первый мотор
    PORTB &= ~(1 << PORTB6);              // подаем низкий уровень сигнала на второй мотор
    delayMicroseconds(delayMoveLow);
    machinePosition.budgeY();             // отслеживаем передвижение в абсолютной системе координат
  }

  void moveZ(uint16_t delayMoveHigh, uint16_t delayMoveLow) {      // метод движения по оси Z
    PORTH |= 1 << PORTH6;                 // подаем высокий уровень сигнала на мотор Z
    delayMicroseconds(delayMoveHigh);
    PORTH &= ~(1 << PORTH6);              // подаем низкий уровень сигнала на мотор Z
    delayMicroseconds(delayMoveLow);
    machinePosition.budgeZ();             // отслеживаем передвижение в абсолютной системе координат
  }

  // метод устанавливает направление движения, устанавливает пины в нужное состояние
  // ,выставляет необходимые флаги и устанавливает скорость движения
  void setMoveParam(const int& g, const int& f, const char& direction) {
    // g: 1/0
    // f: числовое значение подачи
    // direction: t - вверх, d - вниз
    if (g == 1) {
        gSpeed = 1;                 // устанавливаем движение на подаче
        fSpeed = f;                 // записываем в глобальную переменную значение скорости
        speedSetting.setSpeed();    // рассчитываем параметры железа для этой скорости
    } else {
        gSpeed = 0;                 // просто устанавливаем ускоренное перемещение и всё
    }

    if (direction == A_DOWN) {
        zDir = bottomFlag;                  // устанавливаем флаг направления движения "Вниз"
        // digitalWrite(pinDirZ, bottom);      // устанавливаем значение пина, соответствующее данному направлению
        PORTB &= ~(1 << PORTB5);
    } else if (direction == A_UP) {
        zDir = topFlag;                     // устанавливаем флаг направления движения "Вверх"
        // digitalWrite(pinDirZ, top);
        PORTB |= 1 << PORTB5;
    } else if (direction == A_LEFT) {
        xDir = leftFlag;                    // устанавливаем флаг направления "влево"
        digitalWrite(pinDirX1, left);       // устанавливаем значение пинов, соответствующее данному направлению
        digitalWrite(pinDirX2, left);
    } else if (direction == A_RIGHT) {
        xDir = rightFlag;                   // устанавливаем флаг направления "вправо"
        digitalWrite(pinDirX1, right);      // устанавливаем значения пинов направления "вправо"
        digitalWrite(pinDirX2, right);
    } else if (direction == A_FORWARD) {
        yDir = forwardFlag;                 // устанавливаем флаг направления "вперед"
        digitalWrite(pinDirY1, forward);    // устанавливаем значение пинов, соответствующее данному направлению
        digitalWrite(pinDirY2, forward);
    } else if (direction == A_BACK) {
        yDir = backFlag;                    // устанавливаем флаг направления "назад"
        digitalWrite(pinDirY1, back);
        digitalWrite(pinDirY2, back);
    }
  }

  // метод поднимает шпиндель в самый верх (если шпиндель уже не находится в самом верху)
  void toRiseSpindle() {
    setMoveParam(ACCELERATED, 0, A_UP);      // конфигурируем движение вверх на ускоренном ходу
    // находим длину пути и половину длины пути
    findPathLength(machinePosition.getPositionZ(), zDistance);
    // счетчик, сколько шагов уже сделано на пути к G54
    uint32_t counter = 0;
    // устанавливаем начальную (минимальную) скорость страгивания с места
    curBeat = maxBeat;
    while (machinePosition.getPositionZ() <= zDistance ) {
        // рассчитываем скорость ускорения и замедления
        calculateSpeed(counter);
        // делаем шаг
        moveZ(durHigh, (curBeat - durHigh));
        // увеличиваем счетчик сделанных шагов
        counter++;
    }
  }

  // отодвигаем стол в крайнее дальнее положение
  void moveAlongTable() {
    setMoveParam(ACCELERATED, 0, A_BACK);  // конфигурируем движение стола вдаль на ускоренном
    // находим длину пути и половину длины пути
    findPathLength(machinePosition.getPositionY(), 1);
    // счетчик, сколько шагов уже сделано на пути к G54
    uint32_t counter = 0;
    // устанавливаем начальную (минимальную) скорость страгивания с места
    curBeat = maxBeat;
    while (machinePosition.getPositionY() >= 1) {
        // рассчитываем скорость ускорения и замедления
        calculateSpeed(counter);
        // делаем шаг
        moveY(durHigh, (curBeat - durHigh));
        // увеличиваем счетчик сделанных шагов
        counter++;
    }
  }

  // двигаем шпиндель по X до точки смены инструмента
  void moveXToolChange(const int32_t& changePointX) {
    if (machinePosition.getPositionX() < changePointX) {
      // мы находимся слева
      setMoveParam(ACCELERATED, 0, A_RIGHT);  // конфигурируем движение вправо ускор.
    } else if (machinePosition.getPositionX() > changePointX) {
      // мы находимся справа от точки смены инструмента
      setMoveParam(ACCELERATED, 0, A_LEFT);  // конфигурируем движение влево ускор.
    }
    // находим длину пути и половину длины пути
    findPathLength(machinePosition.getPositionX(), changePointX);
    // счетчик, сколько шагов уже сделано на пути к G54
    uint32_t counter = 0;
    // устанавливаем начальную (минимальную) скорость страгивания с места
    curBeat = maxBeat;

    while (machinePosition.getPositionX() != changePointX)
    {
        // рассчитываем скорость ускорения и замедления
        calculateSpeed(counter);
        // делаем шаг
        moveX(durHigh, (curBeat - durHigh));
        // увеличиваем счетчик сделанных шагов
        counter++;
    }
  }

  // двигаем шпиндель по X до G54
  void moveXToG54() {
    // если мы слева от g54
    if (machinePosition.getPositionX() < rPointG54X)
    {
        setMoveParam(ACCELERATED, 0, A_RIGHT);  // конфигурируем движение вправо ускор
    }
    // если мы справа от g54
    else if (machinePosition.getPositionX() > rPointG54X)
    {
        setMoveParam(ACCELERATED, 0, A_LEFT);  // конфигурируем движение влево ускор.
    }

    // находим длину пути и половину длины пути
    findPathLength(machinePosition.getPositionX(), rPointG54X);
    // счетчик, сколько шагов уже сделано на пути к G54
    uint32_t counter = 0;
    // устанавливаем начальную (минимальную) скорость страгивания с места
    curBeat = maxBeat;

    while (machinePosition.getPositionX() != rPointG54X)
    {
        // рассчитываем скорость ускорения и замедления
        calculateSpeed(counter);
        // делаем шаг
        moveX(durHigh, (curBeat - durHigh));
        // увеличиваем счетчик сделанных шагов
        counter++;
    }
  }

  // двигаем шпиндель по Y до G54
  void moveYToG54() {
    if (machinePosition.getPositionY() < rPointG54Y)
    {
        // мы находимся дальше (от оператора) точки g54
        setMoveParam(ACCELERATED, 0, A_FORWARD);  // конфигурируем движение стола вперед на ускоренном
    }
    else if (machinePosition.getPositionY() > rPointG54Y)
    {
        // мы находимся ближе к оператору, чем точка g54
        setMoveParam(ACCELERATED, 0, A_BACK);
    }

    // находим длину пути и половину длины пути
    findPathLength(machinePosition.getPositionY(), rPointG54Y);
    // счетчик, сколько шагов уже сделано на пути к G54
    uint32_t counter = 0;
    // устанавливаем начальную (минимальную) скорость страгивания с места
    curBeat = maxBeat;

    while (machinePosition.getPositionY() != rPointG54Y)
    {
        // рассчитываем скорость ускорения и замедления
        calculateSpeed(counter);
        // делаем шаг
        moveY(durHigh, (curBeat - durHigh));
        // увеличиваем счетчик сделанных шагов
        counter++;
    }
  }

  // метод опускает шпиндель до касания с датчиком инструмента
  uint8_t moveDownUntilTouchSensor() {
    setMoveParam(AT_FEED, 210, A_DOWN);
    // датчик работает на размыкание при нажатии: двигаем шпиндель вниз пока не разомкнет контакты ToolTouchDetected
    while (digitalRead(ToolTouchDetected)) {
        // проверяем на выход за нижний предел оси Z
        if (machinePosition.getPositionZ() <= 0) {
            Serial.println("Over Low Limit Z axis. Tool Sensor nor initialized. Try using a longer tool");
            return OVER_LOW_LIMIT;   // возвращаем код ошибки 1 (выход по оси Z за нижний предел)
        }
        moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));
    }
    return 0;
  }

  // опускаем шпиндель до G54
  void lowerZToG54(const uint32_t& toolLenDif) {
    // конфигурируем движение стола вниз на ускоренном
    setMoveParam(ACCELERATED, 0, A_DOWN);
    // координаты точки назначения по Z
    uint32_t destinationPointZ = rPointG54Z + toolLenDif + spacerHeight;
    // находим длину пути и половину длины пути
    findPathLength(machinePosition.getPositionZ(), destinationPointZ);
    // счетчик, сколько шагов уже сделано на пути к G54
    uint32_t counter = 0;
    // устанавливаем начальную (минимальную) скорость страгивания с места
    curBeat = maxBeat;
    while (machinePosition.getPositionZ() != destinationPointZ) {
        // рассчитываем скорость ускорения и замедления
        calculateSpeed(counter);
        // делаем шаг
        moveZ(durHigh, (curBeat - durHigh));
        // увеличиваем счетчик сделанных шагов
        counter++;
    }
  }

  // метод находит длину пути на основании начальной и конечной позиции
  // а так же длину половины пути
  void findPathLength(uint32_t firstPosition, uint32_t secondPosition)
  {
    // находим длину пути
    if (firstPosition > secondPosition)
    {
        pathLength = firstPosition - secondPosition;
    }
    else
    {
        pathLength = secondPosition - firstPosition;
    }
    // теперь находим длину половины пути
    pathMiddle = pathLength / 2;
  }

    // метод рассчитывает текущую скорость при торможении и ускорении
    void calculateSpeed(uint32_t& counter)
    {
        // если мы еще в первой половине пути
        if (counter < pathMiddle)
        {
            if (curBeat > minBeat)
            {
                curBeat = curBeat - (curBeat / aqAccel);
            }
        }
        // мы уже во второй половине пути
        else
        {
            // уже пришло время тормозить
            if (counter > (pathLength - brakingDistance))
            {
                curBeat = curBeat + (curBeat / aqAccel);
            }
        }
    }

    // метод опускает шпиндель на высоту проставки spacerHeight
    void lowerASpacerHeight()
    {
        // конфигурируем шпиндель для движения вниз
        setMoveParam(AT_FEED, 150, A_DOWN);
        for (uint16_t i = 0; i < spacerHeight; i++)
        {
            moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));
        }
    }

    // метод проверяет, нет ли смещения позиции из-за пропуска шагов
    void checkPosition()
    {
        // скорость движения во время теста
        uint8_t tuneSpeedHigh = 40;
        uint8_t tuneSpeedLow = 210;
        // временные координаты, куда потом вернемся
        const int32_t tempCurX = machinePosition.getPositionX();
        const int32_t tempCurY = machinePosition.getPositionY();
        const int32_t tempCurZ = machinePosition.getPositionZ();
        // счетчики, сколько фактически сделано шагов до касания концевика
        int32_t counterX = 0;
        int32_t counterY = 0;
        int32_t counterZ = 0;
        // разница
        int32_t xDif = 0;
        int32_t yDif = 0;
        int32_t zDif = 0;
        // по каждой из осей двигаемся до касания с концевиком
        // сначала по Z
        setMoveParam(ACCELERATED, 0, A_UP);
        while (!digitalRead(pinLimitSwitchZ)) {
            counterZ++;
            PORTH |= 1 << PORTH6;                 // подаем высокий уровень сигнала на мотор Z
            delayMicroseconds(tuneSpeedHigh);
            PORTH &= ~(1 << PORTH6);              // подаем низкий уровень сигнала на мотор Z
            delayMicroseconds(tuneSpeedLow);
        }
        setMoveParam(ACCELERATED, 0, A_LEFT);
        while (!digitalRead(pinLimitSwitchX))
        {
            counterX++;
            PORTE |= 1 << PORTE4;                 // подаем высокий уровень сигнала на первый мотор
            PORTE |= 1 << PORTE5;                 // подаем высокий уровень сигнала на второй мотор
            delayMicroseconds(tuneSpeedHigh);
            PORTE &= ~(1 << PORTE4);              // подаем низкий уровень сигнала на первый мотор
            PORTE &= ~(1 << PORTE5);              // подаем низкий уровень сигнала на второй мотор
            delayMicroseconds(tuneSpeedLow);
        }
        setMoveParam(ACCELERATED, 0, A_BACK);
        while (!digitalRead(pinLimitSwitchY))
        {
            counterY++;
            PORTG |= 1 << PORTG5;                 // подаем высокий уровень сигнала на первый мотор
            PORTB |= 1 << PORTB6;                 // подаем высокий уровень сигнала на второй мотор
            delayMicroseconds(tuneSpeedHigh);
            PORTG &= ~(1 << PORTG5);              // подаем низкий уровень сигнала на первый мотор
            PORTB &= ~(1 << PORTB6);              // подаем низкий уровень сигнала на второй мотор
            delayMicroseconds(tuneSpeedLow);
        }
        xDif = counterX - tempCurX;
        yDif = counterY - tempCurY;
        zDif = counterZ - (zDistance - tempCurZ);
        Serial.print("xDif: ");
        Serial.println(xDif);
        Serial.print("yDif: ");
        Serial.println(yDif);
        Serial.print("zDif: ");
        Serial.println(zDif);
        Serial.println("formula: counterX - tempCurX");
        Serial.println();
        // возвращаемся обратно
        setMoveParam(ACCELERATED, 0, A_RIGHT);
        while (counterX > 0)
        {
            counterX--;
            PORTE |= 1 << PORTE4;                 // подаем высокий уровень сигнала на первый мотор
            PORTE |= 1 << PORTE5;                 // подаем высокий уровень сигнала на второй мотор
            delayMicroseconds(tuneSpeedHigh);
            PORTE &= ~(1 << PORTE4);              // подаем низкий уровень сигнала на первый мотор
            PORTE &= ~(1 << PORTE5);              // подаем низкий уровень сигнала на второй мотор
            delayMicroseconds(tuneSpeedLow);
        }
        setMoveParam(ACCELERATED, 0, A_FORWARD);
        while (counterY > 0)
        {
            counterY--;
            PORTG |= 1 << PORTG5;                 // подаем высокий уровень сигнала на первый мотор
            PORTB |= 1 << PORTB6;                 // подаем высокий уровень сигнала на второй мотор
            delayMicroseconds(tuneSpeedHigh);
            PORTG &= ~(1 << PORTG5);              // подаем низкий уровень сигнала на первый мотор
            PORTB &= ~(1 << PORTB6);              // подаем низкий уровень сигнала на второй мотор
            delayMicroseconds(tuneSpeedLow);
        }
        setMoveParam(ACCELERATED, 0, A_DOWN);
        while (counterZ > 0)
        {
            counterZ--;
            PORTH |= 1 << PORTH6;                 // подаем высокий уровень сигнала на мотор Z
            delayMicroseconds(tuneSpeedHigh);
            PORTH &= ~(1 << PORTH6);              // подаем низкий уровень сигнала на мотор Z
            delayMicroseconds(tuneSpeedLow);
        }
    }
    // сюда добавлять еще методы
};


AutomaticMove aMove;                // имплементим класс автоматического перемещения


class LimitSwitchTuning {           // класс для настройки нулевой точки станка по концевым выключателям
  uint8_t tuneSpeedHigh = 40;        // скорость движения во время выхода в нулевые позиции
  uint16_t tuneSpeedLow = 140;

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
        PORTE |= 1 << PORTE4;                 // подаем высокий уровень сигнала на первый мотор
        PORTE |= 1 << PORTE5;                 // подаем высокий уровень сигнала на второй мотор
        delayMicroseconds(tuneSpeedHigh);
        PORTE &= ~(1 << PORTE4);              // подаем низкий уровень сигнала на первый мотор
        PORTE &= ~(1 << PORTE5);              // подаем низкий уровень сигнала на второй мотор
        delayMicroseconds(tuneSpeedLow);
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
        PORTG |= 1 << PORTG5;                 // подаем высокий уровень сигнала на первый мотор
        PORTB |= 1 << PORTB6;                 // подаем высокий уровень сигнала на второй мотор
        delayMicroseconds(tuneSpeedHigh);
        PORTG &= ~(1 << PORTG5);              // подаем низкий уровень сигнала на первый мотор
        PORTB &= ~(1 << PORTB6);              // подаем низкий уровень сигнала на второй мотор
        delayMicroseconds(tuneSpeedLow);
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
        PORTH |= 1 << PORTH6;                 // подаем высокий уровень сигнала на мотор Z
        delayMicroseconds(tuneSpeedHigh);
        PORTH &= ~(1 << PORTH6);              // подаем низкий уровень сигнала на мотор Z
        delayMicroseconds(tuneSpeedLow);
      }
    }
  }
};

LimitSwitchTuning limitSwitchTuning;    // имплементим объект вывода станка в нулевые позиции по всем осям


class synFrameProc {                // класс распределяет шаги в кадре
  public:

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
        if (yStepsDone * xStepsFrameABS > xStepsDone * yStepsFrameABS) {  // коэффициент по мастер-оси меньше второй оси?
          sfpMoveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));          // догоняем мастер ось движением только по одной мастер-оси
        } else {                                                                // а если коэффициенты мастер-оси и второй оси равны, или коэффициент мастер-оси больше
          sfpMoveXY(speedSetting.durHighLevel, speedSetting.getSpeed('x', 'y'));         // двикоем сразу две оси
        }
      }
    } else {
      for (uint32_t i = 1; i < yStepsFrameABS; i++) {                           // количество тактов в кадре равно количеству шагов в кадре по оси с наибольшим количеством шагов
        if (xStepsDone * yStepsFrameABS > yStepsDone * xStepsFrameABS) {  // коэффициент по мастер-оси меньше второй оси?
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
        if (zStepsDone * xStepsFrameABS > xStepsDone * zStepsFrameABS) {    // коэффициент по мастер-оси меньше второй оси?
          sfpMoveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));          // догоняем мастер ось движением только по одной мастер-оси
        } else {                                                                // а если коэффициенты мастер-оси и второй оси равны, или коэффициент мастер-оси больше
          sfpMoveXZ(speedSetting.durHighLevel, speedSetting.getSpeed('x', 'z'));        // двигаем сразу две оси
        }
      }
    } else {
      for (uint32_t i = 1; i < zStepsFrameABS; i++) {                           // количество тактов в кадре равно количеству шагов в кадре по оси с наибольшим количеством шагов
        if (xStepsDone * zStepsFrameABS > zStepsDone * xStepsFrameABS) {    // коэффициент по мастер-оси меньше второй оси?
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
        if (zStepsDone * yStepsFrameABS > yStepsDone * zStepsFrameABS) {    // коэффициент по мастер-оси меньше второй оси?
          sfpMoveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));          // догоняем мастер ось движением только по одной мастер-оси
        } else {                                                                // а если коэффициенты мастер-оси и второй оси равны, или коэффициент мастер-оси больше
          sfpMoveYZ(speedSetting.durHighLevel, speedSetting.getSpeed('y', 'z'));        // двигаем сразу две оси
        }
      }
    } else {
      for (uint32_t i = 1; i < zStepsFrameABS; i++) {                           // количество тактов в кадре равно количеству шагов в кадре по оси с наибольшим количеством шагов
        if (yStepsDone * zStepsFrameABS > zStepsDone * yStepsFrameABS) {    // коэффициент по мастер-оси меньше второй оси?
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
        if (yStepsDone * xStepsFrameABS > xStepsDone * yStepsFrameABS) {      // коэффициент по мастер-оси меньше второй оси?
          isNeedMoveY = false;                                                            // в этой паре осей, в этом такте крутим только мастер-ось
        } else {                                                                          // коэффициент по мастер-оси равен, или больше второй оси?
          isNeedMoveY = true;                                                             // в этой паре осей, в этом такте крутим обе оси
        }
        // теперь анализируем вторую пару осей, одна из которых мастер-ось
        if (zStepsDone * xStepsFrameABS > xStepsDone * zStepsFrameABS) {      // коэффициент по мастер-оси меньше второй оси?
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
        if (xStepsDone * yStepsFrameABS > yStepsDone * xStepsFrameABS) {      // коэффициент по мастер-оси меньше второй оси?
          isNeedMoveX = false;                                                            // в этой паре осей, в этом такте крутим только мастер-ось
        } else {                                                                          // коэффициент по мастер-оси равен, или больше второй оси?
          isNeedMoveX = true;                                                             // в этой паре осей, в этом такте крутим обе оси
        }
        // теперь анализируем вторую пару осей, одна из которых мастер-ось
        if (zStepsDone * yStepsFrameABS > yStepsDone * zStepsFrameABS) {      // коэффициент по мастер-оси меньше второй оси?
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
        if (yStepsDone * zStepsFrameABS > zStepsDone * yStepsFrameABS) {      // коэффициент по мастер-оси меньше второй оси?
          isNeedMoveY = false;                                                            // в этой паре осей, в этом такте крутим только мастер-ось
        } else {                                                                          // коэффициент по мастер-оси равен, или больше второй оси?
          isNeedMoveY = true;                                                             // в этой паре осей, в этом такте крутим обе оси
        }
        // теперь анализируем вторую пару осей, одна из которых мастер-ось
        if (xStepsDone * zStepsFrameABS > zStepsDone * xStepsFrameABS) {      // коэффициент по мастер-оси меньше второй оси?
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
    if (!ovfX) {    // если таймер еще не переполнялся ни разу
        while (TCNT3 < moveSpeedLow * 2) {
            // ждем пока счетчик таймера отсчитает нужное время
            if (ovfX) {break;}  // защита от зависания
        }
    }
    PORTE |= 1 << PORTE4;                 // подаем высокий уровень сигнала на первый мотор
    PORTE |= 1 << PORTE5;                 // подаем высокий уровень сигнала на второй мотор
    delayMicroseconds(moveSpeedHigh);
    PORTE &= ~(1 << PORTE4);              // подаем низкий уровень сигнала на первый мотор
    PORTE &= ~(1 << PORTE5);              // подаем низкий уровень сигнала на второй мотор
    TCNT3 = 0x0000;     // начинаем считать, сколько прошло времени от завершения этого шага
    ovfX = false;       // обнуляем флаг переполнения таймера 3
    xStepsDone++;                  // считаем количество фактически выполненных шагов в текущем кадре
//     xDoneToTime++;              // считаем количества шагов от начала программы и до текущего момента
    machinePosition.budgeX();      // отслеживаем перемещение в абсолютной системе координат станка
  }


  void sfpMoveY(uint16_t moveSpeedHigh, uint16_t moveSpeedLow) {     // метод вращает двигатель только по одной оси Y
    if (!ovfY) {    // если таймер еще не переполнялся ни разу
        while (TCNT4 < moveSpeedLow * 2) {
            // ждем пока счетчик таймера отсчитает нужное время
            if (ovfY) {break;}  // защита от зависания
        }
    }
    PORTG |= 1 << PORTG5;                 // подаем высокий уровень сигнала на первый мотор
    PORTB |= 1 << PORTB6;                 // подаем высокий уровень сигнала на второй мотор
    delayMicroseconds(moveSpeedHigh);
    PORTG &= ~(1 << PORTG5);              // подаем низкий уровень сигнала на первый мотор
    PORTB &= ~(1 << PORTB6);              // подаем низкий уровень сигнала на второй мотор
    TCNT4 = 0x0000;     // начинаем считать, сколько прошло времени от завершения этого шага
    ovfY = false;       // обнуляем флаг переполнения таймера 4
    yStepsDone++;                  // считаем количество фактически выполненных шагов в текущем кадре
//     yDoneToTime++;                 // считаем количества шагов от начала программы и до текущего момента
    machinePosition.budgeY();      // отслеживаем перемещение в абсолютной системе координат станка
  }


  void sfpMoveZ(uint16_t moveSpeedHigh, uint16_t moveSpeedLow) {     // метод вращает двигатель только по одной оси Z
    if (!ovfZ) {    // если таймер еще не переполнялся ни разу
        while (TCNT5 < moveSpeedLow * 2) {
            // ждем пока счетчик таймера отсчитает нужное время
            if (ovfZ) {break;}  // защита от зависания
        }
    }
    PORTH |= 1 << PORTH6;                 // подаем высокий уровень сигнала на мотор Z
    delayMicroseconds(moveSpeedHigh);
    PORTH &= ~(1 << PORTH6);              // подаем низкий уровень сигнала на мотор Z
    TCNT5 = 0x0000;     // начинаем считать, сколько прошло времени от завершения этого шага
    ovfZ = false;       // обнуляем флаг переполнения таймера 4
    zStepsDone++;                  // считаем количество фактически выполненных шагов в текущем кадре
//     zDoneToTime++;                 // считаем количества шагов от начала программы и до текущего момента
    machinePosition.budgeZ();      // отслеживаем перемещение в абсолютной системе координат станка
  }


  void sfpMoveXY(uint16_t moveSpeedHigh, uint16_t moveSpeedLow) {     // метод вращает двигатель по двум осям: X и Y
    if (!ovfX) {    // если таймер еще не переполнялся ни разу
        while (TCNT3 < moveSpeedLow * 2) {
            // ждем пока счетчик таймера отсчитает нужное время
            if (ovfX) {break;}  // защита от зависания
        }
    }
    if (!ovfY) {    // если таймер еще не переполнялся ни разу
        while (TCNT4 < moveSpeedLow * 2) {
            // ждем пока счетчик таймера отсчитает нужное время
            if (ovfY) {break;}  // защита от зависания
        }
    }
    PORTE |= 1 << PORTE4;                 // X подаем высокий уровень сигнала на первый мотор
    PORTE |= 1 << PORTE5;                 // X подаем высокий уровень сигнала на второй мотор
    PORTG |= 1 << PORTG5;                 // Y подаем высокий уровень сигнала на первый мотор
    PORTB |= 1 << PORTB6;                 // y подаем высокий уровень сигнала на второй мотор
    delayMicroseconds(moveSpeedHigh);
    PORTE &= ~(1 << PORTE4);              // X подаем низкий уровень сигнала на первый мотор
    PORTE &= ~(1 << PORTE5);              // X подаем низкий уровень сигнала на второй мотор
    PORTG &= ~(1 << PORTG5);              // Y подаем низкий уровень сигнала на первый мотор
    PORTB &= ~(1 << PORTB6);              // Y подаем низкий уровень сигнала на второй мотор
    TCNT3 = 0x0000;     // начинаем считать, сколько прошло времени от завершения этого шага по X
    TCNT4 = 0x0000;     // начинаем считать, сколько прошло времени от завершения этого шага по Y
    ovfX = false;       // обнуляем флаг переполнения таймера 3
    ovfY = false;       // обнуляем флаг переполнения таймера 4
    xStepsDone++;                  // считаем количество фактически выполненных шагов в текущем кадре
    yStepsDone++;
//     xDoneToTime++;                 // считаем количества шагов от начала программы и до текущего момента
//     yDoneToTime++;
    machinePosition.budgeX();      // отслеживаем перемещение в абсолютной системе координат станка
    machinePosition.budgeY();
  }


  void sfpMoveXZ(uint16_t moveSpeedHigh, uint16_t moveSpeedLow) {     // метод вращает двигатель по двум осям: X и Z
    if (!ovfX) {    // если таймер еще не переполнялся ни разу
        while (TCNT3 < moveSpeedLow * 2) {
            // ждем пока счетчик таймера отсчитает нужное время
            if (ovfX) {break;}  // защита от зависания
        }
    }
    if (!ovfZ) {    // если таймер еще не переполнялся ни разу
        while (TCNT5 < moveSpeedLow * 2) {
            // ждем пока счетчик таймера отсчитает нужное время
            if (ovfZ) {break;}  // защита от зависания
        }
    }
    PORTE |= 1 << PORTE4;                 // X подаем высокий уровень сигнала на первый мотор
    PORTE |= 1 << PORTE5;                 // X подаем высокий уровень сигнала на второй мотор
    PORTH |= 1 << PORTH6;                 // Z подаем высокий уровень сигнала на мотор Z
    delayMicroseconds(moveSpeedHigh);
    PORTE &= ~(1 << PORTE4);              // X подаем низкий уровень сигнала на первый мотор
    PORTE &= ~(1 << PORTE5);              // X подаем низкий уровень сигнала на второй мотор
    PORTH &= ~(1 << PORTH6);              // Z подаем низкий уровень сигнала на мотор Z
    TCNT3 = 0x0000;     // начинаем считать, сколько прошло времени от завершения этого шага по X
    TCNT5 = 0x0000;     // начинаем считать, сколько прошло времени от завершения этого шага по Z
    ovfX = false;       // обнуляем флаг переполнения таймера 3
    ovfZ = false;       // обнуляем флаг переполнения таймера 5
    xStepsDone++;                  // считаем количество фактически выполненных шагов в текущем кадре
    zStepsDone++;
//     xDoneToTime++;                 // считаем количества шагов от начала программы и до текущего момента
//     zDoneToTime++;
    machinePosition.budgeX();      // отслеживаем перемещение в абсолютной системе координат станка
    machinePosition.budgeZ();
  }


  void sfpMoveYZ(uint16_t moveSpeedHigh, uint16_t moveSpeedLow) {     // метод вращает двигатель по двум осям: Y и Z
    if (!ovfY) {    // если таймер еще не переполнялся ни разу
        while (TCNT4 < moveSpeedLow * 2) {
            // ждем пока счетчик таймера отсчитает нужное время
            if (ovfY) {break;}  // защита от зависания
        }
    }
    if (!ovfZ) {    // если таймер еще не переполнялся ни разу
        while (TCNT5 < moveSpeedLow * 2) {
            // ждем пока счетчик таймера отсчитает нужное время
            if (ovfZ) {break;}  // защита от зависания
        }
    }
    PORTG |= 1 << PORTG5;                 // Y подаем высокий уровень сигнала на первый мотор
    PORTB |= 1 << PORTB6;                 // Y подаем высокий уровень сигнала на второй мотор
    PORTH |= 1 << PORTH6;                 // Z подаем высокий уровень сигнала на мотор Z
    delayMicroseconds(moveSpeedHigh);
    PORTG &= ~(1 << PORTG5);              // Y подаем низкий уровень сигнала на первый мотор
    PORTB &= ~(1 << PORTB6);              // Y подаем низкий уровень сигнала на второй мотор
    PORTH &= ~(1 << PORTH6);              // Z подаем низкий уровень сигнала на мотор Z
    TCNT4 = 0x0000;     // начинаем считать, сколько прошло времени от завершения этого шага по Y
    TCNT5 = 0x0000;     // начинаем считать, сколько прошло времени от завершения этого шага по Z
    ovfY = false;       // обнуляем флаг переполнения таймера 4
    ovfZ = false;       // обнуляем флаг переполнения таймера 5
    yStepsDone++;                  // считаем количество фактически выполненных шагов в текущем кадре
    zStepsDone++;
//     yDoneToTime++;                 // считаем количества шагов от начала программы и до текущего момента
//     zDoneToTime++;
    machinePosition.budgeY();      // отслеживаем перемещение в абсолютной системе координат станка
    machinePosition.budgeZ();
  }


  void sfpMoveXYZ(uint16_t moveSpeedHigh, uint16_t moveSpeedLow) {     // метод вращает двигатель по трем осям: X, Y и Z
    if (!ovfX) {    // если таймер еще не переполнялся ни разу
        while (TCNT3 < moveSpeedLow * 2) {
            // ждем пока счетчик таймера отсчитает нужное время
            if (ovfX) {break;}  // защита от зависания
        }
    }
    if (!ovfY) {    // если таймер еще не переполнялся ни разу
        while (TCNT4 < moveSpeedLow * 2) {
            // ждем пока счетчик таймера отсчитает нужное время
            if (ovfY) {break;}  // защита от зависания
        }
    }
    if (!ovfZ) {    // если таймер еще не переполнялся ни разу
        while (TCNT5 < moveSpeedLow * 2) {
            // ждем пока счетчик таймера отсчитает нужное время
            if (ovfZ) {break;}  // защита от зависания
        }
    }
    PORTE |= 1 << PORTE4;                 // X подаем высокий уровень сигнала на первый мотор
    PORTE |= 1 << PORTE5;                 // X подаем высокий уровень сигнала на второй мотор
    PORTG |= 1 << PORTG5;                 // Y подаем высокий уровень сигнала на первый мотор
    PORTB |= 1 << PORTB6;                 // y подаем высокий уровень сигнала на второй мотор
    PORTH |= 1 << PORTH6;                 // Z подаем высокий уровень сигнала на мотор Z
    delayMicroseconds(moveSpeedHigh);
    PORTE &= ~(1 << PORTE4);              // X подаем низкий уровень сигнала на первый мотор
    PORTE &= ~(1 << PORTE5);              // X подаем низкий уровень сигнала на второй мотор
    PORTG &= ~(1 << PORTG5);              // Y подаем низкий уровень сигнала на первый мотор
    PORTB &= ~(1 << PORTB6);              // Y подаем низкий уровень сигнала на второй мотор
    PORTH &= ~(1 << PORTH6);              // Z подаем низкий уровень сигнала на мотор Z
    TCNT3 = 0x0000;     // начинаем считать, сколько прошло времени от завершения этого шага
    TCNT4 = 0x0000;     // начинаем считать, сколько прошло времени от завершения этого шага
    TCNT5 = 0x0000;     // начинаем считать, сколько прошло времени от завершения этого шага
    ovfX = false;       // обнуляем флаг переполнения таймера 3
    ovfY = false;       // обнуляем флаг переполнения таймера 4
    ovfZ = false;       // обнуляем флаг переполнения таймера 5
    xStepsDone++;                  // считаем количество фактически выполненных шагов в текущем кадре
    yStepsDone++;
    zStepsDone++;
//     xDoneToTime++;                 // считаем количества шагов от начала программы и до текущего момента
//     yDoneToTime++;
//     zDoneToTime++;
    machinePosition.budgeX();      // отслеживаем перемещение в абсолютной системе координат станка
    machinePosition.budgeY();
    machinePosition.budgeZ();
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
  const uint16_t thirdSpeedManual = 73;
  const uint16_t highsval = 40;          // пауза для высокого уровня сигнала (для всех скоростей одинаковая)

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
    uint8_t breakDist = 237;                            // длина тормозного пути в шагах (зависит от коэффициента замедления)
    uint8_t stopOutWorkArea = digitalRead(pinEnTuning); // проверяем, включено ли ограничение выхода за пределы рабочего пространства

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
  int32_t changePointX = 0;             // координаты точки смены инструмента
  int32_t changePointY = 0;
  int32_t changePointZ = 0;
  int32_t curToolEnd = 0;       // координаты торца текущего инструмента
  int32_t toolLenDif = 0;               // разница длины нового и предыдущего инструмента

  // метод продолжает программу после смены инструмента в середине программы:
  // метод должен возвращать код успешности выполнения задачи:
  //        если инструмент нужной длины, метод в конце возвращает 0 (успешно)
  //        если длина инструмента не подходит, возвращается код ошибки
  //            (7 - слишком короткий, 8 - слишком длинный)
    // - если необходимо, поднимаем шпиндель в самый верх
    // - если необходимо, отодвигаем стол в крайнее дальнее положение
    // - если необходимо, подводим шпиндель по оси X, причем шпиндель может находиться
    //      как слева от датчика касания инструмента, так и справа
    // опускает новый инструмент до момента срабатывания датчика касания инструмента (на подаче)
    // записываем координаты торца инструмента (инициализируем)
    // поднимаем его на несколько миллиметров
    // вычисляем разницу длины предыдущего и нынешнего инструмента
    // сравниваем с точкой начала координат заготовки и:
    //      если длина инструмента не подходит,
    //          завершаем метод (return), возвращая код ошибки (const)
    //      если всё в порядке, просто продолжаем выполнять функцию (и даже не обрабатываем это условие)
    // далее на ускоренном ходу поднмает инструмента в самый верх
    // ведет шпиндель сначала по оси X, а затем по Y до точки начала координат заготовки
    // опускает шпиндель до точки начала координат заготовки
    // дальше просто эта функция завершается и .ncm программа продолжается как обычно
  uint8_t continueProgram() {
    aMove.toRiseSpindle();        // поднимаем шпиндель
    aMove.moveAlongTable();        // отодвигаем стол
    aMove.moveXToolChange(changePointX);    // двигаем шпиндель по X до точки смены инструмента
    // опускаем шпиндель до касания датчика инструмента
    if (aMove.moveDownUntilTouchSensor()) {
        aMove.toRiseSpindle();        // поднимаем шпиндель
        return GENERAL_ERROR;
    }
    int32_t tempCurToolEnd = machinePosition.getPositionZ();
    int32_t tempToolLenDif = tempCurToolEnd - changePointZ;

    bool whatIsIt =
            (
                static_cast<int32_t>(rPointG54Z) +
                static_cast<int32_t>(tempToolLenDif)
            )
            > static_cast<int32_t>(zDistance);
    Serial.println(whatIsIt);

    if  (
            (
                static_cast<int32_t>(rPointG54Z) +
                static_cast<int32_t>(tempToolLenDif)
            )
            > static_cast<int32_t>(zDistance)
        ) {
        Serial.println("Tool is too long");
        return TOO_LONG_TOOL;
    } else if ((rPointG54Z + tempToolLenDif) < 0) {
        Serial.println("Tool is too short");
        return TOO_SHORT_TOOL;
    }

    curToolEnd = tempCurToolEnd;
    toolLenDif = tempToolLenDif;

    Serial.print("curToolEnd: ");
    Serial.println(curToolEnd);
    Serial.print("toolLenDif: ");
    Serial.println(toolLenDif);

    aMove.toRiseSpindle();        // поднимаем шпиндель
    aMove.moveXToG54();       // двигаемся до G54 по всем трем осям
    aMove.moveYToG54();
    aMove.lowerZToG54(toolLenDif);
    // дополнительно опускаем шпиндель от G54 еще на глубину проставки
    aMove.lowerASpacerHeight();

    Serial.println("The tool is successfully installed!");
    return 0;   // возвращаем успешный код выполнения
  }

  // метод устанавливает координаты точки смены инструмента
  uint8_t setToolSensorPoint() {
    // поиск координат датчика касания инструмента начинается включением тумблера pinSetToolSensor - пин 40
    // для инициализации датчика надо вручную подвести шпиндель над датчиком и нажать тумблер pinSetToolSensor - движение в этой
    // функции это вниз до касания датчика. Записываются все три координаты - это и есть точка сверки длин инструментов
    // После касания инструментом датчика, поднимаем инструмент на пару миллиметров Z

    // опускаем шпиндель до касапния датчика инструмента
    if (aMove.moveDownUntilTouchSensor()) {
        return GENERAL_ERROR;
    }
    // всё, инструмент коснулся датчика, записываем все три координаты этого местоположения
    changePointX = machinePosition.getPositionX();
    changePointY = machinePosition.getPositionY();
    changePointZ = machinePosition.getPositionZ();
    curToolEnd = machinePosition.getPositionZ();    // записываем координаты торца текущего инструмента
    // теперь приподнимаем инструмент над датчиком на несколько миллиметров
    if (raiseFewMilliveters(2)) {
        return GENERAL_ERROR;
    }
    Serial.println("Tool change sensor initialized.");
    toolLenDif = 0;                         // обнуляем разницу длины инструментов
    return 0;
  }

  // поднять шпиндель на несколько миллиметров
  uint8_t raiseFewMilliveters(uint8_t distance) {
    int zStepsInMm      = 800;                          // количество шагов в 1мм по оси Z
    int finalDistance   = zStepsInMm * distance;        // шпиндель поднимется на distance миллиметров
    aMove.setMoveParam(ACCELERATED, 0, A_UP);  // инициализируем характер движения
    for (int i = 0; i < finalDistance; i++) {
        if (machinePosition.getPositionZ() >= zDistance) {
            Serial.println("Over High Limit Z axis. (raiseFewMilliveters())");
            return OVER_HIGH_LIMIT;
        }
        aMove.moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));
    }
    return 0;
  }
};

ToolChangePoint changeP;                // имплементим точку смены инструмента


class ReferentPoint {
  public:

  void setReferentialPointG54() {    // устанавливаем референтную точку
    // Предполагается, что референтная точка устанавливается через параллельку толщиной 5мм.
    // Следовательно, реальная референтная точка будет на 5мм (4000 шагов
    // при микрошаге 1/8) ниже, чем текущее положение шпинделя (только для оси Z)
    rPointG54X = machinePosition.getPositionX();
    rPointG54Y = machinePosition.getPositionY();
    rPointG54Z = machinePosition.getPositionZ() - spacerHeight;
    Serial.println("G54 point is installed successfully.");
    // пауза, чтобы в порт не летело бесконечное количество сообщений
        delay(1000);
  }

  void goToRPoint() {       // метод ведет шпиндель к референтной точке
    // проверяем, была ли установлена точка G54
    if (rPointG54X && rPointG54X && rPointG54X)
    {
        gSpeed = 0;         // декларируем, что перемещения сейчас будут на ускоренном ходу
        //!!!
        aMove.toRiseSpindle();    // сначала надо поднять шпиндель в самый верх (если референтная точка находится ниже максимальной высоты оси Z)
        aMove.moveXToG54();       // теперь, когда шпиндель вверху, перемещаемся по оси X до референтной точки (если в этом есть необходимость)
        aMove.moveYToG54();       // перемещаемся по оси Y до референтной точки (если в этом есть необходимость)
        aMove.lowerZToG54(changeP.toolLenDif);    // опускаем шпиндель по оси Z до референтной точки
                            // (если референтная точка ниже максимальной высоты оси Z)
                            // недоход до реф. точки составит величину spacerHeight (высота параллельки для выставления реф. точки)
                            // изначально эта величина планируется 5мм (4000 шагов)
    }
    else
    {
        Serial.println("G54 point coordinates are not set");
        // пауза, чтобы в порт не летело бесконечное количество сообщений
        delay(1000);
    }
  }
};

ReferentPoint refPoint;               // имплементим класс "Референтная точка"


// прерывания по переполнению
ISR(TIMER3_OVF_vect)
{
  ovfX = true;
}
ISR(TIMER4_OVF_vect)
{
  ovfY = true;
}
ISR(TIMER5_OVF_vect)
{
  ovfZ = true;
}

// заготовки прерываний по сравнению с регистрами COMPA 3, 4 и 5 таймеров
// ISR(TIMER3_COMPA_vect) {}
// ISR(TIMER4_COMPA_vect) {}
// ISR(TIMER5_COMPA_vect) {}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.setTimeout(3);

  // инициализируем таймер 3
  // включаем питание на таймере 3, 4 и 5 (по умолчанию с завода питиние
  // как правило должно быть включено, но мы на всякий случай это проверим)
  // чтобы включить питание, в регистр PRR1 (Power Reduction Register 1),
  // в биты под номерами 3, 4 и 5 PRTIM3, PRTIM4 и PRTIM5 (Power Reduction Timer/Counter)
  // нужно записать логический 0
  PRR1 &= ~(1 << PRTIM3);
  PRR1 &= ~(1 << PRTIM4);
  PRR1 &= ~(1 << PRTIM5);
  // предварительно зануляем все управляющие регистры таймеров
  // для защиты от случайных значений
  TCCR3A = 0x00;
  TCCR3B = 0x00;
  TCCR3C = 0x00;
  TCCR4A = 0x00;
  TCCR4B = 0x00;
  TCCR4C = 0x00;
  TCCR5A = 0x00;
  TCCR5B = 0x00;
  TCCR5C = 0x00;
  // устанавливаем предделитель частоты 1/8 (0b00000010)
  TCCR3B |= 1 << CS31;
  TCCR4B |= 1 << CS41;
  TCCR5B |= 1 << CS51;
  // инициализируем регистр с масками прерываний
  TIMSK3 = 0x00;    // на всякий случай обнуляем от случайных значений
  TIMSK4 = 0x00;
  TIMSK5 = 0x00;
  // разрешаем прерывания:
  // по переполнению
  TIMSK3 |= 1 << TOIE3;
  TIMSK4 |= 1 << TOIE4;
  TIMSK5 |= 1 << TOIE3;
  // по сравнению c регистром сравнения A (OCRnA)
  // TIMSK3 |= 1 << OCIE3A;
  // TIMSK4 |= 1 << OCIE4A;
  // TIMSK5 |= 1 << OCIE5A;
  // для справки:
  // счетные регистры:      TCNT3, TCNT4, TCNT5
  // регистры сравнения:    OCR3A, OCR4A, OCR5A
  //                        OCR3B, OCR4B, OCR4B
  //                        OCR3C, OCR4C, OCR5C

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
  pinMode(pinTouchProbe, INPUT);        // пин, снимает показания с датчика 3D Touch Probe
  pinMode(startSearchG54Rectangle, INPUT);  // тумблер, старт поиска нулевой точки заготовки в полуавтоматическом режиме

  pinMode(pinGoChangePoint, INPUT);     // кнопка, идем к точке смены инструмента по X
  pinMode(pinSetToolSensor, INPUT);     // тумблер, запускает процесс инициализации датчика касания инструмента
  pinMode(pinAutoSetTool, INPUT);       // кнопка, продолжает программу после смены инструмента
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


int32_t absNum(int32_t num) {        // возвращает значение числа шагов в кадре по модулю (без учета знака +\-)
  if (num >= 0) {
    return num;
  } else {
    return -num;
  }
}

// открываем файл для чтения
void openNcmFile()
{
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
}

void read_line_sd()
{
    // обнуляем значения количества шагов, чтобы не складывалось со значениями из прошого кадра
    // обнулять надо, этого требует алгоритм заполнения этой группы переменных
    xStepsFrame = 0;
    yStepsFrame = 0;
    zStepsFrame = 0;
    xStepsFrameABS = 0;
    yStepsFrameABS = 0;
    zStepsFrameABS = 0;

    char curChar = ncFile.read();   // читаем символ из .ncm файла

    while (true) {                  // поштучно читаем символы из файла
        if (curChar == '\n') {
            break;                  // заканчиваем читать этот кадр
        } else if (curChar == 'X') {                // в строке нашли координату X
            char xStepsTemp[7];                     // количество шагов в кадре по X (массив char)
            // номер разряда числа, куда будет записан прочитанный символ (в xStepsTemp)
            uint8_t i = 0;
            while (true) {
                curChar = ncFile.read();            // читаем очередной символ
                // если прочитанный символ - это число
                if (curChar >= ZERO && curChar <= NINE) {
                    xStepsTemp[i] = curChar;        // записываем символ в соответствующир разряд числа
                    i++;                            // увеличиваем разряд для следующей цифры
                    continue;                       // переходим к чтению следующего символа в .ncm файле
                } else if (curChar == '-') {        // в текущей координате обнаружен знак минус
                    minusX = true;
                    continue;                       // переходим к чтению следующего символа в .ncm файле
                } else {
                    // если мы пришли сюда, значит все цифры числа прочитаны полностью
                    // переводим массив символов в число:
                    // создаем полноценную строку добавлением символа конца строки (без этого atol() не работает)
                    xStepsTemp[i] = '\0';
                    xStepsFrame = atol(xStepsTemp); // массив символов переводим в числовой тип
                    xStepsFrameABS = xStepsFrame;   // пока еще не добавили знак минуса количеству шагов по координате,
                                                    // даже если она и отрицательная, записываем значение ABS
                    if (minusX) {
                        xStepsFrame = -xStepsFrame;
                        minusX = false;             // обнуляем значение флага
                    }
                    break;                          // Выходим из цикла поиска всех цифр за буквой
                }
            }
        } else if (curChar == 'Y') {                // в строке нашли координату Y
            char yStepsTemp[7];                     // количество шагов в кадре по Y (массив char)
            // номер разряда числа, куда будет записан прочитанный символ (в yStepsTemp)
            uint8_t i = 0;
            while (true) {
                curChar = ncFile.read();            // читаем очередной символ
                // если прочитанный символ - это число
                if (curChar >= ZERO && curChar <= NINE) {
                    yStepsTemp[i] = curChar;        // записываем символ в соответствующир разряд числа
                    i++;                            // увеличиваем разряд для следующей цифры
                    continue;                       // переходим к чтению следующего символа в .ncm файле
                } else if (curChar == '-') {        // в текущей координате обнаружен знак минус
                    minusY = true;
                    continue;                       // переходим к чтению следующего символа в .ncm файле
                } else {
                    // если мы пришли сюда, значит все цифры числа прочитаны полностью
                    // переводим массив символов в число:
                    // создаем полноценную строку добавлением символа конца строки (без этого atol() не работает)
                    yStepsTemp[i] = '\0';
                    yStepsFrame = atol(yStepsTemp); // массив символов переводим в числовой тип
                    yStepsFrameABS = yStepsFrame;   // пока еще не добавили знак минуса количеству шагов по координате,
                                                    // даже если она и отрицательная, записываем значение ABS
                    if (minusY) {
                        yStepsFrame = -yStepsFrame;
                        minusY = false;             // обнуляем значение флага
                    }
                    break;                          // Выходим из цикла поиска всех цифр за буквой
                }
            }
        }else if (curChar == 'Z') {
            char zStepsTemp[7];                     // количество шагов в кадре по Z (массив char)
            // номер разряда числа, куда будет записан прочитанный символ (в zStepsTemp)
            uint8_t i = 0;
            while (true) {
                curChar = ncFile.read();            // читаем очередной символ
                // если прочитанный символ - это число
                if (curChar >= ZERO && curChar <= NINE) {
                    zStepsTemp[i] = curChar;        // записываем символ в соответствующир разряд числа
                    i++;                            // увеличиваем разряд для следующей цифры
                    continue;                       // переходим к чтению следующего символа в .ncm файле
                } else if (curChar == '-') {        // в текущей координате обнаружен знак минус
                    minusZ = true;
                    continue;                       // переходим к чтению следующего символа в .ncm файле
                } else {
                    // если мы пришли сюда, значит все цифры числа прочитаны полностью
                    // переводим массив символов в число:
                    // создаем полноценную строку добавлением символа конца строки (без этого atol() не работает)
                    zStepsTemp[i] = '\0';
                    zStepsFrame = atol(zStepsTemp); // массив символов переводим в числовой тип
                    zStepsFrameABS = zStepsFrame;   // пока еще не добавили знак минуса количеству шагов по координате,
                                                    // даже если она и отрицательная, записываем значение ABS
                    if (minusZ) {
                        zStepsFrame = -zStepsFrame;
                        minusZ = false;             // обнуляем значение флага
                    }
                    break;                          // Выходим из цикла поиска всех цифр за буквой
                }
            }
        } else if (curChar == 'G') {
            char gTemp[3];                          // код G (массив char)
            // номер разряда числа, куда будет записан прочитанный символ (в gTemp)
            uint8_t i = 0;
            while (true) {
                curChar = ncFile.read();            // читаем следующий символ
                // если прочитанный символ число:
                if (curChar >= ZERO && curChar <= NINE) {
                    gTemp[i] = curChar;
                    i++;                            // увеличиваем разряд для следующей цифры
                    continue;                       // переходим к чтению следующего символа в .ncm файле
                } else {                            // Всё число записано во временную переменную полностью и пришло время узнаь, что же это за команда G
                    // создаем полноценную строку добавлением символа конца строки (без этого atol() не работает)
                    gTemp[i] = '\0';
                    uint8_t gTempInt = atol(gTemp); // массив символов переводим в числовой тип
                    // Если это команда G: ускоренного перемещения || скорости на подаче
                    if (gTempInt == ACCELERATED || gTempInt == AT_FEED) {
                        gSpeed = gTempInt;
                    }
                    break;                          // Выходим из цикла поиска всех цифр за буквой
                }
            }
        } else if (curChar == 'F') {
            char fTempChar[5];
            // номер разряда числа, куда будет записан прочитанный символ (в fTempChar)
            uint8_t i = 0;
            while (true) {                          // выясняем скорость подачи (считываем число из строки data по одной цифре)
                curChar = ncFile.read();            // читаем следующий символ
                // если прочитанный символ число:
                if (curChar >= ZERO && curChar <= NINE) {
                    fTempChar[i] = curChar;
                    i++;                            // увеличиваем разряд для следующей цифры
                    continue;                       // переходим к чтению следующего символа в .ncm файле
                } else {
                    // создаем полноценную строку добавлением символа конца строки (без этого atol() не работает)
                    fTempChar[i] = '\0';
                    fSpeed = atol(fTempChar);       // массив символов переводим в числовой тип
                    speedSetting.setSpeed();          // вычисляем параметры движения для новой скорости
                    break;                            // Выходим из цикла поиска всех цифр за буквой
                }
            }
        } else if (curChar == 'T') {
            char tTempChar[4];
            tTempChar[0] = '_';
            tTempChar[1] = '_';
            tTempChar[2] = '_';
            tTempChar[3] = '\0';
            uint8_t i = 0;
            while (true) {
                curChar = ncFile.read();            // читаем следующий символ
                // если прочитанный символ число:
                if (curChar >= ZERO && curChar <= NINE) {
                    tTempChar[i] = curChar;
                    i++;                            // увеличиваем разряд для следующей цифры
                    continue;                       // переходим к чтению следующего символа в .ncm файле
                } else {
                    // выводим в порт номер инструмента, какой сейчас нужен
                    Serial.print("need to install tool number: ");
                    Serial.println(tTempChar);
                    // проверяем, что шаги не пропускались:
                    aMove.checkPosition();
                    // Номер инструмента полностью записан.
                    // Переходим к процедуре физической замены инструмента.
                    // Для этого сначала нужно поднять шпиндель в самый верх и
                    // отодвинуть стол, чтобы была возможность достать прежнюю фрезу
                    // и вставить новую.
                    aMove.toRiseSpindle();          // поднимаем шпиндель
                    aMove.moveAlongTable();         // отодвигаем стол
                    // Теперь можно останавливать шпиндель и менять фрезу.
                    // А пока меняется фреза, ожидаем нажатия кнопок:
                    //   2. все кнопки ручного перемещения по всем осям
                    //   3. кнопка начать движение к точке смены инструмента
                    //   4. перейти к референтной точке
                    while (true) {
                        // 2. проверяем нажата ли одна из кнопок ручного перемещения
                        mControl.isOnManual();
                        // 4. продолжаем программу после физической замены инструмента
                        if (digitalRead(pinAutoSetTool)) {
                            // продолжаем попытки установить инструмент, пока код выполнения больше нуля.
                            // когда changeP.continueProgram() вернет код выполнения 0, продолжаем программу
                            while (changeP.continueProgram()) {
                                // поднимаем шпиндель в самый верх для установки подходящего инструмента
                                aMove.toRiseSpindle();
                                while (!digitalRead(pinAutoSetTool)) {
                                    // ждем нажатия кнопки Continue prograм после замены инструмента
                                    // неподходящего по длине на подходящий по длине
                                    mControl.isOnManual();  // слушаем нажатие кнопок ручного управления
                                }
                            }
                            break;
                        }
                    }
                    break;                            // Выходим из цикла поиска всех цифр за буквой
                }
            }
        } else {
            curChar = ncFile.read();            // читаем следующий символ
        }
    }
}

void defineDirection() {            // функция определения направления движения всех координат
  if (xStepsFrame != 0) {           // есть движение по данной оси
    if (xStepsFrame > 0) {          // значение координаты выше нуля
      PORTE |= 1 << PORTE3;         // pinDirX1, right (HIGH)
      PORTH |= 1 << PORTH3;         // pinDirX2
      // устанавливаем флаг направления движения в данном кадре
      // (нужно для системы слежения за позицией в абсолютной системе координат)
      xDir = rightFlag;
    } else {                        // значение координаты ниже нуля
      PORTE &= ~(1 << PORTE3);      // pinDirX1, left (LOW)
      PORTH &= ~(1 << PORTH3);      // pinDirX2
      xDir = leftFlag;
    }
  }
  if (yStepsFrame != 0) {           // есть движение по данной оси
    if (yStepsFrame > 0) {          // значение координаты выше нуля
      PORTH |= 1 << PORTH4;         // pinDirY1, forward (HIGH)
      PORTB |= 1 << PORTB7;         // pinDirY2
      yDir = forwardFlag;           // устанавливаем флаг направления движения в данном кадре (нужно для системы слежения за позицией в абсолютной системе координат)
    } else {                        // значение координаты ниже нуля
      PORTH &= ~(1 << PORTH4);      // pinDirY1
      PORTB &= ~(1 << PORTB7);
      yDir = backFlag;
    }
  }
  if (zStepsFrame != 0) {           // есть движение по данной оси
    if (zStepsFrame > 0) {          // значение координаты выше нуля
      PORTB |= 1 << PORTB5;         // pinDirZ, top (HIGH)
      zDir = topFlag;               // устанавливаем флаг направления движения в данном кадре (нужно для системы слежения за позицией в абсолютной системе координат)
    } else {                        // значение координаты ниже нуля
      PORTB &= ~(1 << PORTB5);      // pinDirZ
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

// void report() {
//   // считаем запланированное количество шагов, сколько должно было бы быть выполнено от начала программы до текущего момента:
//   xPlaneToTime = xPlaneToTime + xStepsFrameABS;
//   yPlaneToTime = yPlaneToTime + yStepsFrameABS;
//   zPlaneToTime = zPlaneToTime + zStepsFrameABS;
//
//   // массивы символов для вывода в консоль типа 00000328
//   char fXStepsFrameABS[9];  // запланированные шаги в пределах одного кадра
//   char fYStepsFrameABS[9];
//   char fZStepsFrameABS[9];
//   char fXStepsDone[9];    // выполненные шаги в пределах одного кадра
//   char fYStepsDone[9];
//   char fZStepsDone[9];
//   char fXPlaneToTime[9];   // запланированные шаги от начала программы до данного момента
//   char fYPlaneToTime[9];
//   char fZPlaneToTime[9];
//   char fXDoneToTime[9];  // выполненные шаги от начала программы до данного момента
//   char fYDoneToTime[9];
//   char fZDoneToTime[9];
//
//   intToChar(xStepsFrameABS);            // преобразуем запланированные шаги в пределах одного кадра из числа в строку типа 00000328 и записываем в массивы выше
//   for (int i = 0; i < 9; i++) {fXStepsFrameABS[i] = tempNumChar[i];}  // копируем временный массив tempNumChar[] в нужных массив
//   intToChar(yStepsFrameABS);
//   for (int i = 0; i < 9; i++) {fYStepsFrameABS[i] = tempNumChar[i];}
//   intToChar(zStepsFrameABS);
//   for (int i = 0; i < 9; i++) {fZStepsFrameABS[i] = tempNumChar[i];}
//
//   intToChar(xStepsDone);              // преобразуем выполненные шаги в пределах одного кадра из числа в строку типа 00000328 и записываем в нужный массив
//   for (int i = 0; i < 9; i++) {fXStepsDone[i] = tempNumChar[i];}
//   intToChar(yStepsDone);
//   for (int i = 0; i < 9; i++) {fYStepsDone[i] = tempNumChar[i];}
//   intToChar(zStepsDone);
//   for (int i = 0; i < 9; i++) {fZStepsDone[i] = tempNumChar[i];}
//
//   intToChar(xPlaneToTime);  // преобразуем запланированные шаги от начала программы до настоящего момента из числа в строку типа 00000328 и записываем в нужный массив
//   for (int i = 0; i < 9; i++) {fXPlaneToTime[i] = tempNumChar[i];}
//   intToChar(yPlaneToTime);
//   for (int i = 0; i < 9; i++) {fYPlaneToTime[i] = tempNumChar[i];}
//   intToChar(zPlaneToTime);
//   for (int i = 0; i < 9; i++) {fZPlaneToTime[i] = tempNumChar[i];}
//
//   intToChar(xDoneToTime);  // преобразуем выполненные шаги от начала программы до настоящего момента из числа в строку типа 00000328 и записываем в нужный массив
//   for (int i = 0; i < 9; i++) {fXDoneToTime[i] = tempNumChar[i];}
//   intToChar(yDoneToTime);
//   for (int i = 0; i < 9; i++) {fYDoneToTime[i] = tempNumChar[i];}
//   intToChar(zDoneToTime);
//   for (int i = 0; i < 9; i++) {fZDoneToTime[i] = tempNumChar[i];}
//
//   Serial.println("Farame steps planned : X" + String(fXStepsFrameABS) + " Y" + String(fYStepsFrameABS) + " Z" + String(fZStepsFrameABS));
//   Serial.println("Farame steps done    : X" + String(fXStepsDone) + " Y" + String(fYStepsDone) + " Z" + String(fZStepsDone));
//   Serial.println("To time steps planned: X" + String(fXPlaneToTime) + " Y" + String(fYPlaneToTime) + " Z" + String(fZPlaneToTime));
//   Serial.println("To time steps done   : X" + String(fXDoneToTime) + " Y" + String(fYDoneToTime) + " Z" + String(fZDoneToTime));
//   Serial.println();
// }

void startProgram()
{
    // перед началом программы проверяем ряд условий:
    // 1. выключен ли тумблер поиска точки G54
    if (digitalRead(startSearchG54Rectangle))
    {
        Serial.println("you need to turn off the search toggle switch for point G54 (startSearchG54Rectangle)");
    }
    // все условия для начала выполнения программы соблюдены
    else
    {
        // идем в точку G54 (если мы еще не там)
        refPoint.goToRPoint();
        // опускаем шпиндель на глубину проставки, потому что refPoint.goToRPoint();
        // приводит шпиндель к G54 + высота проставки
        aMove.lowerASpacerHeight();
        synFrameProc sfp;             // имплементим класс распределения шагов// uint32_t oldZPosition = machinePosition.getPositionZ();
        openNcmFile();                // открываем sd карту для чтения
        while (true)
        {
          read_line_sd();             // читаем строку из файла на флешке и парсим её
          defineDirection();          // устанавливаем направление движения по всем осям
          sfp.sfpFrameProcessing();   // отрабатываем шаги
          // report();                   // мониторим результаты работы программы на экране компа
          xStepsDone = 0;             // обнуляем количество выполненных шагов в текущем кадре
          yStepsDone = 0;
          zStepsDone = 0;
        }
    }
}

// структура с массивом координат точки G54
struct G54
{
    uint32_t x;                         // координата центра заготовки по оси X
    uint32_t y;                         // координата центра заготовки по оси Y
    uint32_t z;                         // координата центра заготовки по оси Z
};

// стркутура с координатами краев заготовки
struct WorkpieceEdges
{
    uint32_t upperSide;             // координата верха заготовки (по оси Z)
    uint32_t leftSide;              // координата левой стенки заготовки (по оси X)
    uint32_t rightSide;             // координата правой стенки заготовки (по оси X)
    uint32_t backSide;              // координата задней стенки заготовки (ось Y)
    uint32_t frontSide;             // координата передней стенки заготовки (ось Y)
};

// структура с количеством шагов в одном миллиметре по осям X, Y и отдельно Z
struct StepsInMm
{
    const uint16_t xy = 400;       // шагов в 1мм по осям X и Y
    const uint16_t z = 800;        // при 1/8 шага в 1мм движения по оси Z 800 импульсов
};

// структура - расстояние отвода датчика от стенки заготовки после касания датчика (в шагах)
struct RetractionSteps
{
    uint16_t xy;
    uint16_t z;
};


// класс описывает поиск нулевой точки заготовки
class G54Finder
{
private:
    // скорость движения шпинделя в момент поиска G54 (в миллиметрах в секунду)
    const uint16_t searchSpeed;
    // скорость отвода шпинделя после касания датчика (в миллиметрах в секунду)
    const uint16_t retractSpeed;
    // длительность паузы после срабатывания датчика касания (в миллисекундах)
    const uint16_t pauseDuration;
    // структура с количеством шагов в одном миллиметре по осям X, Y и отдельно Z
    StepsInMm stepsInMm;
    // расстояние отвода датчика от стенки после касания в миллиметрах
    const uint8_t distRetraction;
    // расстояние отвода датчика от стенки после касания в шагах двигателя
    RetractionSteps retractionSteps =
    {stepsInMm.xy * distRetraction, stepsInMm.z  * distRetraction};

    // определение структуры с координатами краев заготовки
    WorkpieceEdges workpieceEdges = {0, 0, 0, 0, 0};
    // определяем структуру с координатами точки G54
    G54 g54 = {0, 0, 0};
    // радиус шарика щупа датчика в миллиметрах
    const float mmBallRadius = 0.985;
    // радиус шарика щупа датчика в шагах
    uint16_t stepsBallRadiusXY = mmBallRadius * stepsInMm.xy;
    uint16_t stepsBallRadiusZ  = mmBallRadius * stepsInMm.z;
    // погрешность датчика в шагах (расстояние срабатывания)
    // найдено с параллелькой (плитка 5мм)
    const uint16_t sensorErrorX = 30;
    const uint16_t sensorErrorY = 26;
    const uint16_t sensorErrorZ = 50;

public:
    // конструктор класса
    G54Finder() :
    searchSpeed(50), retractSpeed(180), pauseDuration(500), distRetraction(1)
    {}
    void searchG54Start()
    {
        // исключаем дребезг контактов
        if (contactDebouncing())
        {
            // находимся в режиме поска G54 пока включен тумблер startSearchG54Rectangle
            while (digitalRead(startSearchG54Rectangle))
            {
                // cлушаем нажатия кнопок перемещения по осям:
                if (digitalRead(pinToLeft))
                {
                    // двигаем шпиндель влево и одновременно слушаем датчик касания
                    searchRightSide();
                }
                else if (digitalRead(pinToRight))
                {
                    // двигаем шпиндель вправо и слушаем датчик касания
                    searchLeftSide();
                }
                else if (digitalRead(pinToForward))
                {
                    // двигаем шпиндель вперед (к оператору) и слушаем датчик касания
                    searchBackSide();
                }
                else if (digitalRead(pinToBack))
                {
                    // двигаем шпиндель назад (от оператора) и слушаем датчик касания
                    searchFrontSide();
                }
                else if (digitalRead(pinToBottom))
                {
                    // двигаем шпиндель вниз и слушаем датчик касания
                    searchUpperSide();
                }
                else if (digitalRead(pinToTop))
                {
                    // просто двигаем шпиндель вверх
                    simpleLiftUpSpindle();
                }
                // нажата кнопка "установить точку G54"
                else if (digitalRead(pinSetG54))
                {
                    // проверяем, не забыли ли мы обозначить верхнюю грань заготовки
                    if (workpieceEdges.upperSide != 0)
                    {
                        // верхняя грань точки G54 всегда одинаковая, поэтому мы её присваиваем
                        // прямо здесь и больлше к этому возвращаться не будем
                        rPointG54Z = workpieceEdges.upperSide;
                        // дальше, анализируем, какие стороны заготовки были найдены и на основании этих
                        // точек решаем, в каком месте будет точки G54
                        setG54();
                    }
                }
                // нажата кнопка "перейти к точке G54"
                else if (digitalRead(pinGoToG54))
                {
                    refPoint.goToRPoint();
                }
            }
        }
    }

    void setG54()
    {
        // G54 - углы:
        // левый верхний
        if ( workpieceEdges.leftSide  &&  workpieceEdges.backSide &&
            !workpieceEdges.rightSide && !workpieceEdges.frontSide)
        {
            rPointG54X = workpieceEdges.leftSide;
            rPointG54Y = workpieceEdges.backSide;
        }
        // правый верхний
        else if (!workpieceEdges.leftSide  &&  workpieceEdges.backSide &&
                  workpieceEdges.rightSide && !workpieceEdges.frontSide)
        {
            rPointG54X = workpieceEdges.rightSide;
            rPointG54Y = workpieceEdges.backSide;
        }
        // правый нижний
        else if (!workpieceEdges.leftSide  && !workpieceEdges.backSide &&
                  workpieceEdges.rightSide &&  workpieceEdges.frontSide)
        {
            rPointG54X = workpieceEdges.rightSide;
            rPointG54Y = workpieceEdges.frontSide;
        }
        // левый нижний
        else if ( workpieceEdges.leftSide  && !workpieceEdges.backSide &&
                 !workpieceEdges.rightSide &&  workpieceEdges.frontSide)
        {
            rPointG54X = workpieceEdges.leftSide;
            rPointG54Y = workpieceEdges.frontSide;
        }
        // G54 в центре рёбер:
        // в центре левого ребра
        else if ( workpieceEdges.leftSide  &&  workpieceEdges.backSide &&
                 !workpieceEdges.rightSide &&  workpieceEdges.frontSide)
        {
            rPointG54X = workpieceEdges.leftSide;
            rPointG54Y = workpieceEdges.frontSide +
                         ((workpieceEdges.backSide - workpieceEdges.frontSide) / 2);
        }
        // в центре заднего ребра
        else if ( workpieceEdges.leftSide  &&  workpieceEdges.backSide &&
                  workpieceEdges.rightSide && !workpieceEdges.frontSide)
        {
            rPointG54X = workpieceEdges.leftSide +
                         ((workpieceEdges.rightSide - workpieceEdges.leftSide) / 2);
            rPointG54Y = workpieceEdges.backSide;
        }
        // в центре правого ребра
        else if (!workpieceEdges.leftSide  &&  workpieceEdges.backSide &&
                  workpieceEdges.rightSide &&  workpieceEdges.frontSide)
        {
            rPointG54X = workpieceEdges.rightSide;
            rPointG54Y = workpieceEdges.frontSide +
                         ((workpieceEdges.backSide - workpieceEdges.frontSide) / 2);
        }
        // в центре переднего ребра
        else if ( workpieceEdges.leftSide  && !workpieceEdges.backSide &&
                  workpieceEdges.rightSide &&  workpieceEdges.frontSide)
        {
            rPointG54X = workpieceEdges.leftSide +
                         ((workpieceEdges.rightSide - workpieceEdges.leftSide) / 2);
            rPointG54Y = workpieceEdges.frontSide;
        }
        // в центре заготовки
        else if ( workpieceEdges.leftSide  &&  workpieceEdges.backSide &&
                  workpieceEdges.rightSide &&  workpieceEdges.frontSide)
        {
            rPointG54X = workpieceEdges.leftSide +
                         ((workpieceEdges.rightSide - workpieceEdges.leftSide) / 2);
            rPointG54Y = workpieceEdges.frontSide +
                         ((workpieceEdges.backSide - workpieceEdges.frontSide) / 2);
        }
        Serial.println("G54 point installed successfully!");
        // пауза, чтобы в порт не летело бесконечное количество сообщений
        delay(1000);
        // обнуляем все найденные стороны, чтобы можно было повторно начать поиск сторон
        workpieceEdges.leftSide  = 0;
        workpieceEdges.rightSide = 0;
        workpieceEdges.upperSide = 0;
        workpieceEdges.backSide  = 0;
        workpieceEdges.frontSide = 0;
    }

    // метод борется с дребезгом контактов при нажатии кнопки
    bool contactDebouncing()
    {
        uint16_t duration = 1000;                           // продолжительность паузы в миллисекундах
        // время начала профилактической паузы подавления дребезга контактов
        unsigned long startDebouncing = millis();
        // время конца профилактической паузы подавления дребезга контактов
        unsigned long finishDebouncing = startDebouncing + duration;
        // выдерживаем необходимую паузу
        while (millis() < finishDebouncing) {}
        // проверяем, если тумблер все еще нажат, значит его нажал оператор и это не дребезг контактов
        if (digitalRead(startSearchG54Rectangle))
        {
            return true;    // включится режим поска точки G54
        }
        else
        {
            return false;   // выполнение программы вернется обратно в Main()
        }
    }

    // метод отодвигает шпиндель от стенки заготовки после касания датчиком
    void toRetract(const char& dirRetract)
    {
        // настраиваем двигатели для движения
        aMove.setMoveParam(AT_FEED, retractSpeed, dirRetract);

        // отвод шпинделя запрошен по оси X
        if (dirRetract == A_RIGHT || dirRetract == A_LEFT)
        {
            // двигаем шпиндель по оси X в направлении dirRetract на расстояние retractionSteps
            for (uint16_t i = 0; i < retractionSteps.xy; i++)
            {
                aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
            }
        }
        // отвод шпинделя запрошен по оси Y
        else if (dirRetract == A_FORWARD || dirRetract == A_BACK)
        {
            // двигаем шпиндель по оси Y в направлении dirRetract на расстояние retractionSteps
            for (uint16_t i = 0; i < retractionSteps.xy; i++)
            {
                aMove.moveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
            }
        }
        // отвод шпинделя запрошен по оси Z
        else if (dirRetract == A_UP)
        {
            // двигаем шпиндель по оси Z в направлении dirRetract (по сути, отвод только вверх)
            // на расстояние retractionSteps
            for (uint16_t i = 0; i < retractionSteps.z; i++)
            {
                aMove.moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));
            }
        }
        // ждем немного, пока оператор будет отпускать кнопку движения,
        // чтобы шпиндель снова не поехал в заготовку
        delay(pauseDuration);
    }

    // метод ищет правую сторону заготовки
    void searchRightSide()
    {
        // настраиваем двигатели для движения влево
        aMove.setMoveParam(AT_FEED, searchSpeed, A_LEFT);
        // пока нажата кнопка "влево"
        while (digitalRead(pinToLeft))
        {
            // проверяем выход шпинделя за пределы рабочей зоны станка
            if (machinePosition.getPositionX() <= 0)
            {
                Serial.println("The spindle extends beyond the working area to the left");
                break;
            }
            // делаем шаг влево
            aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
            // слушаем датчик касания
            if (!digitalRead(pinTouchProbe))
            {
                // датчик касания нашел правую стенку
                // записываем текущую позищию по X как координату правой стенки заготовки
                workpieceEdges.rightSide = machinePosition.getPositionX() - stepsBallRadiusXY + sensorErrorX;
                // отодвигаем шпиндель немного вправо отстенки
                toRetract(A_RIGHT);
                Serial.println("Right side of the workpiece found.");
                break;  // заканчиваем слушать нажатую кнопку "влево"
            }
        }
    }

    // метод ищет левую сторону заготовки
    void searchLeftSide()
    {
        // настраиваем двигатели для движения вправо
        aMove.setMoveParam(AT_FEED, searchSpeed, A_RIGHT);
        // пока нажата кнопка "вправо"
        while (digitalRead(pinToRight))
        {
            // проверяем выход шпинделя за пределы рабочей зоны станка
            if (machinePosition.getPositionX() >= widthXAxis)
            {
                Serial.println("The spindle extends beyond the working area to the right");
                break;
            }
            // делаем шаг вправо
            aMove.moveX(speedSetting.durHighLevel, speedSetting.getSpeed('x'));
            // слушаем датчик касания
            if (!digitalRead(pinTouchProbe))
            {
                // датчик касания нашел левую стенку
                // записываем текущую позищию по X как координату левой стенки заготовки
                workpieceEdges.leftSide = machinePosition.getPositionX() + stepsBallRadiusXY - sensorErrorX;
                // отодвигаем шпиндель немного левее от стенки
                toRetract(A_LEFT);
                Serial.println("Left side of the workpiece found.");
                break;  // заканчиваем слушать нажатую кнопку "вправо"
            }
        }
    }

    // метод ищет заднюю сторону заготовки (дальнюю от оператора)
    void searchBackSide()
    {
        // настраиваем двигатели для движения назад (от оператора)
        aMove.setMoveParam(AT_FEED, searchSpeed, A_BACK);
        // пока нажата кнопка "назад"
        while (digitalRead(pinToForward))
        {
            // проверяем выход шпинделя за пределы рабочей зоны станка
            if (machinePosition.getPositionY() <= 0)
            {
                Serial.println("The spindle extends beyond the working area to the forward");
                break;
            }
            // делаем шаг назад
            aMove.moveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
            // слушаем датчик касания
            if (!digitalRead(pinTouchProbe))
            {
                // датчик касания нашел переднюю стенку
                // записываем текущую позищию по Y как координату передней стенки заготовки
                workpieceEdges.backSide = machinePosition.getPositionY() - stepsBallRadiusXY + sensorErrorY;
                // немного отодвигаем шпиндель от стенки
                toRetract(A_FORWARD);
                Serial.println("Back side of the workpiece found.");
                break;  // заканчиваем слушать нажатую кнопку "назад"
            }
        }
    }

    // метод ищет переднюю сторону заготовки
    void searchFrontSide()
    {
        // настраиваем двигатели для движения вперед (к оператору)
        aMove.setMoveParam(AT_FEED, searchSpeed, A_FORWARD);
        // пока нажата кнопка "вперед"
        while (digitalRead(pinToBack))
        {
            // проверяем выход шпинделя за пределы рабочей зоны станка
            if (machinePosition.getPositionY() >= lengthYAxis)
            {
                Serial.println("The spindle extends beyond the working area to the back");
                break;
            }
            // делаем шаг вперед
            aMove.moveY(speedSetting.durHighLevel, speedSetting.getSpeed('y'));
            // слушаем датчик касания
            if (!digitalRead(pinTouchProbe))
            {
                // датчик касания нашел заднюю стенку
                // записываем текущую позищию по Y как координату задней стенки заготовки
                workpieceEdges.frontSide = machinePosition.getPositionY() + stepsBallRadiusXY - sensorErrorY;
                // немного отодвигаем шпиндель от стенки
                toRetract(A_BACK);
                Serial.println("Front side of the workpiece found.");
                break;  // заканчиваем слушать нажатую кнопку "вперед"
            }
        }
    }

    // метод ищет верхнюю сторону заготовки
    void searchUpperSide()
    {
        // настраиваем двигатели для движения вниз
        aMove.setMoveParam(AT_FEED, searchSpeed, A_DOWN);
        // пока нажата кнопка "вниз"
        while (digitalRead(pinToBottom))
        {
            // проверяем выход шпинделя за пределы рабочей зоны станка
            if (machinePosition.getPositionZ() <= 0)
            {
                Serial.println("The spindle extends beyond the working area to the down");
                break;
            }
            // делаем шаг вниз
            aMove.moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));
            // слушаем датчик касания
            if (!digitalRead(pinTouchProbe))
            {
                // датчик касания нашел верхнюю стенку заготовки
                // записываем текущую позищию по Z как координату верхней стенки заготовки
                workpieceEdges.upperSide = machinePosition.getPositionZ() + sensorErrorZ;
                // немного приподнимаем шпиндель над заготовкой
                toRetract(A_UP);
                Serial.println("Top side of the workpiece found.");
                break;  // заканчиваем слушать нажатую кнопку "вниз"
            }
        }
    }

    // метод просто поднимает шпиндель вверх (находясь в режиме поиска G54 - то есть
    // пока включен тумблер этого режима)
    // этот метод нужен, потому что в режиме поиска G54 обычные передвижения
    // шпиндели недоступны
    void simpleLiftUpSpindle()
    {
        // настраиваем двигатели для движения вверх
        aMove.setMoveParam(AT_FEED, searchSpeed, A_UP);
        // пока нажата кнопка "вверх"
        while (digitalRead(pinToTop))
        {
            // проверяем выход шпинделя за пределы рабочей зоны станка
            if (machinePosition.getPositionZ() >= zDistance)
            {
                Serial.println("The spindle extends beyond the working area to the up");
                break;
            }
            // делаем шаг вверх
            aMove.moveZ(speedSetting.durHighLevel, speedSetting.getSpeed('z'));
        }
    }
};

G54Finder g54f;             // имплементим класс полуавтоматического поиска точки G54

void loop() {
  delay(1500);                  // даем возможность запуститься блокам питания перед считыванием состояния пульта упавления
  Serial.println("The machine is started.");
  digitalWrite(pinEn, LOW);     // включаем двигатели
  Serial.println("Engines are running.");
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
    } else if (digitalRead(pinSetToolSensor)) {
      changeP.setToolSensorPoint();         // начинаем процесс инициализации датчика касания инструмента
    } else if (digitalRead(pinGoChangePoint)) {
      aMove.moveXToolChange(changeP.changePointX);    // переходим к точке смены инструмента по X
    } else if (digitalRead(startSearchG54Rectangle)) {
      g54f.searchG54Start();                // начинаем полуавтоматический поиск G54 прямоугольной заготовки
    }
  }
}
