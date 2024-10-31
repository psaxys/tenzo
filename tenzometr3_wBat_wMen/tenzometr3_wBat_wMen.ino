#include "HX711.h"
#include <GyverOLED.h>
#include <EEPROM.h>

//GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
GyverOLED<SSD1306_128x64, OLED_BUFFER> oled;

#define ITEMS 5     // Общее кол во пунктов
#define SUBITEMS 2  // Общее кол во пунктов
#define FW_VERSION 3.6

#define INIT_ADDR 1023  // номер ячейки
#define INIT_UNITS 0    // ключ первого запуска

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 10;
const int LOADCELL_SCK_PIN = 9;

const byte btnOK = 4;
const byte btnMenu = 5;
const byte btnUp = 6;
const byte btnDn = 7;

uint32_t myTimer, myTimer2, myTimer2_1, menuTimer;
uint32_t eepromTimer = 0;
boolean eepromFlag = false;

float tenzo;
float my_vcc_const;  // константа вольтметра
float bat_vol;
//int disp_units = 0;

HX711 scale;

void setup() {
  pinMode(btnOK, INPUT_PULLUP);
  pinMode(btnMenu, INPUT_PULLUP);
  pinMode(btnUp, INPUT_PULLUP);
  pinMode(btnDn, INPUT_PULLUP);

  Serial.begin(38400);
  my_vcc_const = 1.1;
  oled.init();   // инициализация
  oled.clear();  // очистить дисплей (или буфер)
  oled.setScale(2);
  oled.setCursor(5, 1);
  oled.print("TENSOMETER");
  oled.setCursor(5, 3);
  oled.print(FW_VERSION);
  oled.setCursor(20, 7);
  oled.setScale(1);
  oled.print("axyal@yandex.ru");
  oled.update();

  delay(1000);
  oled.clear();
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  oled.home();
  oled.setScale(1);
  oled.setCursor(25, 3);
  oled.println("Инициализация");
  oled.update();
  oled.clear();

  scale.set_scale(149.f);  // константа калибровки
  scale.tare();            // сброс в 0
  oled.clear();
  oled.home();
}

void loop() {
  MenuButton();
  scale.power_down();  // put the ADC in sleep mode
  if (millis() - myTimer2 > 500) {
    scale.power_up();
  }
  myTimer2 = millis();
}

void MyMenu() {
  oled.clear();
  oled.home();
  oled.setScale(1);
  void (*resetFunc)(void) = 0;
  while (1) {
    static int8_t pointer = 0;  // Переменная указатель
    if (digitalRead(btnUp) == LOW) {
      pointer = constrain(pointer - 1, 0, ITEMS - 1);  // Двигаем указатель в пределах дисплея
    }

    if (digitalRead(btnDn) == LOW) {
      pointer = constrain(pointer + 1, 0, ITEMS - 1);
    }

    if (digitalRead(btnOK) == LOW) {  // Нажатие на ОК - переход в пункт меню
      switch (pointer) {              // По номеру указателей располагаем вложенные функции (можно вложенные меню)
        case 0: settings(); break;    // По нажатию на ОК при наведении на 0й пункт вызвать функцию
        case 1: units(); break;
        case 2: info(); break;
        case 3: calibration(); break;
        case 4:
          resetFunc();
          break;
          /* case 5: break;
        case 6: break;
        case 7: func(); break; */
      }
    }

    /* меню */
    //    oled.clear();  // Очищаем буфер
    oled.home();  // Курсор в левый верхний угол
    oled.print    // Вывод всех пунктов
      (F(
        "  Настройки\r\n"  // Не забываем про '\r\n' - перенос строки
        "  Единицы измерения\r\n"
        "  Общая информация\r\n"
        "  Калибровка датчика\r\n"
        "  Перезагрузка\r\n"
        /*"  Parameter 5:\r\n"
      "  Parameter 6:\r\n"
      "  Parameter 7:"*/
        ));
    printPointer(pointer);  // Вывод указателя
    oled.update();          // Выводим кадр на дисплей
    if (digitalRead(btnMenu) == LOW) {
      oled.clear();
      return;
    }
  }
}

void MySubMenu() {
  oled.clear();
  oled.home();
  oled.setScale(1);
  while (1) {
    static int8_t pointer = 0;  // Переменная указатель
                                // int ButtonPress = analogRead(A0);
    if (digitalRead(btnUp) == LOW) {
      pointer = constrain(pointer - 1, 0, SUBITEMS - 1);  // Двигаем указатель в пределах дисплея
    }

    if (digitalRead(btnDn) == LOW) {
      pointer = constrain(pointer + 1, 0, SUBITEMS - 1);
    }

    /* меню */
    //    oled.clear();  // Очищаем буфер
    oled.home();  // Курсор в левый верхний угол
    oled.print    // Вывод всех пунктов
      (F(
        "  Килограмм\r\n"  // Не забываем про '\r\n' - перенос строки
        "  Ньютон\r\n"));
    printPointer(pointer);                          // Вывод указателя
    oled.update();                                  // Выводим кадр на дисплей
    if (digitalRead(btnOK) == LOW) {                // Нажатие на ОК - переход в пункт меню
      switch (pointer) {                            // По номеру указателей располагаем вложенные функции (можно вложенные меню)
        case 0: EEPROM.write(INIT_ADDR, 0); break;  // EEPROM килограммы
        case 1: EEPROM.write(INIT_ADDR, 1); break;  // EEPROM ньютоны
      }
      //  return;
    }
    if (digitalRead(btnMenu) == LOW) {
      oled.clear();
      return;
    }
  }
}

void printPointer(uint8_t pointer) {
  // Указатель в начале строки
  oled.setCursor(0, pointer);
  oled.print(">");
}

void info(void) {
  oled.clear();
  oled.home();
  oled.println("Общая информация:");
  oled.println("Калибровочный коэф.: ");
  oled.println(scale.get_scale());
  oled.println("АЦП датчика: ");
  oled.println(scale.read());
  oled.print("Заряд АКБ: ");
  oled.print(bat_vol);
  oled.println("%");
  oled.println(" ");
  oled.print(F("Press ESC to return"));
  oled.update();
  while (1) {
    // int ButtonPress = analogRead(A0);
    if (digitalRead(btnMenu) == LOW) {
      oled.clear();
      return;
    }  // return возвращает нас в предыдущее меню
  }
}

void units(void) {
  MySubMenu();
  while (1) {
    // int ButtonPress = analogRead(A0);
    if (digitalRead(btnMenu) == LOW) {
      oled.clear();
      return;
    }  // return возвращает нас в предыдущее меню
  }
}

void settings(void) {
  oled.clear();
  oled.home();
  //oled.setScale(2);
  oled.autoPrintln(true);
  oled.println("Изменение настроек в данный момент не требуется.");
  oled.autoPrintln(false);
  oled.println(" ");
  oled.print(F("Press ESC to return"));
  oled.update();
  while (1) {
    //int ButtonPress = analogRead(A0);
    if (digitalRead(btnMenu) == LOW) {
      oled.clear();
      return;
    }  // return возвращает нас в предыдущее меню
  }
}

void calibration(void) {
  oled.clear();
  oled.home();
  //EEPROM.read(1022);
  oled.autoPrintln(true);
  oled.println(F("Параметр калибровки: "));
  oled.autoPrintln(false);
  oled.println(EEPROM.read(1022));
  oled.print(F("Press ESC to return"));
  oled.update();
  while (1) {
    //int ButtonPress = analogRead(A0);
    if (digitalRead(btnMenu) == LOW) {
      oled.clear();
      return;
    }  // return возвращает нас в предыдущее меню
  }
}

void func(void) {
  oled.clear();
  oled.home();
  oled.print(F("Press ESC to return"));
  oled.update();
  while (1) {
    // int ButtonPress = analogRead(A0);
    if (digitalRead(btnMenu) == LOW) {
      oled.clear();
      return;
    }  // return возвращает нас в предыдущее меню
  }
}

void MenuButton() {
  while (1) {
    oled.clear();
    if (millis() - myTimer > 5000) {
      int Vbat = (analogRead(A1) * readVcc()) / 1023;
      bat_vol = (Vbat / 0.091);
      if (bat_vol < 2400) {
        while (1) {
          oled.clear();
          oled.home();
          oled.setScale(2);
          oled.setCursor(25, 2);
          oled.print("батарея");
          oled.setCursor(10, 5);
          oled.print("разряжена");
          oled.update();
          return;
        }
      }
      bat_vol = constrain(bat_vol, 2400, 4000);
      bat_vol = map(bat_vol, 2400, 4000, 0, 100);
      Serial.println(bat_vol);
      myTimer = millis();
    }
    oled.home();
    oled.setCursor(110, 0);
    drawBattery(bat_vol);
    oled.home();
    if (EEPROM.read(1023) == 0) {
      oled.setScale(2);
      oled.setCursor(100, 6);
      oled.print("kg");
      oled.setScale(4);
      oled.setCursor(20, 2);
      tenzo = scale.get_units(5) / 1000;
      if (tenzo > 0) {
        oled.print(tenzo, 2);
        if (tenzo > -0.01 && tenzo < 0.01) {
          oled.setScale(1);
          oled.setCursor(55, 7);
          oled.print(">0<");
        }
        if (tenzo >= 0.1 && tenzo <= 1.4) {
          oled.setScale(1);
          oled.setCursor(50, 7);
          oled.print("LOW");
        }
        if (tenzo >= 1.4 && tenzo <= 1.6) {
          oled.setScale(1);
          oled.setCursor(50, 7);
          oled.print("NORM");
        }
        if (tenzo >= 1.6) {
          oled.setScale(1);
          oled.setCursor(50, 7);
          oled.print("HIGH");
        }
      } else {
        oled.println("0.00");
        oled.setScale(1);
        oled.setCursor(55, 7);
        oled.print(">0<");
      }
      oled.update();
    }
    if (EEPROM.read(1023) == 1) {
      oled.setScale(2);
      oled.setCursor(100, 6);
      oled.print("N");
      oled.setScale(4);
      oled.setCursor(40, 2);
      tenzo = (scale.get_units(5) / 1000) * 9.806652;
      if (tenzo > 0) {
        oled.print(tenzo, 0);
        if (tenzo > -0.01 && tenzo < 0.01) {
          oled.setScale(1);
          oled.setCursor(55, 7);
          oled.print(">0<");
        }
        if (tenzo >= 0.1 && tenzo <= 14) {
          oled.setScale(1);
          oled.setCursor(50, 7);
          oled.print("LOW");
        }
        if (tenzo >= 14 && tenzo <= 16) {
          oled.setScale(1);
          oled.setCursor(50, 7);
          oled.print("NORM");
        }
        if (tenzo >= 16) {
          oled.setScale(1);
          oled.setCursor(50, 7);
          oled.print("HIGH");
        }
      } else {
        oled.println("0");
        oled.setScale(1);
        oled.setCursor(55, 7);
        oled.print(">0<");
      }
      oled.update();
    }

    if (digitalRead(btnDn) == LOW) {
      scale.tare();
      oled.clear();
      oled.home();
      oled.setScale(4);
      oled.setCursor(10, 2);
      oled.print("> 0 <");
      oled.update();
      delay(500);
      return;
    }
    if (digitalRead(btnUp) == LOW) {
      if (tenzo > 0) {
        while (1) {
          oled.setScale(4);
          oled.setCursor(20, 2);
          oled.print(tenzo, 2);
          oled.setScale(1);
          oled.setCursor(10, 7);
          oled.print("HOLD");
          oled.update();
          if (digitalRead(btnMenu) == LOW) {
            oled.clear();
            return;
          }
        }
      }
      return;
    }
    if (digitalRead(btnMenu) == LOW) {
      MyMenu();
    }
    if (digitalRead(btnOK) == LOW) {
      oled.clear();
      return;
    }
  }

  return;
}

long readVcc() {  //функция чтения внутреннего опорного напряжения, универсальная (для всех ардуин)
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2);             // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);  // Start conversion
  while (bit_is_set(ADCSRA, ADSC))
    ;                   // measuring
  uint8_t low = ADCL;   // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH;  // unlocks both
  long result = (high << 8) | low;
  result = my_vcc_const * 1023 * 1000 / result;  // расчёт реального VCC
  return result;                                 // возвращает VCC
}

//Рисуем батарейку
void drawBattery(byte percent) {
  oled.drawByte(0b00111100);  // пипка
  oled.drawByte(0b00111100);
  oled.drawByte(0b11111111);  // стенка
  for (byte i = 0; i < 100 / 8; i++) {
    if (i < (100 - percent) / 8) oled.drawByte(0b10000001);
    else oled.drawByte(0b11111111);
  }
  oled.drawByte(0b11111111);  // попка
}