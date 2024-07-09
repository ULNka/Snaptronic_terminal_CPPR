/*
    Библиотека для удобного управления коллекторными моторами через драйвер
    Документация: https://alexgyver.ru/gyvermotor/
    GitHub: https://github.com/GyverLibs/GyverMotor
    Возможности:
    - Контроль скорости и направления вращения
    - Работа с ШИМ любого разрешения
    - Плавный пуск и изменение скорости
    - Активный тормоз
    - Порог минимального ШИМ
    - Deadtime
    - Поддержка 5 типов драйверов

    AlexGyver, alex@alexgyver.ru
    https://alexgyver.ru/
    MIT License

    Версии:
    v1.1 - убраны дефайны
    v1.2 - возвращены дефайны
    v2.0:
        - Программный deadtime
        - Отрицательные скорости
        - Поддержка двух типов драйверов и реле
        - Плавный пуск и изменение скорости
    v2.1: небольшие фиксы и добавления
    v2.2: оптимизация
    v2.3: добавлена поддержка esp (исправлены ошибки)
    v2.4: совместимость с другими библами
    v2.5: добавлен тип DRIVER2WIRE_NO_INVERT
    v3.0: переделана логика minDuty, добавлен режим для ШИМ любой битности
    v3.1: мелкие исправления
    v3.2: улучшена стабильность плавного режима
    v3.2.1: вернул run() в public
    v4.0: исправлен баг в GyverMotor. Добавлен GyverMotor2
*/

#ifndef _GyverMotor_h
#define _GyverMotor_h
#include "main.h"
#include <stdlib.h>
#include <stdint.h>

#define _SMOOTH_PRD 50  // таймер smoothTick, мс
typedef enum GM_workMode {
    FORWARD,
    BACKWARD,
    STOP,
    BRAKE,
    AUTO = 0,
}GM_workMode;

    int16_t _dutyS = 0;
    int16_t _minDuty = 0, _state = 0;


    bool _direction = false;
    int8_t _level = 0;  // логика инвертирована!
    int _maxDuty = 500;
    GM_workMode _mode = FORWARD, _lastMode = FORWARD;
    uint16_t _deadtime = 0;
    uint8_t _speed = 15;
    uint32_t _tmr = 0;
    float _k;
    int16_t _duty = 0;

//enum GM_driverType {
//    DRIVER2WIRE_NO_INVERT,  // двухпроводной драйвер, в котором при смене направления не нужна инверсия ШИМ
//    DRIVER2WIRE,            // двухпроводной драйвер (направление + ШИМ)
//    DRIVER3WIRE,            // трёхпроводной драйвер (два пина направления + ШИМ)
//    RELAY2WIRE,             // реле в качестве драйвера (два пина направления)
//};

#define NORMAL 0
#define REVERSE 1

    // установка скорости -255..255 (8 бит) и -1023..1023 (10 бит)
    void setSpeed(uint16_t duty);

    // сменить режим работы мотора:
    // FORWARD - вперёд
    // BACKWARD - назад
    // STOP - остановить
    // BRAKE - активный тормоз
    // AUTO - подчиняется setSpeed (-255.. 255)
    void setMotorMode(GM_workMode mode);

    // направление вращения
    // NORM - обычное
    // REVERSE - обратное
    void setMotorDirection(bool direction);

    // дать прямую команду мотору (без смены режима)
    void run(GM_workMode mode, int16_t duty);

    // установить минимальную скважность (при которой мотор начинает крутиться)
    void setMinDuty(int duty);

    // установить deadtime (в микросекундах). По умолч 0
    void setDeadtime(uint16_t deadtime);

    // установить уровень драйвера (по умолч. HIGH)
    void setLevel(int8_t level);

    // плавное изменение к указанной скорости (к значению ШИМ)
    void smoothTick(int16_t duty);

    // скорость изменения скорости
    void setSmoothSpeed(uint8_t speed);

    // возвращает -1 при вращении BACKWARD, 1 при FORWARD и 0 при остановке и торможении
    uint8_t getState();

    void setPins(uint8_t a, uint16_t c);

    int16_t constrain (int16_t x, int16_t a, int16_t b);

    void run(GM_workMode mode, int16_t duty) {
        // дедтайм
        if (_deadtime > 0 && _lastMode != mode) {
            _lastMode = mode;
            setPins(_level, 0);  // выключить всё
    //        delayMicroseconds(_deadtime);
        }

        if (_direction) {
             if (mode == FORWARD) mode = BACKWARD;
             else if (mode == BACKWARD) mode = FORWARD;
         }



        switch (mode) {
            case FORWARD:
                setPins(_level, duty);
                _state = 1;
                break;
            case BACKWARD:
                setPins(!_level, (_maxDuty - duty));
                _state = -1;
                break;
            case BRAKE:
                setPins(!_level, !_level * 255);
                _state = 0;
                break;  // при 0/255 analogWrite сделает 0/1
            case STOP:
                setPins(_level, _level * 255);
                _duty = _dutyS = 0;
                _state = 0;
                break;
        }
    }
    void setSpeed(uint16_t duty) {
        if (_mode < 2) {  // FORWARD/BACKWARD/AUTO
            _duty = constrain(duty, -_maxDuty, _maxDuty);

            // фикс стандартного analogWrite(пин, 500) для >8 бит
            if (_maxDuty > 500 && abs(_duty) == 500) _duty++;

            if (duty == 0) run(STOP, 0);
            else {
                if (duty > 0) {
                    if (_minDuty != 0) _duty = _duty * _k + _minDuty;  // сжимаем диапазон
                    run(_mode, _duty);
                } else {
                    if (_minDuty != 0) _duty = _duty * _k - _minDuty;  // сжимаем диапазон
                    run(BACKWARD, -_duty);
                }
            }
        }
    }

    void setPins(uint8_t a, uint16_t c) {
    	if(a){
    HAL_GPIO_WritePin(QOUT1_GPIO_Port, QOUT1_Pin, GPIO_PIN_SET);
    	} else HAL_GPIO_WritePin(QOUT1_GPIO_Port, QOUT1_Pin, GPIO_PIN_RESET);
      TIM4->CCR2 = (uint32_t)c;
    }

    void smoothTick(int16_t duty) {
        if (HAL_GetTick() - _tmr >= _SMOOTH_PRD) {
            _tmr = HAL_GetTick();
            if (abs(_dutyS - duty) > _speed) _dutyS += (_dutyS < duty) ? _speed : -_speed;
            else _dutyS = duty;
            setSpeed(_dutyS);
        }
    }

    uint8_t getState() {
        return _state;
    }

    void setMinDuty(int duty) {
        _minDuty = duty;
        _k = 1.0 - (float)_minDuty / _maxDuty;
    }

    void setMode(GM_workMode mode) {
        if (_mode == mode) return;
        _mode = mode;
        run(mode, _duty);
    }

    void setSmoothSpeed(uint8_t speed) {
        _speed = speed;
    }

    void setMotorDirection(bool direction) {
        _direction = direction;
    }

    void setDeadtime(uint16_t deadtime) {
        _deadtime = deadtime;
    }

    void setLevel(int8_t level) {
        _level = !level;
    }


    int16_t constrain (int16_t x, int16_t a, int16_t b){
    	if(x<=a) return a;
    	if(x>=b) return b;
    	return x;
    }
	void openDoor(uint16_t slowTime, uint32_t startTime, uint16_t speed) {

		setMotorDirection(NORMAL);
		if (HAL_GetTick() - startTime >= 12000) {
			_speed = 80;
			smoothTick(0);
//			_state=0;
		} else if (HAL_GetTick() - startTime >= slowTime) {
			_speed = 10;
			smoothTick(220);
		} else {
			_speed = 50;
			smoothTick(speed);
		}
	}

	void closeDoor(uint16_t slowTime, uint32_t startTime, uint16_t speed) {

		setMotorDirection(REVERSE);
		if (HAL_GetTick() - startTime >= 12000) {
			_speed = 50;
			smoothTick(0);
//			_state=0;
		} else if (HAL_GetTick() - startTime >= slowTime) {
			_speed = 10;
			smoothTick(speed);
		} else {
			_speed = 50;
			smoothTick(220);
		}
	}

	void initDoorPosition(uint16_t speed, uint16_t time, uint32_t startTime) {
	if (HAL_GetTick() - startTime >= time) {
		setSpeed(0);
	} else {
		_speed = 30;
		smoothTick(speed);
	}
}

	void releaseDoor(uint32_t startTime){
		if(HAL_GetTick()-startTime>=2000){
			_speed = 50;
			smoothTick(0);
		} else{
			_speed = 50;
			smoothTick(250);
		}
	}
	//Comment

#endif
