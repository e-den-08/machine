# токен доступа на gitlab: glpat-yyoscWzRn11fJJGPqrAn
# перенос на prestigio
# ответ с prestigio
# третий и исправленный ответ с престижио

import os
import sys                  # модуль необходим для sys.exit в случае если найдено обстоятельство препятствующее
                            # дальнейшей обработке файла
# инициализация:
file_name = 'head_7'

folder_source = 'source/'
encoding_val = 'cp1251'

bl_compensation_x = 3       # длина компенсации люфта в шагах
bl_compensation_y = 4
z_compensation = 45000      # все координаты по Z поднимаются на данный коэффициент. Связано это с тем, что в данной
# модификации станка направляющие оси X слегка провисают. Датчик высоты инструмента стоит в самом углу и это самая
# высокая точка. А когда фреза приходит в центр стола, после поворота детали на 90 градусов, фреза
# немного зарезает модель. В качестве временной меры пробуем
# при повороте заготовки на 90 градусов немного приподнимать все координаты Z.

mmInStep_100k = 250        # длина шага в миллиметрах (значение умножено в 100.000 раз во избежание дробей)

# для осей X и Y следующие значения:
#   в микрошаговом режиме: 1/8
#                               импульсов на оборот:        200 * 8  =      1600
#                               миллиметров в 1 обороте:    4
#                               импульсов в 1 миллиметре:   1600 / 4 =      400
#                               миллиметров в 1 импульсе:   1мм / 400имп =  0.0025
#                               значение переменной mmInStep_100k: 0.0025 * 100000 = 250
# таблица для осей X и Y:
# _____________________________________________________________________________________________________________
# | microStep | мм_на_оборот | импульсов_на_оборот | импульсов_в_1мм | мм_в_1_импульсе | mmInStep_100k |
# |-------------------------------------------------------------------------------------------------------------|
# |     1/8   |       4      |        1600 имп.    |     400 имп.    |     0.0025мм    |      250      |
#  -------------------------------------------------------------------------------------------------------------

mmInStepZ_100k = 125       # длина шага в миллиметрах по оси Z (значение умножено в 100.000 раз во избежание дробей)
                           # (дело в том, что по оси Z во первых винт однозаходный, во воторых ограничение по скорости
                           # на двигателе ДШИ 200 и соответственно другой микрошаг, чем на остальных осях)

#   микрошаг по оси Z (однозаходный винт, 2мм на оборот)
#   таблица для оси Z:
# ___________________________________________________________________________________________
# | microStep | импульсов_на_оборот  |  импульсов_в_1мм | мм_в_1_импульсе  |  mmInStepZ_100k |
# -------------------------------------------------------------------------------------------
# |     1/8   |        1600 имп.     |      800 имп.    |     0.00125мм    |       125       |
# двухзаходный винт
# -------------------------------------------------------------------------------------------
# |     1/16  |        3200 имп.     |      800 имп.    |     0.00125мм    |       125       |
# -------------------------------------------------------------------------------------------


# читаем файл управляющей программы
file = open(f'{folder_source}{file_name}.nc', 'rt', encoding=encoding_val)
gc_list = [line for line in file]

# объявление переменных
arr_prog = list()           # массив с координатами для передачи на контроллер

lastMove_dir_x = ''         # направление движения в предыущем кадре по осям
lastMove_dir_y = ''

abs_x_position_steps = 0    # текущая позиция шпинделя в шагах от начала системы координат станка
abs_y_position_steps = 0
abs_z_position_steps = 0

x_cur = 0                   # текущая координтата
y_cur = 0
z_cur = 0                   # для 000201
x_move = 0                  # количество шагов в кадре
y_move = 0
z_move = 0

realMoveX = 0               # реальная длина пройденного пути
realMoveY = 0
realMoveZ = 0

x_pars = 0                  # числовые значения шагов по осям в кадре, который в данный момент отправляется
y_pars = 0
z_pars = 0

frameNumber = 0             # номер кадра, который в данный момент обрабатывается
count_frames = 0            # счетчик количества строк, записанных в файл ncm

x_total = 0                 # количество шагов по осям за всю программу
y_total = 0
z_total = 0


# функция для определения необходимости компенсации по осям
# если необходимо, возвращает количество шагов по оси, увеличенное на размер компенсации
def backlash_compensation(axis):
    if axis == 'x':
        if lastMove_dir_x and x_move != 0:
            if is_equal_sign(axis, lastMove_dir_x, x_move):  # направление сохраняется
                return x_move  # возвращаем неизмененное количество шагов
            else:  # направление меняется
                if x_move < 0:  # увеличиваем отрицательный x_move
                    return x_move - bl_compensation_x
                else:  # увеличиваем положительный x_move
                    return x_move + bl_compensation_x

        # если в этом кадре по оси X движения нет, возвращаем нуль - он и будет отправлен на контроллер
        if x_move == 0:
            return 0
    if axis == 'y':
        if lastMove_dir_y and y_move != 0:
            if is_equal_sign(axis, lastMove_dir_y, y_move):  # направление сохраняется
                return y_move  # возвращаем неизмененное количество шагов
            else:  # направление меняется
                if y_move < 0:  # увеличиваем отрицательный y_move
                    return y_move - bl_compensation_y
                else:  # увеличиваем положительный y_move
                    return y_move + bl_compensation_y

        # если в этом кадре по оси Y движения нет, возвращаем нуль - он и будет отправлен на контроллер
        if y_move == 0:
            return 0


# проверяет, одинаковые ли знаки у двух чисел (+ и -)
def is_equal_sign(axis, dir_move, move):
    if axis == 'x':     # какую ось проверяем в этом запросе?
        if (dir_move == 'right' and move > 0) or (dir_move == 'left' and move < 0):
            return True
        else:
            return False
    if axis == 'y':     # какую ось проверяем в этом запросе?
        if (dir_move == 'up' and move > 0) or (dir_move == 'down' and move < 0):
            return True
        else:
            return False


# первое присваивание значений lastMove_dir_x и lastMove_dir_y
def init_last_move():
    global lastMove_dir_x
    global lastMove_dir_y

    if not lastMove_dir_x:                  # lastMove_dir_x еще на заполнен (еще не было движения по этой оси)
        if x_move > 0:                          # в данном кадре движение вправо
            lastMove_dir_x = 'left'                 # указываем противоположное нынешнему движение, чтобы создать натяг
            # print(f'первое движение по X: lastMove_dir_x = {lastMove_dir_x}')
        if x_move < 0:                          # в данном кадре движение влево
            lastMove_dir_x = 'right'
            # print(f'первое движение по X: lastMove_dir_x = {lastMove_dir_x}')
        # если x_move == 0 ничего не пишем => условие f not lastMove_dir_x: сработает в следующем кадре

    if not lastMove_dir_y:                  # lastMove_dir_y еще на заполнен (еще не было движения по этой оси)
        if y_move > 0:                          # в данном кадре движение вверх
            lastMove_dir_y = 'down'              # указываем противоположное нынешнему движение, чтобы создать натяг
            # print(f'первое движение по Y: lastMove_dir_y = {lastMove_dir_y}')
        if y_move < 0:                          # в данном кадре движение вниз
            lastMove_dir_y = 'up'
            # print(f'первое движение по Y: lastMove_dir_y = {lastMove_dir_y}')
        # если y_move == 0 ничего не пишем => условие f not lastMove_dir_y: сработает в следующем кадре


# регулярное присваивание значений lastMove_dir_x и lastMove_dir_y
def assign_lastmovedir():
    global lastMove_dir_x
    global lastMove_dir_y

    if x_move > 0:
        lastMove_dir_x = 'right'
    if x_move < 0:
        lastMove_dir_x = 'left'

    if y_move > 0:
        lastMove_dir_y = 'up'
    if y_move < 0:
        lastMove_dir_y = 'down'


# математическое округление
def int_r(num):
    if num > 0:
        return int(num + 0.5)
    else:
        return int(num - 0.5)


# функция переводит текстовую координату в число
def unit_to_int(unit_coordinate):
    int_is_complete = False     # флаг, определяет, что целая часть числа уже заполнена (дошли до точки)
    int_part = ''               # целая часть координаты
    fract_part = ''             # дробная часть координаты
    sign = ''                   # знак числа координаты
    # print(unit_coordinate)
    for item in unit_coordinate:
        if item == '+':         # определяем знак координаты
            sign = item
            continue
        if item == '-':
            sign = item
            continue
        if not int_is_complete:     # еще обрабатываем целую часть числа
            if item == '.':
                int_is_complete = True  # переходим к дробной части
                continue
            int_part += item    # добавляем символ в целую часть
        else:
            fract_part += item  # добавляем символ в дробную часть
    if len(int_part) == 0:      # проверка на пустую строку
        int_part = '0'
    if len(fract_part) == 0:
        fract_part = '0'
    if len(fract_part) == 1:
        fract_part += '00'
    if len(fract_part) == 2:
        fract_part += '0'
    num_int_part = int(int_part) * 100000   # создаем целую часть координаты в числовом виде
    num_fract_part = int(fract_part) * 100  # создаем дробную часть в числовом виде
    final_coordinate = num_int_part + num_fract_part
    if sign == '-':                         # если отрицательная координата
        final_coordinate *= -1
    # print(final_coordinate)
    return final_coordinate


# проверяем, не равны ли нулю одновременно обе координаты в кадре
def both_axis_zero():
    if x_move == 0 and y_move == 0 and z_move == 0:     # ищем, где обе координаты равны нулю
        return True                         # условие if увидит True и выполнит continue
    else:
        return False


# функция возвращает номер инструмента в строке. Возвращает в виде int числа
def tool_number(list_line):
    for unit in list_line:  # перебираем юниты и ищем в каком из них номер инструмента
        if unit.startswith('T'):  # данный юнит содержит номер инструмента
            return int(unit[1:])


# проверяем, есть ли внутри строки комментарии в круглых скобках и если есть убираем их
def delete_comments(line):
    inside_comment = False
    temp_line = ''
    line_list = line.split()
    for unit in line_list:
        if unit.startswith('('):
            inside_comment = True
            continue
        if unit.startswith(')'):
            inside_comment = False
            continue
        if inside_comment is False:
            temp_line += f'{unit} '
    return temp_line.rstrip()


# начинаем обработку файла управляющей программы
# перебираем массив G-кодa
for line in gc_list:
    if line.startswith('('):        # если текущая строка - это комментарий
        # исходим из того, что если строка начинается с комментария, то вся строка целиком и есть комментарий
        # и этот комментарий мы запишем в ncm файл, а затем выведем через Serial.print();
        arr_prog.append(line.rstrip())
        continue                    # переходим к следующей
    # проверяем, есть ли внутри строки комментарии в круглых скобках и если есть убираем их и дальше обрабатываем строку
    # уже с удаленными комментариями
    if '(' in line:                 # если внутри строки есть комментарий
        line = delete_comments(line)    # убираем комментарий из строки
    # в строке присутствует вспомогательный код типа M
    # ВАЖНО!!!!!!!!!!!!!!!!!!!
    # это условие всегда должно проверяться до обнаружения координат X, Y и Z, потому что при обнаружении в строке кода
    # смены инструмента, переменные, хранящие текущее положение шпинделя (x_cur, y_cur и z_cur) обнуляются. И они должны
    # обнуляться ДО считываня координат В ТЕКУЩЕМ КАДРЕ и рассчета количества шагов!
    if 'M' in line:
        list_line = line.split()            # разбиваем строку на юниты
        for unit in list_line:                  # перебираем все юниты в текущей строке и ищем, где есть M коды
            if unit.startswith('M'):                # в текущем юните нашли какой-то M код
                cur_m_code = int(unit[1:])              # цифровое представление кода записываем в переменную типа int
                if cur_m_code == 6:                     # обрабатываем код M6 (M06) смена инструмента
                    tool_num = tool_number(list_line)
                    if tool_num != 0:                   # T0 в конце программы записывать не надо
                        arr_prog.append(f'T{tool_num}')
                        # обнуляем текущие координаты, потому что после смены инструмента при нажатии кнопки
                        # "Продолжить" станок отправится в нулевую точку заготовки и движение к координатам из
                        # следующего кадра будет происходить из нулевой точки заготовки
                        x_cur = 0
                        y_cur = 0
                        z_cur = 0
                        # print("CHANGE TOOL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                if cur_m_code == 0:                     # обрабатываем код M0 "программируемый останов"
                    arr_prog.append("M0")
                    # обнуляем текущие координаты, потому что после останова при нажатии кнопки "Продолжить"
                    # станок отправится в нулевую точку заготовки и движение к координатам из следующего кадра
                    # будет происходить из нулевой точки заготовки
                    x_cur = 0
                    y_cur = 0
                    z_cur = 0
    # в строке присутствует определение скорости подачи F
    if 'F' in line:
        list_line = line.split()            # из строки делаем список строки, стостоящий из юнитов
        for unit in list_line:              # перебираем юниты в текущей строке и ищем, в каком из юнитов скорость F
            if unit.startswith('F'):            # данный юнит содержит скорость подачи
                arr_prog.append("F" + str(round(float(unit[1:]))))
    # в строке присутствует какая-то G-команда
    if 'G' in line:
        list_line = line.split()            # формируем массив строки G-кода с юнитами
        for unit in list_line:                  # перебираем все юниты в текущей строке и ищем где есть G-команда
            if unit.startswith('G'):                # в данном юните нашли какой-то G-код
                # по сути, нам совсем не нужно писать в результат абсолютно все G-команды. Поэтому определяем,
                # что за G-команда нам встретилась и если это то, что нам надо, будем вносить это в список arr_prog
                cur_g_code = int(unit[1:])
                if cur_g_code == 0:
                    arr_prog.append("G" + str(cur_g_code))
                elif cur_g_code == 1:
                    arr_prog.append("G" + str(cur_g_code))
                elif cur_g_code == 2 or cur_g_code == 3:
                    print("В файле " + file_name + ".nc присутствует круговая интерполяция G3 или G2")
                    print("дальнейшая обработка файла невозможна")
                    sys.exit(1)
    # в строчке нашли координаты по X, Y или Z
    if 'X' in line or 'Y' or 'Z' in line:
        x_move = 0                         # обнуляем переменные, содержащие количество шагов в кадре, чтобы
        y_move = 0                         # не оставалось старых значений
        z_move = 0
        # print(frameNumber)                 # Номер обрабатываемого кадра (нач. с нуля - НЕ МЕНЯТЬ!)
        list_line = line.split()            # формируем массив строки G-кода с юнитами
        for unit in list_line:              # начинаем поиск координат в строке
            # нашли X
            if unit.startswith('X'):
                x_new = unit_to_int(unit[1:])           # координату X из текста переводим в число (* 100.000)
                # print(f'x_new: {x_new}')
                x_delta = (x_new - x_cur)               # находим длину пути по X в кадре
                # print(f'x_cur: {x_cur}')
                # print(f'x_delta: {x_delta}')
                x_move = int_r(x_delta / mmInStep_100k)  # находим количество шагов в кадре
                # print(f'x_move: {x_move}')
                realMoveX = x_move * mmInStep_100k       # исходя из количества шагов выясняем реальную длину пути
                # print(f'realMoveX: {realMoveX}')
                x_cur += realMoveX                       # записываем текущие координаты по X
                # print(f'x_cur: {x_cur}')
                x_total += abs(x_move)              # считаем общее количество шагов по X
                abs_x_position_steps += x_move  # находим положение шпинделя в шагах от начала координат станка
            # нашли Y
            if unit.startswith('Y'):
                y_new = unit_to_int(unit[1:])
                # print(f'y_new: {y_new}')
                y_delta = (y_new - y_cur)
                # print(f'y_cur: {y_cur}')
                # print(f'y_delta: {y_delta}')
                y_move = int_r(y_delta / mmInStep_100k)
                # print(f'y_move: {y_move}')
                realMoveY = y_move * mmInStep_100k  # находим количество шагов в кадре
                # print(f'realMoveY: {realMoveY}')
                y_cur += realMoveY
                # print(f'y_cur: {y_cur}')
                y_total += abs(y_move)
                abs_y_position_steps += y_move  # находим положение шпинделя в шагах от начала координат станка
                # print(f'abs_y: {abs_y_position_steps}')
            # нашли Z
            if unit.startswith('Z'):
                z_new = unit_to_int(unit[1:])           # координату Z из текста переводим в число (* 100.000)
                z_new += z_compensation
                z_delta = (z_new - z_cur)               # находим длину пути по Z в кадре
                # print(f'z_cur: {z_cur}')
                # print(f'z_delta: {z_delta}')
                z_move = int_r(z_delta / mmInStepZ_100k)  # находим количество шагов в кадре
                # print(f'z_move: {z_move}')
                realMoveZ = z_move * mmInStepZ_100k       # исходя из количества шагов выясняем реальную длину пути
                # print(f'realMoveZ: {realMoveZ}')
                z_cur += realMoveZ                       # записываем текущие координаты по Z
                # print(f'z_cur: {z_cur}')
                z_total += abs(z_move)              # считаем общее количество шагов по Z
                abs_z_position_steps += z_move      # находим положение шпинделя в шагах от начала координат станка

        if both_axis_zero():    # проверяем, не равны ли нулю одновременно обе координаты в кадре
            continue            # если обе координаты равны нулю - не обрабатываем их и не отправляем на контроллер

        # впервые заполняем значения lastMove_dir_x и lastMove_dir_y
        init_last_move()

        x_move_output = backlash_compensation('x')   # значение перемещения по X с учетом компенсации люфта
        y_move_output = backlash_compensation('y')   # значение перемещения по Y с учетом компенсации люфта

        # присваиваем значения переменным lastMove_dir_x и lastMove_dir_y
        assign_lastmovedir()

        #
        x_text = 'X' + str(x_move_output)
        y_text = 'Y' + str(y_move_output)
        z_text = 'Z' + str(z_move)

        # начинаем формировать строку для отправки на контроллер:
        line_output = ''
        if x_move_output != 0:
            line_output += x_text
        if y_move_output != 0:
            line_output += y_text
        if z_move != 0:
            line_output += z_text

        arr_prog.append(line_output)    # присоединяем строку к массиву управляющей программы


        # print("исходник:         " + line.strip())
        # print("шаги в кадре:     " + "X" + str(x_move_output) + " Y" + str(y_move_output) + " Z" + str(z_move))
        # print("позиция в шагах:  " + "absX: " + str(abs_x_position_steps) +
        #       "; absY: " + str(abs_y_position_steps) +
        #       "; absZ: " + str(abs_z_position_steps))
        # print("позиция в мм:     " + "mmX: " + str(abs_x_position_steps * 0.0025) +
        #       "; absY: " + str(abs_y_position_steps * 0.0025) +
        #       "; absZ: " + str(abs_z_position_steps * 0.0025))
        # print("")
        frameNumber += 1        # Счетчик номеров кадра, обрабатываемого в данный момент (нач. с нуля - НЕ МЕНЯТЬ!)


arr_prog.append('end')                  # добавляем метку конца программы


# записываем получившийся список в файл
ncm = open(f'{folder_source}{file_name}.ncm', 'w', encoding=encoding_val)
for line in arr_prog:
    ncm.write(line + '\n')
    count_frames += 1
ncm.close()

print(f'file name: {file_name}')
print('Finished')
print(f'{count_frames} lines in file ncm')
# печатаем получившуюся управляющую программу в консоли перед началом исполнения
# i_print = 0
# for line in arr_prog:
#     print(f'{i_print}: {line}')
#     i_print += 1
