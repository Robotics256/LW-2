# Лабораторная работа № 2

## Основы цифрового моделирования производственных логистических процессов

Проект демонстрирует управление автономным мобильным роботом TurtleBot3 Burger в симуляторе Gazebo с использованием ROS 2 Humble. В ходе работы строится карта помещения, сохраняются целевые точки, запускается автономная навигация Nav2, а задания роботу передаются через MQTT.

## Цель работы

Изучить основы моделирования производственной логистики и освоить:

- запуск TurtleBot3 в Gazebo;
- ручное управление роботом;
- построение 2D-карты с помощью Cartographer;
- сохранение целевых и промежуточных точек;
- автономную навигацию с помощью Nav2;
- передачу и контроль заданий через MQTT;
- выполнение последовательных маршрутов доставки.

## Требования

- Ubuntu 22.04, в том числе в WSL2;
- ROS 2 Humble;
- Python 3 с библиотекой `paho-mqtt`;
- Gazebo Classic, RViz, Cartographer и Navigation2;
- доступ к интернету для работы с публичным MQTT-брокером;
- MQTT-клиент, например MQTTX, — при ручной отправке заданий.

## Структура проекта

```text
LW-2/
├── README.md
├── SCRIPTS/
│   ├── S1_installRos2.sh         # установка ROS 2 и сборка workspace
│   ├── S2_startSim.sh            # запуск мира TurtleBot3 в Gazebo
│   ├── teleop_keyboard_save.py   # ручное управление и запись точек
│   ├── mqtt_driver.py            # мост MQTT → Nav2
│   ├── sent_one_task.py          # отправка отдельного задания
│   ├── sent_many_task.py         # последовательная отправка заданий
│   └── navigate_waypoints.py     # навигация по списку точек
└── ros2_ws/                      # workspace ROS 2 с пакетами TurtleBot3
```

Все команды ниже выполняются из домашней директории, в которой находится папка `LW-2`:

## Установка

Скрипт устанавливает ROS 2 Humble, Python-зависимости и собирает workspace. Для его работы необходимы права `sudo` и подключение к интернету.

```bash
chmod +x ./LW-2/SCRIPTS/S1_installRos2.sh
./LW-2/SCRIPTS/S1_installRos2.sh
```

При повторной ручной сборке workspace используйте:

```bash
source /opt/ros/humble/setup.bash
cd ~/LW-2/ros2_ws
colcon build --symlink-install --executor sequential
```

## 1. Запуск симуляции

Быстрый запуск:

```bash
chmod +x ./LW-2/SCRIPTS/S2_startSim.sh
./LW-2/SCRIPTS/S2_startSim.sh
```

Либо запуск вручную:

```bash
export TURTLEBOT3_MODEL=burger
source ./LW-2/ros2_ws/install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py
```

Каждый следующий компонент запускайте в отдельной вкладке терминала, не закрывая Gazebo.

## 2. Ручное управление

```bash
export TURTLEBOT3_MODEL=burger
source ./LW-2/ros2_ws/install/setup.bash
python3 ./LW-2/SCRIPTS/teleop_keyboard_save.py
```

Клавиши управления:

| Клавиша | Действие |
|---|---|
| `W` / `X` | увеличить / уменьшить линейную скорость |
| `A` / `D` | увеличить / уменьшить угловую скорость |
| `S` или `Space` | остановить робота |
| `G` | остановиться и сохранить текущую позу под заданным именем |
| `Ctrl+C` | завершить программу |

Клавиши в программе обрабатываются в нижнем регистре. По умолчанию точки добавляются в `./points.txt`, то есть в текущую директорию терминала. Для однозначного сохранения в домашней директории можно передать путь явно:

```bash
python3 ./LW-2/SCRIPTS/teleop_keyboard_save.py ~/points.txt
```

Формат строки в файле: `имя, x, y, orientation_z, orientation_w`.

## 3. Построение и сохранение карты

Запустите Cartographer:

```bash
export TURTLEBOT3_MODEL=burger
source ./LW-2/ros2_ws/install/setup.bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```

С помощью программы ручного управления объедьте всё помещение. В RViz должны отобразиться стены, препятствия и доступные проходы. Затем сохраните карту:

```bash
source ./LW-2/ros2_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p use_sim_time:=true
```

Будут созданы файлы `~/map.pgm` и `~/map.yaml`. После сохранения остановите Cartographer сочетанием `Ctrl+C`.

## 4. Расстановка точек

Продолжая управлять роботом через `teleop_keyboard_save.py`, перемещайтесь в нужные позиции и нажимайте `G`. Введите уникальное имя точки и нажмите `Enter`.

Рекомендуемый набор имён:

- `parking` — место стоянки;
- `target1` и `target2` — зоны загрузки и выгрузки;
- `way_a`, `way_b` и другие `way_*` — промежуточные точки маршрутов.

Имена чувствительны к регистру и должны точно совпадать с именами, которые затем передаются в MQTT-заданиях. Файл `~/points.txt` необходим драйверу навигации.

## 5. Запуск автономной навигации и MQTT

Для работы системы одновременно нужны Gazebo, Navigation2 и MQTT-драйвер.

### Терминал 1 — Gazebo

```bash
./LW-2/SCRIPTS/S2_startSim.sh
```

### Терминал 2 — Navigation2

```bash
export TURTLEBOT3_MODEL=burger
source ./LW-2/ros2_ws/install/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true map:=$HOME/map.yaml
```

После появления карты в RViz задайте начальную позицию робота инструментом **2D Pose Estimate**, если локализация не установилась автоматически.

### Терминал 3 — MQTT-драйвер

Перед запуском при необходимости измените одинаковым образом параметры `WORKSPACE` и `MODULE_NAME` в файлах:

- `SCRIPTS/mqtt_driver.py`;
- `SCRIPTS/sent_one_task.py`;
- `SCRIPTS/sent_many_task.py`.

В проекте по умолчанию используются вариант 21, пространство `space_v21` и робот `amr1_v21`.

Запускайте драйвер из домашней директории: именно там он по умолчанию ищет `points.txt`.

```bash
cd ~
source /opt/ros/humble/setup.bash
source ./LW-2/ros2_ws/install/setup.bash
python3 ./LW-2/SCRIPTS/mqtt_driver.py
```

Используемые MQTT-параметры:

| Параметр | Значение по умолчанию |
|---|---|
| Брокер | `broker.emqx.io:1883` |
| Топик команд | `space_v21/devices/in/delivery/amr1_v21` |
| Топик статусов | `space_v21/devices/out/delivery/amr1_v21` |

Статусы задания: `0` — выполнено, `1` — выполняется, `2` — ошибка, `3` — робот занят и задание отклонено.

## 6. Отправка задания

### Через MQTTX

Подключитесь к `mqtt://broker.emqx.io:1883`, подпишитесь на топик статусов и опубликуйте в топик команд сообщение без флага `Retain`:

```json
{
  "Module": {
    "Name": "amr1_v21",
    "Type": "delivery",
    "UID": 12346,
    "Data": {
      "Task": {
        "Id": 67890,
        "Type": "Point",
        "Data": "parking"
      }
    }
  }
}
```

Для каждого нового сообщения изменяйте `UID` и `Task.Id`. Поле `Task.Data` должно содержать имя существующей точки из `points.txt`.

### Через Python

Отправка одного задания:

```bash
python3 ./LW-2/SCRIPTS/sent_one_task.py
```

Скрипт отправляет команды подряд и не ждёт завершения предыдущего перемещения. Перед запуском замените строку `"..."` в конце файла на существующую точку.

Последовательный маршрут с ожиданием статуса `0` после каждой точки:

```bash
python3 ./LW-2/SCRIPTS/sent_many_task.py
```

В `sent_many_task.py` также необходимо заменить `"..."` на имя сохранённой точки. Тайм-аут ожидания одного задания в примере равен 300 секундам.

## Логистические процессы

На основе `sent_many_task.py` требуется реализовать два процесса:

1. Маршрут A: `parking → target1 → way_a → target2 → parking`.
2. Маршрут B: `parking → target2 → way_b → target1 → parking`.

После прибытия в `target1` и `target2` программа должна выводить в консоль сообщение о загрузке или выгрузке груза. Выбор маршрута можно организовать через консольный ввод. Фактические имена точек в коде должны соответствовать вашему файлу `points.txt`.

## Что включить в отчёт

- окно Gazebo после запуска симуляции;
- полученную карту в RViz;
- содержимое `points.txt`;
- три примера перемещения по MQTT и принятые статусы;
- результат (снимок экрана консоли) запуска `sent_one_task.py`;
- код последовательного маршрута и положения робота в его точках;
- реализацию и результаты маршрутов A и B;
- выводы по работе.

Все запущенные процессы завершаются сочетанием `Ctrl+C` в соответствующих терминалах.
