# Склад автозавода

Мир `automotive` размером 72 × 48 м перенесён из AMR_sim вместе с синтетической
картой. В нём есть доки, стеллажи, оборудование и разметка; внешние модели
не требуются. Старт Burger — `(0, -20)` в координатах Gazebo.

Запуск только симуляции с метками и скрытым лидаром:

```bash
~/LW-2/SCRIPTS/S2_startSim.sh world:=automotive points_file:=$HOME/points_automotive.txt
```

Построение своей карты и запись точек:

```bash
~/LW-2/SCRIPTS/S2_startMapping.sh world:=automotive
```

В отдельных терминалах запустите Cartographer и teleop, как в README;
teleop передайте `~/points_automotive.txt`. Метки при записи скрыты, лидар
включён. Сохраняйте карту под отдельным именем `~/automotive_map`.

Полный запуск на своей карте:

```bash
~/LW-2/SCRIPTS/start_tb3.sh ~/automotive_map.yaml \
  --world automotive --points-file ~/points_automotive.txt
```

Для эталонной карты можно опустить первый аргумент; тогда используется
`maps/warehouse_automotive.yaml` из пакета `turtlebot3_gazebo`. Точки должны
быть в системе координат этой карты. Начальную позу задайте в RViz.
Малый склад остаётся стандартным при запуске без `--world`/`world:=`.
