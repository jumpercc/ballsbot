# ROS модуль для TCA9548

Переключатеть между группами из i2c устройств.

Можно читать данные с датчиков, i2c адреса которых конфликтуют.

Поддерживается до восьми устройств с одним адресом на одной TCA9548.

А, ввиду возможности задания адреса самой TCA9548, до 64ёх на одной i2c шине (через 8 TCA9548).