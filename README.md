# espidf-arduino-more-wifi-buffs

Essentially https://github.com/pioarduino/platform-espressif32/tree/main/examples/espidf-arduino-blink with the option `CONFIG_ESP_WIFI_STATIC_TX_BUFFER` ([docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/kconfig.html#config-esp-wifi-tx-buffer)) cranked up from 8 to 64.

Also set `CONFIG_ESPTOOLPY_FLASHSIZE_8MB` instead of 16MB because that's what this chip has.