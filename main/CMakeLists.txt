idf_component_register(SRCS "main_ide.cpp" "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES arduino LoRa PubSubClient DHT)