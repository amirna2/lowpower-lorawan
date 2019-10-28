# Low power LoRaWAN End Device 
This repository contains the necessary documents and sources to implement a LoRaWAN end-device based on a 32-bit ARM Cortex M0+ ATSAMD21G18A Arduino compatible development board from Rocket Scream: https://www.rocketscream.com/blog/product/mini-ultra-pro-v3-with-radio/

The board integrates with a HopeRF RFM95W LoRaWAN chipset https://www.hoperf.com/modules/lora/RFM95.html

The LoRa end device prototype is designed for monitoring outdoor environmental conditions and be part of a private LoRaWAN Network managed by open source LoRa Server https://www.loraserver.io/

The server provides an Application Server that manages the network and forwards the sensor data to an InfluxDB/Grafana visualization dashboard as a proof of concept.

