# FaraSenseSensor

Código do sensor FaraSense para placas de desenvolvimento ESP32 Bluetooth

Descrição: Aqui você encontrará um código completo de funcionamento de um Sensor de Corrente Não Invasivo 100A SCT-013 integrado a uma placa de desenvolvimento ESP 32 Bluetooth.

Nesse código você encontrará o uso da biblioteca Emon (padrão do sistema do Sensor SCT-013) comparado ao código de cálculo de IRMS customizado 

Esse código customizado ["calcIrms()"] colhe 4000 amostras calibrando sua leitura e aplicando o off-set senoidal promovendo uma leitura mais sensível do VA da corrente medida pelo sensor. Essa função procura utilizar a sensibilidade das portas 12 bits do ESP32 Bluetooth par auma leitura diferenciada, mais sensível e com uma granulação maior.

Código calibrado com a leitura de uma lâmpada incandescente de 1A de testes com Multímetro medindo a corrente presente na rede.

-- mais mudanças - 


