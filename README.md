# FaraSenseSensor

### Código do sensor FaraSense para placas de desenvolvimento ESP32 Bluetooth

* Descrição: Aqui você encontrará um código completo de funcionamento de um Sensor de Corrente Não Invasivo 100A SCT-013 integrado a uma placa de desenvolvimento ESP 32 Bluetooth de 3.3v.

Nesse código você encontrará o uso da biblioteca Emon (padrão do sistema do Sensor SCT-013) comparado ao código de cálculo de IRMS customizado. Essa biblioteca (Emon) se comporta de maneira um pouco fora dos padrões de leitura da onda senoidal que revela os valores de corrente do circuito. Por isso, foi efetuado o cálculo independente da biblioteca diretamente no código, para se conseguir valores corretos com o redimencionamento do circuito com capacitor e resistores de acordo com a tensão 3.3v.

Esse código customizado "calcIrms()" colhe 4000 amostras calibrando sua leitura e aplicando o off-set senoidal promovendo uma leitura mais sensível do VA da corrente medida pelo sensor. Essa função procura utilizar a sensibilidade das portas 12 bits do ESP32 Bluetooth par auma leitura diferenciada, mais sensível e com uma granulação maior.

Código calibrado com a leitura de uma lâmpada incandescente de 1A de testes com Multímetro medindo a corrente presente na rede.

## Instalação da Placa ESP32

Para instalação da placa de desenvolvimento na IDE Arduino, você vai precisar copiar o link de propriedades da placa para o campo "Additional Board Manager URLs" dentro de Propriedades e, logo em seguida, baixar o módulo de desenvolvimento esp32 dentro de Gerenciador de Placas.

Link: [https://dl.espressif.com/dl/package_esp32_index.json](https://dl.espressif.com/dl/package_esp32_index.json)

## Instalação das Bibliotecas (Importante!)

Para instalação das bibliotecas, você deve copiar o conteúdo da pasta Biblioteca após a instalação da placa ESP32 no Gerenciador de Placas do Arduíno.

Copie o conteúdo para alguma pasta de libraries de sua preferência. Lembre-se: Pastas duplicadas em libraries diferentes no Arduino IDE faz com que o compilador não consiga escolher pela concorrência, gerando erros vermelhos no console. Remova as pastas excedentes nas demais pastas e vá compilando até a IDE concluir todo o processo.

Qualquer dúvida no uso, entre em contato no e-mail: [tassiolucas@live.com](tassiolucas@live.com)


