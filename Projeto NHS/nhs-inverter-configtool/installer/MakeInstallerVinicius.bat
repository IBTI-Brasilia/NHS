::@echo off
::Path para o WinDeployQt
set WD=C:\Qt\5.12.3\mingw73_64\bin.\windeployqt.exe
::Path para o BinaryCreator
set BC=C:\Qt\Tools\QtInstallerFramework\3.1\bin\binarycreator.exe
::Path para a versão release do programa
set RL=C:\Users\vinic\Documents\build-NHSInverterConfig-Desktop_Qt_5_12_3_MinGW_64_bit-Release\release
::Path para a pasta installer
set INST=C:\Users\vinic\Documents\nhs-inverter-configtool\installer
::Path para o icone do programa
set ICON=C:\Users\vinic\Documents\build-NHSInverterConfig-Desktop_Qt_5_12_3_MinGW_64_bit-Release\release\app.ico
::Path para 7-zip - Tem aspas pq "Program Files" tem um espaço :(
set sz="C:\Program Files (x86)\7-Zip\7z"
::Nome do instalador
set NAME=InverterConfigInstall_23_09_v3

set /p NAME=Qual o nome do arquivo ?: 
::windeployqt na pasta release
%WD% --compiler-runtime %RL%
::Copia o icone para a pasta release
copy %ICON% %RL%
::Zipa tudo com 7zip e manda para installer data
cd %RL%
%sz% a -r data *
cd %INST%
move %RL%\data.7z .\packages\org.qtproject.nhs.inverterconfig\data
::Gera o instalador
%BC% -c config\config.xml -p packages %NAME%.exe
PAUSE "Pausado"